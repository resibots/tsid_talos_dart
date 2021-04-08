#include <Eigen/Core>
#include <iomanip>
#include <map>
#include <memory>
#include <utility>
#include <vector>

/* Pinocchio !!!! NEED TO BE INCLUDED BEFORE BOOST*/
#include <pinocchio/algorithm/joint-configuration.hpp> // integrate
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <boost/filesystem.hpp>
#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/utils.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/utils/stop-watch.hpp>

// #include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>

#include "inria_wbc/controllers/talos_pos_tracker.hpp"
#include "inria_wbc/controllers/tasks.hpp"
#include "inria_wbc/stabilizers/stabilizer.hpp"
#include "inria_wbc/utils/utils.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace controllers {
        static Register<TalosPosTracker> __talos_pos_tracking("talos-pos-tracker");

        TalosPosTracker::TalosPosTracker(const Params& params) : PosTracker(params)
        {
            parse_configuration_yaml(params.sot_config_path);
            if (verbose_)
                std::cout << "Talos pos tracker initialized" << std::endl;
        }

        void TalosPosTracker::parse_configuration_yaml(const std::string& sot_config_path)
        {
            // init stabilizer
            {
                YAML::Node c = IWBC_CHECK(YAML::LoadFile(sot_config_path)["CONTROLLER"]["stabilizer"]);
                _use_stabilizer = IWBC_CHECK(c["activated"].as<bool>());
                _activate_zmp = IWBC_CHECK(c["activate_zmp"].as<bool>());
                _torso_max_roll = IWBC_CHECK(c["torso_max_roll"].as<double>());

                //set the _torso_max_roll in the bounds for safety
                auto names = robot_->model().names;
                names.erase(names.begin(), names.begin() + names.size() - robot_->na());
                auto q_lb = robot_->model().lowerPositionLimit.tail(robot_->na());
                auto q_ub = robot_->model().upperPositionLimit.tail(robot_->na());
                std::vector<std::string> to_limit = {"leg_left_2_joint", "leg_right_2_joint"};

                for (auto& n : to_limit) {
                    IWBC_ASSERT(std::find(names.begin(), names.end(), n) != names.end(), "Talos should have ", n);
                    auto id = std::distance(names.begin(), std::find(names.begin(), names.end(), n));

                    IWBC_ASSERT((q_lb[id] <= q0_.tail(robot_->na()).transpose()[id]) && (q_ub[id] >= q0_.tail(robot_->na()).transpose()[id]), "Error in bounds, the torso limits are not viable");

                    q_lb[id] = q0_.tail(robot_->na()).transpose()[id] - _torso_max_roll;
                    q_ub[id] = q0_.tail(robot_->na()).transpose()[id] + _torso_max_roll;
                }

                bound_task()->setPositionBounds(q_lb, q_ub);

                //get stabilizers gains
                _com_gains.resize(6);
                _ankle_gains.resize(6);
                _ffda_gains.resize(3);
                _zmp_p.resize(6);
                _zmp_d.resize(6);
                _zmp_w.resize(6);

                _com_gains.setZero();
                _ankle_gains.setZero();
                _ffda_gains.setZero();
                _zmp_p.setZero();
                _zmp_d.setZero();
                _zmp_w.setZero();

                IWBC_ASSERT(IWBC_CHECK(c["com"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in p for the com stabilizer");
                IWBC_ASSERT(IWBC_CHECK(c["ankle"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in d for the ankle stabilizer");
                IWBC_ASSERT(IWBC_CHECK(c["ffda"].as<std::vector<double>>()).size() == 3, "you need 6 coefficient in p for the ffda stabilizer");
                IWBC_ASSERT(IWBC_CHECK(c["zmp_p"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in p for the zmp stabilizer");
                IWBC_ASSERT(IWBC_CHECK(c["zmp_d"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in d for the zmp stabilizer");
                IWBC_ASSERT(IWBC_CHECK(c["zmp_w"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in w for the zmp stabilizer");

                _com_gains = Eigen::VectorXd::Map(IWBC_CHECK(c["com"].as<std::vector<double>>()).data(), _com_gains.size());
                _ankle_gains = Eigen::VectorXd::Map(IWBC_CHECK(c["ankle"].as<std::vector<double>>()).data(), _ankle_gains.size());
                _ffda_gains = Eigen::VectorXd::Map(IWBC_CHECK(c["ffda"].as<std::vector<double>>()).data(), _ffda_gains.size());
                _zmp_p = Eigen::VectorXd::Map(IWBC_CHECK(c["zmp_p"].as<std::vector<double>>()).data(), _zmp_p.size());
                _zmp_d = Eigen::VectorXd::Map(IWBC_CHECK(c["zmp_d"].as<std::vector<double>>()).data(), _zmp_d.size());
                _zmp_w = Eigen::VectorXd::Map(IWBC_CHECK(c["zmp_w"].as<std::vector<double>>()).data(), _zmp_w.size());

                auto history = c["filter_size"].as<int>();
                _cop_estimator.set_history_size(history);

                _lf_force_filtered.setZero();
                _rf_force_filtered.setZero();
                _lf_force_filter = std::make_shared<estimators::MovingAverageFilter>(3, history); //force data of size 3
                _rf_force_filter = std::make_shared<estimators::MovingAverageFilter>(3, history); //force data of size 3

                _lf_torque_filtered.setZero();
                _rf_torque_filtered.setZero();
                _lf_torque_filter = std::make_shared<estimators::MovingAverageFilter>(3, history); //force data of size 3
                _rf_torque_filter = std::make_shared<estimators::MovingAverageFilter>(3, history); //force data of size 3
            }

            // init collision detection
            {
                YAML::Node c = IWBC_CHECK(YAML::LoadFile(sot_config_path)["CONTROLLER"]["collision_detection"]);
                _use_torque_collision_detection = IWBC_CHECK(c["activated"].as<bool>());
                auto filter_window_size = IWBC_CHECK(c["filter_size"].as<int>());
                auto max_invalid = IWBC_CHECK(c["max_invalid"].as<int>());

                _torque_collision_joints = {
                    "leg_left_1_joint", "leg_left_2_joint", "leg_left_3_joint", "leg_left_4_joint", "leg_left_5_joint", "leg_left_6_joint",
                    "leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint", "leg_right_6_joint",
                    "torso_1_joint", "torso_2_joint",
                    "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint",
                    "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint"};

                auto filtered_dof_names = this->all_dofs(true); // filter out mimics
                for (const auto& joint : _torque_collision_joints) {
                    auto it = std::find(filtered_dof_names.begin(), filtered_dof_names.end(), joint);
                    _torque_collision_joints_ids.push_back(std::distance(filtered_dof_names.begin(), it));
                }

                _torque_collision_threshold.resize(_torque_collision_joints.size());
                _torque_collision_threshold << 3.5e+05, 3.9e+05, 2.9e+05, 4.4e+05, 5.7e+05, 2.4e+05,
                    3.5e+05, 3.9e+05, 2.9e+05, 4.4e+05, 5.7e+05, 2.4e+05,
                    1e+01, 1e+01,
                    1e+01, 1e+01, 1e+01, 1e+01,
                    1e+01, 1e+01, 1e+01, 1e+01;

                // update thresholds from file (if any)
                if (c["thresholds"]) {
                    auto path = boost::filesystem::path(sot_config_path).parent_path();
                    auto p_thresh = IWBC_CHECK(path / boost::filesystem::path(c["thresholds"].as<std::string>()));
                    parse_collision_thresholds(p_thresh.string());
                }

                _torque_collision_filter = std::make_shared<estimators::MovingAverageFilter>(_torque_collision_joints.size(), filter_window_size);

                _torque_collision_detection = safety::TorqueCollisionDetection(_torque_collision_threshold);
                _torque_collision_detection.set_max_consecutive_invalid(max_invalid);
                _torque_collision_detection.set_filter(_torque_collision_filter);
            }

            if (verbose_) {
                std::cout << "Stabilizer:" << _use_stabilizer << std::endl;
                std::cout << "com:" << _com_gains.transpose() << std::endl;
                std::cout << "ankle:" << _ankle_gains.transpose() << std::endl;
                std::cout << "ffda:" << _ffda_gains.transpose() << std::endl;
                std::cout << "Zmp:" << _activate_zmp << std::endl;
                std::cout << "zmp_p:" << _zmp_p.transpose() << std::endl;
                std::cout << "zmp_d:" << _zmp_d.transpose() << std::endl;
                std::cout << "zmp_w:" << _zmp_w.transpose() << std::endl;

                std::cout << "Collision detection:" << _use_torque_collision_detection << std::endl;
                std::cout << "with thresholds" << std::endl;
                for (size_t id = 0; id < _torque_collision_joints.size(); ++id)
                    std::cout << _torque_collision_joints[id] << ": " << _torque_collision_threshold(id) << std::endl;
            }
        }

        void TalosPosTracker::parse_collision_thresholds(const std::string& config_path)
        {
            YAML::Node config = IWBC_CHECK(YAML::LoadFile(config_path));
            for (size_t jid = 0; jid < _torque_collision_joints.size(); ++jid) {
                std::string joint = _torque_collision_joints[jid];
                if (config[joint])
                    _torque_collision_threshold(jid) = IWBC_CHECK(config[joint].as<double>());
            }

            return;
        }

        void TalosPosTracker::update(const SensorData& sensor_data)
        {
            std::map<std::string, tsid::trajectories::TrajectorySample> contact_sample_ref;
            std::map<std::string, pinocchio::SE3> contact_se3_ref;
            std::map<std::string, Eigen::Matrix<double, 6, 1>> contact_force_ref;

            auto ac = activated_contacts_;
            for (auto& contact_name : ac) {
                pinocchio::SE3 se3;
                auto contact_pos = contact(contact_name)->getMotionTask().getReference().pos;
                tsid::math::vectorToSE3(contact_pos, se3);
                contact_se3_ref[contact_name] = se3;
                contact_sample_ref[contact_name] = contact(contact_name)->getMotionTask().getReference();
                contact_force_ref[contact_name] = contact(contact_name)->getForceReference();
            }
            auto com_ref = com_task()->getReference();
            auto left_ankle_ref = get_full_se3_ref("lf");
            auto right_ankle_ref = get_full_se3_ref("rf");
            auto torso_ref = get_full_se3_ref("torso");

            if (_use_stabilizer) {
                IWBC_ASSERT(sensor_data.find("lf_torque") != sensor_data.end(), "the stabilizer needs the LF torque");
                IWBC_ASSERT(sensor_data.find("rf_torque") != sensor_data.end(), "the stabilizer needs the RF torque");
                IWBC_ASSERT(sensor_data.find("velocity") != sensor_data.end(), "the stabilizer needs the velocity");

                // estimate the CoP / ZMP
                bool cop_ok = _cop_estimator.update(com_ref.pos.head(2),
                    model_joint_pos("leg_left_6_joint").translation(),
                    model_joint_pos("leg_right_6_joint").translation(),
                    sensor_data.at("lf_torque"), sensor_data.at("lf_force"),
                    sensor_data.at("rf_torque"), sensor_data.at("rf_force"));

                // if the foot is on the ground
                if (sensor_data.at("lf_force").norm() > _cop_estimator.fmin()) {
                    _lf_force_filtered = _lf_force_filter->filter(sensor_data.at("lf_force"));
                    _lf_torque_filtered = _lf_torque_filter->filter(sensor_data.at("lf_torque"));
                }
                else {
                    _lf_force_filtered.setZero();
                    _lf_torque_filtered.setZero();
                }
                if (sensor_data.at("rf_force").norm() > _cop_estimator.fmin()) {
                    _rf_force_filtered = _rf_force_filter->filter(sensor_data.at("rf_force"));
                    _rf_torque_filtered = _rf_torque_filter->filter(sensor_data.at("rf_torque"));
                }
                else {
                    _rf_force_filtered.setZero();
                    _rf_force_filtered.setZero();
                }

                tsid::trajectories::TrajectorySample lf_se3_sample, lf_contact_sample, rf_se3_sample, rf_contact_sample;
                tsid::trajectories::TrajectorySample com_sample, torso_sample;
                tsid::trajectories::TrajectorySample model_current_com = stabilizer::data_to_sample(tsid_->data());

                // com_admittance
                if (cop_ok
                    && !std::isnan(_cop_estimator.cop_filtered()(0))
                    && !std::isnan(_cop_estimator.cop_filtered()(1))) {

                    stabilizer::com_admittance(dt_, _com_gains, _cop_estimator.cop_filtered(), model_current_com, com_ref, com_sample);
                    set_com_ref(com_sample);
                }

                //zmp admittance
                if (cop_ok
                    && !std::isnan(_cop_estimator.cop_filtered()(0))
                    && !std::isnan(_cop_estimator.cop_filtered()(1))
                    && _activate_zmp) {

                    double M = pinocchio_total_model_mass();
                    Eigen::Matrix<double, 6, 1> left_fref, right_fref;

                    stabilizer::zmp_distributor_admittance(dt_, _zmp_p, _zmp_d, M, contact_se3_ref, ac, _cop_estimator.cop_filtered(), model_current_com, left_fref, right_fref);

                    contact("contact_lfoot")->Contact6d::setRegularizationTaskWeightVector(_zmp_w);
                    contact("contact_rfoot")->Contact6d::setRegularizationTaskWeightVector(_zmp_w);
                    contact("contact_lfoot")->Contact6d::setForceReference(left_fref);
                    contact("contact_rfoot")->Contact6d::setForceReference(right_fref);
                }

                // left ankle_admittance
                if (cop_ok
                    && !std::isnan(_cop_estimator.lcop_filtered()(0))
                    && !std::isnan(_cop_estimator.lcop_filtered()(1))
                    && std::find(ac.begin(), ac.end(), "contact_lfoot") != ac.end()) {

                    stabilizer::ankle_admittance(dt_, _ankle_gains, _cop_estimator.lcop_filtered(), get_full_se3_ref("lf"), contact_sample_ref["contact_lfoot"], lf_se3_sample, lf_contact_sample);
                    set_se3_ref(lf_se3_sample, "lf");
                    contact("contact_lfoot")->setReference(lf_contact_sample);
                }

                //right ankle_admittance
                if (cop_ok
                    && !std::isnan(_cop_estimator.rcop_filtered()(0))
                    && !std::isnan(_cop_estimator.rcop_filtered()(1))
                    && std::find(ac.begin(), ac.end(), "contact_rfoot") != ac.end()) {

                    stabilizer::ankle_admittance(dt_, _ankle_gains, _cop_estimator.rcop_filtered(), get_full_se3_ref("rf"), contact_sample_ref["contact_rfoot"], rf_se3_sample, rf_contact_sample);
                    set_se3_ref(rf_se3_sample, "rf");
                    contact("contact_rfoot")->setReference(rf_contact_sample);
                }

                //foot force difference admittance
                if (activated_contacts_forces_.find("contact_rfoot") != activated_contacts_forces_.end()
                    && activated_contacts_forces_.find("contact_lfoot") != activated_contacts_forces_.end()
                    && _lf_force_filter->data_ready()
                    && _rf_force_filter->data_ready()) {

                    //normal force of the contacts from tsid
                    double lf_normal_force = contact("contact_lfoot")->Contact6d::getNormalForce(activated_contacts_forces_["contact_lfoot"]);
                    double rf_normal_force = contact("contact_rfoot")->Contact6d::getNormalForce(activated_contacts_forces_["contact_rfoot"]);

                    stabilizer::foot_force_difference_admittance(dt_, _ffda_gains, lf_normal_force, rf_normal_force, _lf_force_filtered, _rf_force_filtered, get_full_se3_ref("torso"), torso_sample);
                    set_se3_ref(torso_sample, "torso");
                }
            }

            if (_use_torque_collision_detection) {
                IWBC_ASSERT(sensor_data.find("joints_torque") != sensor_data.end(), "torque collision detection requires torque sensor data");
                IWBC_ASSERT(sensor_data.at("joints_torque").size() == _torque_collision_joints.size(), "torque sensor data has a wrong size. call torque_sensor_joints() for needed values");

                auto tsid_tau = utils::slice_vec(this->tau(), _torque_collision_joints_ids);
                _collision_detected = (false == _torque_collision_detection.check(tsid_tau, sensor_data.at("joints_torque")));
            }

            // solve everything
            _solve();

            // set the CoM back (useful if the behavior does not the set the ref at each timestep)
            if (_use_stabilizer) {
                set_com_ref(com_ref);
                set_se3_ref(left_ankle_ref, "lf");
                set_se3_ref(right_ankle_ref, "rf");
                set_se3_ref(torso_ref, "torso");

                for (auto& contact_name : ac) {
                    contact(contact_name)->setReference(contact_sample_ref[contact_name]);
                    contact(contact_name)->Contact6d::setForceReference(contact_force_ref[contact_name]);
                }
            }
        }

        void TalosPosTracker::clear_collision_detection()
        {
            _torque_collision_detection.reset();
            _torque_collision_filter->reset();
            _collision_detected = false;
        }

    } // namespace controllers
} // namespace inria_wbc
