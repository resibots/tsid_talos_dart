/* Pinocchio !!!! NEED TO BE INCLUDED BEFORE BOOST*/
#include <pinocchio/algorithm/joint-configuration.hpp> // integrate
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "Eigen/Core"
#include "map"
#include "vector"
#include <iomanip>
#include <memory>
#include <utility>

#include <tsid/contacts/contact-6d.hpp>
#include <tsid/contacts/contact-point.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/math/utils.hpp>
#include <tsid/robots/fwd.hpp>
#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/utils.hpp>
#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/trajectories/trajectory-base.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>
#include <tsid/trajectories/trajectory-se3.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/utils/stop-watch.hpp>

#include "inria_wbc/controllers/talos_pos_tracking.hpp"

using namespace tsid;
using namespace tsid::trajectories;
using namespace tsid::math;
using namespace tsid::contacts;
using namespace tsid::tasks;
using namespace tsid::solvers;
using namespace tsid::robots;
using namespace std;
using namespace inria_wbc::utils;
namespace inria_wbc {
    namespace controllers {

        TalosPosTracking::TalosPosTracking(const Params& params) : TalosBaseController(params)
        {
            if (!params.sot_config_path.empty())
                parse_configuration_yaml(params.sot_config_path);

            set_stack_configuration();
            init_references();
        }

        void TalosPosTracking::set_default_opt_params(std::map<std::string, double>& p)
        {
            p["w_com"] = 10.0; //# weight of center of mass task
            p["w_posture"] = 0.75; //# weight of joint posture task
            p["w_forceRef_feet"] = 1e-3; //# weight of force regularization task
            p["w_forceRef_hands"] = 1e-3; //# weight of force regularization task
            p["w_floatingb"] = 20.0; //# weight of floatingb task
            p["w_velocity"] = 1.0; //# weight of velocity bounds
            p["w_rh"] = 10.0; //# weight of right hand  task
            p["w_lh"] = 10.0; //# weight of left hand  task
            p["w_rf"] = 1.0; //# weight of right foot  task
            p["w_lf"] = 1.0; //# weight of left foot  task
            p["w_torso"] = 1000.0; //# weight of torso task

            p["kp_contact"] = 30.0; //# proportional gain of contact constraint
            p["kp_com"] = 3000.0; //# proportional gain of center of mass task
            p["kp_posture"] = 30.0; //# proportional gain of joint posture task
            p["kp_floatingb"] = 3000.0; //# proportional gain of floatingb task
            p["kp_rh"] = 300.0; //# proportional gain of right hand task
            p["kp_lh"] = 300.0; //# proportional gain of left hand task
            p["kp_rf"] = 30.0; //# proportional gain of right foot task
            p["kp_lf"] = 30.0; //# proportional gain of left foot task
            p["kp_torso"] = 30.0; //# proportional gain of the torso task
        }

        void TalosPosTracking::parse_configuration_yaml(const std::string& sot_config_path)
        {
            std::ifstream yamlConfigFile(sot_config_path);
            if (!yamlConfigFile.good()) {
                if (verbose_) {
                    std::cout << sot_config_path << " not found" << std::endl;
                    std::cout << "Taking default stack of tasks weights" << std::endl;
                }
            }
            else {
                if (verbose_)
                    std::cout << "Taking the stack of tasks parameters in " << sot_config_path << std::endl;

                // for the opt_params (task weight and gains) we keep the value if we have it
                // if not found, we look at the YAML file
                // if not found, we take the default value from the map (set_defautlt_opt_params)
                opt_params_t p;
                set_default_opt_params(p);
                YAML::Node config = YAML::LoadFile(sot_config_path);
                for (auto& x : p)
                    if (params_.opt_params.find(x.first) == params_.opt_params.end())
                        if (!parse(params_.opt_params[x.first], x.first, config, verbose_))
                            params_.opt_params[x.first] = p[x.first];

                // stabilizer
                inria_wbc::utils::parse(_use_stabilizer, "activated", config, false, "STABILIZER");
                inria_wbc::utils::parse(_stabilizer_p(0), "p_x", config, false, "STABILIZER");
                inria_wbc::utils::parse(_stabilizer_p(1), "p_y", config, false, "STABILIZER");
                inria_wbc::utils::parse(_stabilizer_d(0), "d_x", config, false, "STABILIZER");
                inria_wbc::utils::parse(_stabilizer_d(1), "d_y", config, false, "STABILIZER");
                int history = _cop_estimator.history_size();
                inria_wbc::utils::parse(history, "filter_size", config, false, "STABILIZER");
                _cop_estimator.set_history_size(history);

                if (verbose_) {
                    std::cout << "Stabilizer:" << _use_stabilizer << std::endl;
                    std::cout << "P:" << _stabilizer_p.transpose() << std::endl;
                    std::cout << "D:" << _stabilizer_d.transpose() << std::endl;
                }
            }
        }

        void TalosPosTracking::set_stack_configuration()
        {

            const opt_params_t& p = params_.opt_params;
            if (verbose_)
                for (auto& x : p)
                    std::cout << x.first << " => " << params_.opt_params[x.first] << std::endl;

            ////////////////////Gather Initial Pose //////////////////////////////////////
            q_tsid_ = robot_->model().referenceConfigurations["pal_start"];
            Eigen::Quaterniond quat(q_tsid_(6), q_tsid_(3), q_tsid_(4), q_tsid_(5));
            Eigen::AngleAxisd aaxis(quat);
            q0_ << q_tsid_.head(3), aaxis.angle() * aaxis.axis(), q_tsid_.tail(robot_->na()); //q_tsid_ is of size 37 (pos+quat+nactuated)

            ////////////////////Create the inverse-dynamics formulation///////////////////
            tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);

            ////////////////////Create an HQP solver /////////////////////////////////////
            using solver_t = std::shared_ptr<tsid::solvers::SolverHQPBase>;
            solver_ = solver_t(SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST, "solver-eiquadprog"));
            solver_->resize(tsid_->nVar(), tsid_->nEq(), tsid_->nIn());

            ////////////////////Compute Problem Data at init /////////////////////////////
            const uint nv = robot_->nv();
            tsid_->computeProblemData(dt_, q_tsid_, Vector::Zero(nv));
            const pinocchio::Data& data = tsid_->data();

            ////////////////////Compute Tasks, Bounds and Contacts ///////////////////////
            ////////// Add the contacts
            Matrix3x contact_points_(3, 4);
            contact_points_ << -lxn_, -lxn_, +lxp_, +lxp_,
                -lyn_, +lyp_, -lyn_, +lyp_,
                lz_, lz_, lz_, lz_;

            contactRF_ = std::make_shared<Contact6d>("contact_rfoot", *robot_, rf_frame_name_,
                contact_points_, contactNormal_,
                mu_, fMin_, fMax_);
            contactRF_->Kp(p.at("kp_contact") * Vector::Ones(6));
            contactRF_->Kd(2.0 * contactRF_->Kp().cwiseSqrt());
            contact_rf_ref_ = robot_->position(data, robot_->model().getJointId(rf_frame_name_));
            contactRF_->setReference(contact_rf_ref_);
            tsid_->addRigidContact(*contactRF_, p.at("w_forceRef_feet"));

            contactLF_ = std::make_shared<Contact6d>("contact_lfoot", *robot_, lf_frame_name_,
                contact_points_, contactNormal_,
                mu_, fMin_, fMax_);
            contactLF_->Kp(p.at("kp_contact") * Vector::Ones(6));
            contactLF_->Kd(2.0 * contactLF_->Kp().cwiseSqrt());
            contact_lf_ref_ = robot_->position(data, robot_->model().getJointId(lf_frame_name_));
            contactLF_->setReference(contact_lf_ref_);
            tsid_->addRigidContact(*contactLF_, p.at("w_forceRef_feet"));

            ////////// Add the com task
            com_task_ = std::make_shared<TaskComEquality>("task-com", *robot_);
            com_task_->Kp(p.at("kp_com") * Vector::Ones(3));
            com_task_->Kd(2.0 * com_task_->Kp().cwiseSqrt());
            tsid_->addMotionTask(*com_task_, p.at("w_com"), 1);

            ////////// Add the posture task
            posture_task_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);
            posture_task_->Kp(p.at("kp_posture") * Vector::Ones(nv - 6));
            posture_task_->Kd(2.0 * posture_task_->Kp().cwiseSqrt());
            Vector mask_post = Vector::Ones(nv - 6);
            // for(int i =0; i < 11; i++){
            //   mask_post[i] = 0;
            // }
            posture_task_->setMask(mask_post);
            tsid_->addMotionTask(*posture_task_, p.at("w_posture"), 1);

            ////////// Vertical torso task (try to keep it vertical)
            vert_torso_task_ = std::make_shared<TaskSE3Equality>("task-vert-torso", *robot_, torso_frame_name_);
            vert_torso_task_->Kp(p.at("kp_torso") * Vector::Ones(6));
            vert_torso_task_->Kd(2.0 * vert_torso_task_->Kp().cwiseSqrt());
            Vector mask_torso = Vector::Zero(6);
            mask_torso[4] = 1; // only constrain the pitch of the body
            mask_torso[3] = 1; // only constrain the roll of the body

            vert_torso_task_->setMask(mask_torso);
            tsid_->addMotionTask(*vert_torso_task_, p.at("w_torso"), 1);

            ////////// Add the floatingb task
            floatingb_task_ = std::make_shared<TaskSE3Equality>("task-floatingb", *robot_, fb_joint_name_);
            floatingb_task_->Kp(p.at("kp_floatingb") * Vector::Ones(6));
            floatingb_task_->Kd(2.0 * floatingb_task_->Kp().cwiseSqrt());
            Vector mask_floatingb = Vector::Ones(6);
            for (int i = 0; i < 3; i++) {
                mask_floatingb[i] = 0; // DO NOT CONSTRAIN floatingb POSITION IN SOT
            }
            floatingb_task_->setMask(mask_floatingb);
            tsid_->addMotionTask(*floatingb_task_, p.at("w_floatingb"), 1);

            ////////// Add the left hand  task
            lh_task_ = std::make_shared<TaskSE3Equality>("task-lh", *robot_, "gripper_left_joint");
            lh_task_->Kp(p.at("kp_lh") * Vector::Ones(6));
            lh_task_->Kd(2.0 * lh_task_->Kp().cwiseSqrt());
            Vector mask_lh = Vector::Ones(6);
            for (int i = 3; i < 6; i++) {
                mask_lh[i] = 0; // DO NOT CONSTRAIN HAND ORIENTATION IN SOT
            }
            lh_task_->setMask(mask_lh);
            tsid_->addMotionTask(*lh_task_, p.at("w_lh"), 1);

            ////////// Add the right hand task
            rh_task_ = std::make_shared<TaskSE3Equality>("task-rh", *robot_, "gripper_right_joint");
            rh_task_->Kp(p.at("kp_rh") * Vector::Ones(6));
            rh_task_->Kd(2.0 * rh_task_->Kp().cwiseSqrt());
            Vector mask_rh = Vector::Ones(6);
            for (int i = 3; i < 6; i++) {
                mask_rh[i] = 0; // DO NOT CONSTRAIN HAND ORIENTATION IN SOT
            }
            rh_task_->setMask(mask_rh);
            tsid_->addMotionTask(*rh_task_, p.at("w_rh"), 1);

            ////////// Add the left foot  task
            lf_task_ = std::make_shared<TaskSE3Equality>("task-lf", *robot_, "leg_left_6_joint");
            lf_task_->Kp(p.at("kp_lf") * Vector::Ones(6));
            lf_task_->Kd(2.0 * lf_task_->Kp().cwiseSqrt());
            Vector maskLf = Vector::Ones(6);
            lf_task_->setMask(maskLf);
            tsid_->addMotionTask(*lf_task_, p.at("w_lf"), 1);

            ////////// Add the right foot  task
            rf_task_ = std::make_shared<TaskSE3Equality>("task-rf", *robot_, "leg_right_6_joint");
            rf_task_->Kp(p.at("kp_rf") * Vector::Ones(6));
            rf_task_->Kd(2.0 * rf_task_->Kp().cwiseSqrt());
            Vector maskRf = Vector::Ones(6);
            rf_task_->setMask(maskRf);
            tsid_->addMotionTask(*rf_task_, p.at("w_rf"), 1);

            ////////// Add the position, velocity and acceleration limits
            bounds_task_ = std::make_shared<TaskJointPosVelAccBounds>("task-posVelAcc-bounds", *robot_, dt_, verbose_);
            dq_max_ = robot_->model().velocityLimit.tail(robot_->na());
            ddq_max_ = dq_max_ / dt_;
            bounds_task_->setVelocityBounds(dq_max_);
            bounds_task_->setAccelerationBounds(ddq_max_);
            q_lb_ = robot_->model().lowerPositionLimit.tail(robot_->na());
            q_ub_ = robot_->model().upperPositionLimit.tail(robot_->na());
            bounds_task_->setPositionBounds(q_lb_, q_ub_);
            tsid_->addMotionTask(*bounds_task_, p.at("w_velocity"), 0); //add pos vel acc bounds
        }

        void TalosPosTracking::init_references()
        {
            auto com_init = robot_->com(tsid_->data());
            auto posture_init = q_tsid_.tail(robot_->na());
            auto floatingb_init = robot_->position(tsid_->data(), robot_->model().getJointId(fb_joint_name_));
            auto lf_init = robot_->position(tsid_->data(), robot_->model().getJointId("leg_left_6_joint"));
            auto rf_init = robot_->position(tsid_->data(), robot_->model().getJointId("leg_right_6_joint"));
            auto lh_init = robot_->position(tsid_->data(), robot_->model().getJointId("gripper_left_joint"));
            auto rh_init = robot_->position(tsid_->data(), robot_->model().getJointId("gripper_right_joint"));
            auto torso_init = robot_->framePosition(tsid_->data(), robot_->model().getFrameId(torso_frame_name_));

            traj_com_ = std::make_shared<TrajectoryEuclidianConstant>("traj_com", com_init);
            traj_posture_ = std::make_shared<TrajectoryEuclidianConstant>("traj_posture", posture_init);
            traj_floatingb_ = std::make_shared<TrajectorySE3Constant>("traj_floatingb", floatingb_init);
            traj_lf_ = std::make_shared<TrajectorySE3Constant>("traj_lf", lf_init);
            traj_rf_ = std::make_shared<TrajectorySE3Constant>("traj_rf", rf_init);
            traj_lh_ = std::make_shared<TrajectorySE3Constant>("traj_lh", lh_init);
            traj_rh_ = std::make_shared<TrajectorySE3Constant>("traj_rh", rh_init);
            traj_torso_ = std::make_shared<TrajectorySE3Constant>("traj_torso", torso_init);

            TrajectorySample sample_com = traj_com_->computeNext();
            TrajectorySample sample_posture = traj_posture_->computeNext();
            TrajectorySample sample_floatingb = traj_floatingb_->computeNext();
            TrajectorySample sample_lf = traj_lf_->computeNext();
            TrajectorySample sample_rf = traj_rf_->computeNext();
            TrajectorySample sample_lh = traj_lh_->computeNext();
            TrajectorySample sample_rh = traj_rh_->computeNext();
            TrajectorySample sample_torso = traj_torso_->computeNext();

            com_task_->setReference(sample_com);
            posture_task_->setReference(sample_posture);
            floatingb_task_->setReference(sample_floatingb);
            lf_task_->setReference(sample_lf);
            rf_task_->setReference(sample_rf);
            lh_task_->setReference(sample_lh);
            rh_task_->setReference(sample_rh);
            vert_torso_task_->setReference(sample_torso);

            TaskTrajReferenceSE3 lh = {.task = lh_task_, .ref = lh_init, .traj = traj_lh_};
            TaskTrajReferenceSE3 rh = {.task = rh_task_, .ref = rh_init, .traj = traj_rh_};
            TaskTrajReferenceSE3 lf = {.task = lf_task_, .ref = lf_init, .traj = traj_lf_};
            TaskTrajReferenceSE3 rf = {.task = rf_task_, .ref = rf_init, .traj = traj_rf_};
            TaskTrajReferenceSE3 floatingb = {.task = floatingb_task_, .ref = floatingb_init, .traj = traj_floatingb_};

            se3_task_traj_map_["lh"] = lh;
            se3_task_traj_map_["rh"] = rh;
            se3_task_traj_map_["lf"] = lf;
            se3_task_traj_map_["rf"] = rf;
            se3_task_traj_map_["floatingb"] = floatingb;
        }

        bool TalosPosTracking::update(const SensorData& sensor_data)
        {
            // get the actual reference
            auto com_ref = com_task_->getReference().pos;

            // estimate the CoP / ZMP
            bool cop_ok = _cop_estimator.update(com_ref.head(2),
                model_joint_pos("leg_left_6_joint").translation(),
                model_joint_pos("leg_right_6_joint").translation(),
                sensor_data.lf_torque, sensor_data.lf_force,
                sensor_data.rf_torque, sensor_data.rf_force);

            // modify the CoM reference (stabilizer) if the CoP is valid
            if (_use_stabilizer && cop_ok && !isnan(_cop_estimator.cop_filtered()(0)) && !isnan(_cop_estimator.cop_filtered()(1))) {
                Eigen::VectorXd cor = _stabilizer_p.array() * (com_ref.head(2) - _cop_estimator.cop_filtered()).array()
                    + _stabilizer_d.array() * _cop_estimator.derror_filtered().array();
                Eigen::VectorXd ref_m = com_ref - Eigen::Vector3d(cor(0), cor(1), 0);

                set_com_ref(ref_m);
            }

            // solve everything
            bool solved = _solve();

            // set the CoM back (useful if the behavior does not the set the ref at each timestep)
            set_com_ref(com_ref);

            return solved;
        }

        const pinocchio::SE3& TalosPosTracking::se3_ref(const std::string& task_name) const
        {
            auto it = se3_task_traj_map_.find(task_name);
            assert(it != se3_task_traj_map_.end());
            return it->second.ref;
        }

        const pinocchio::SE3& TalosPosTracking::model_joint_pos(const std::string& joint_name) const
        {
            return robot_->position(tsid_->data(), robot_->model().getJointId(joint_name));
        }

        const Eigen::Vector2d& TalosPosTracking::cop() const
        {
            return _cop_estimator.cop_filtered();
        }

        void TalosPosTracking::set_se3_ref(const pinocchio::SE3& ref, const std::string& task_name)
        {
            auto it = se3_task_traj_map_.find(task_name);
            assert(it != se3_task_traj_map_.end());
            auto task_traj = it->second;
            task_traj.ref = ref;
            task_traj.traj->setReference(task_traj.ref);
            tsid::trajectories::TrajectorySample sample = task_traj.traj->computeNext();
            task_traj.task->setReference(sample);
        }

        void TalosPosTracking::set_com_ref(const Vector3& ref)
        {
            traj_com_->setReference(ref);
            TrajectorySample sample_com_ = traj_com_->computeNext();
            com_task_->setReference(sample_com_);
        }

        void TalosPosTracking::set_posture_ref(const tsid::math::Vector& ref, const std::string& task_name)
        {
            traj_posture_->setReference(ref);
            TrajectorySample sample_posture_ = traj_posture_->computeNext();
            posture_task_->setReference(sample_posture_);
        }

        void TalosPosTracking::remove_contact(const std::string& contact_name)
        {
            tsid_->removeRigidContact(contact_name);
        }

        void TalosPosTracking::add_contact(const std::string& contact_name)
        {
            const opt_params_t& p = params_.opt_params;
            const pinocchio::Data& data = tsid_->data();
            if (contact_name == "contact_rfoot") {
                contact_rf_ref_ = robot_->position(data, robot_->model().getJointId(rf_frame_name_));
                contactRF_->setReference(contact_rf_ref_);
                tsid_->addRigidContact(*contactRF_, p.at("w_forceRef_feet"));
            }
            else if (contact_name == "contact_lfoot") {
                contact_lf_ref_ = robot_->position(data, robot_->model().getJointId(lf_frame_name_));
                contactLF_->setReference(contact_lf_ref_);
                tsid_->addRigidContact(*contactLF_, p.at("w_forceRef_feet"));
            }
            else {
                std::cout << "unknown contact" << std::endl;
            }
        }
    } // namespace controllers
} // namespace inria_wbc
