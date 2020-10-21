/* Pinocchio !!!! NEED TO BE INCLUDED BEFORE BOOST*/
#include <pinocchio/algorithm/joint-configuration.hpp> // integrate
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "Eigen/Core"
#include "map"
#include "string"
#include "vector"
#include <iomanip>
#include <memory>
#include <unordered_map>
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

#include "inria_wbc/controllers/talos_base_controller.hpp"
#include <chrono>

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
        TalosBaseController::TalosBaseController(const Params& params)
        {
            params_ = params;
            verbose_ = params_.verbose;
            pinocchio::Model robot_model;
            if (!params.floating_base_joint_name.empty()) {
                fb_joint_name_ = params.floating_base_joint_name; //floating base joint already in urdf
                pinocchio::urdf::buildModel(params.urdf_path, robot_model, verbose_);
            }
            else {
                pinocchio::urdf::buildModel(params.urdf_path, pinocchio::JointModelFreeFlyer(), robot_model, verbose_);
                fb_joint_name_ = "root_joint";
            }
            robot_ = std::make_shared<RobotWrapper>(robot_model, verbose_);
            pinocchio::srdf::loadReferenceConfigurations(robot_->model(), params.srdf_path, verbose_); //the srdf contains initial joint positions

            _reset();
        }

        // reset everything (useful when cloning)
        void TalosBaseController::_reset()
        {
            dt_ = params_.dt;
            mimic_dof_names_ = params_.mimic_dof_names;
            t_ = 0.0;

            uint nactuated = robot_->na();
            uint ndofs = robot_->nv(); // na + 6 (floating base)

            v_tsid_ = Vector::Zero(ndofs);
            a_tsid_ = Vector::Zero(ndofs);
            tau_tsid_ = Vector::Zero(nactuated);
            q_lb_ = Vector::Zero(nactuated);
            q_ub_ = Vector::Zero(nactuated);
            dq_max_ = Vector::Zero(nactuated);
            ddq_max_ = Vector::Zero(nactuated);

            q0_.resize(ndofs);
            q_.resize(ndofs);
            dq_.resize(ndofs);
            ddq_.resize(ndofs);
            tau_.resize(ndofs);

            //q0_ is set with the stack from q_tsid
            q_.setZero(q_.size());
            dq_.setZero(dq_.size());
            ddq_.setZero(ddq_.size());
            tau_.setZero(tau_.size());

            tsid_joint_names_ = all_dofs(false);
            non_mimic_indexes_ = get_non_mimics_indexes();
        }

        std::vector<int> TalosBaseController::get_non_mimics_indexes()
        {
            std::vector<int> non_mimic_indexes;
            std::vector<int> mimic_indexes;
            if (!mimic_dof_names_.empty()) {
                for (auto& m : mimic_dof_names_) {
                    auto it = std::find(tsid_joint_names_.begin(), tsid_joint_names_.end(), m);
                    assert(it != tsid_joint_names_.end());
                    mimic_indexes.push_back(std::distance(tsid_joint_names_.begin(), it));
                }
                for (int i = 0; i < q_.size(); i++) {
                    auto it = std::find(mimic_indexes.begin(), mimic_indexes.end(), i);
                    if (it == mimic_indexes.end())
                        non_mimic_indexes.push_back(i);
                }
            }
            else {
                non_mimic_indexes.resize(q_.size());
                std::generate(non_mimic_indexes.begin(), non_mimic_indexes.end(), [n = 0]() mutable { return n++; });
            }
            return non_mimic_indexes;
        }

        bool TalosBaseController::solve()
        {
            //Compute the current data from the current position and solve to find next position
            assert(tsid_);

            const HQPData& HQPData = tsid_->computeProblemData(t_, q_tsid_, v_tsid_);

            assert(solver_);
            const HQPOutput& sol = solver_->solve(HQPData);

            if (sol.status == HQP_STATUS_OPTIMAL) {
                const Vector& tau = tsid_->getActuatorForces(sol);
                const Vector& dv = tsid_->getAccelerations(sol);
                tau_tsid_ = tau;
                a_tsid_ = dv;
                v_tsid_ += dt_ * dv;

                q_tsid_ = pinocchio::integrate(robot_->model(), q_tsid_, dt_ * v_tsid_);
                t_ += dt_;

                Eigen::Quaterniond quat(q_tsid_(6), q_tsid_(3), q_tsid_(4), q_tsid_(5));
                Eigen::AngleAxisd aaxis(quat);
                q_ << q_tsid_.head(3), aaxis.angle() * aaxis.axis(), q_tsid_.tail(robot_->nq() - 7); //q_tsid_ of size 37 (pos+quat+nactuated)
                dq_ = v_tsid_; //the speed of the free flyerjoint is dim 6 even if its pos id dim 7
                tau_ << 0, 0, 0, 0, 0, 0, tau_tsid_; //the size of tau is actually 30 (nactuated)
                ddq_ = a_tsid_;
                return true;
            }
            else {
                std::cerr << "Controller failed, can't solve problem " << std::endl;
                std::cerr << "Status : " + toString(sol.status);
                switch (sol.status) {
                case -1:
                    std::cerr << " => Unknown";
                    break;
                case 1:
                    std::cerr << " => Infeasible ";
                    break;
                case 2:
                    std::cerr << " => Unbounded ";
                    break;
                case 3:
                    std::cerr << " => Max iter reached ";
                    break;
                case 4:
                    std::cerr << " => Error ";
                    break;
                default:
                    std::cerr << " => Uknown status";
                }
                std::cerr << std::endl;
                return false;
            }
        }

        // Removes the universe and root (floating base) joint names
        std::vector<std::string> TalosBaseController::controllable_dofs(bool filter_mimics)
        {
            auto na = robot_->model().names;
            auto tsid_controllables = std::vector<std::string>(robot_->model().names.begin() + 2, robot_->model().names.end());
            if (filter_mimics)
                return remove_intersection(tsid_controllables, mimic_dof_names_);
            else
                return tsid_controllables;
        }

        // Order of the floating base in q_ according to dart naming convention
        std::vector<std::string> TalosBaseController::floating_base_dofs()
        {
            std::vector<std::string> floating_base_dofs;
            if (fb_joint_name_ == "root_joint") {
                floating_base_dofs = {"rootJoint_pos_x",
                    "rootJoint_pos_y",
                    "rootJoint_pos_z",
                    "rootJoint_rot_x",
                    "rootJoint_rot_y",
                    "rootJoint_rot_z"};
            }
            else {
                floating_base_dofs = {fb_joint_name_ + "_pos_x",
                    fb_joint_name_ + "_pos_y",
                    fb_joint_name_ + "_pos_z",
                    fb_joint_name_ + "_rot_x",
                    fb_joint_name_ + "_rot_y",
                    fb_joint_name_ + "_rot_z"};
            }

            return floating_base_dofs;
        }

        std::vector<std::string> TalosBaseController::all_dofs(bool filter_mimics)
        {
            std::vector<std::string> all_dofs = floating_base_dofs();
            std::vector<std::string> control_dofs = controllable_dofs(filter_mimics);
            all_dofs.insert(all_dofs.end(), control_dofs.begin(), control_dofs.end());
            if (filter_mimics)
                return remove_intersection(all_dofs, mimic_dof_names_);
            else
                return all_dofs;
        }

        Eigen::VectorXd TalosBaseController::ddq(bool filter_mimics)
        {
            if (filter_mimics)
                return slice_vec(ddq_, non_mimic_indexes_);
            else
                return ddq_;
        }

        Eigen::VectorXd TalosBaseController::dq(bool filter_mimics)
        {
            if (filter_mimics)
                return slice_vec(dq_, non_mimic_indexes_);
            else
                return dq_;
        }

        Eigen::VectorXd TalosBaseController::q(bool filter_mimics)
        {
            if (filter_mimics)
                return slice_vec(q_, non_mimic_indexes_);
            else
                return q_;
        }

        Eigen::VectorXd TalosBaseController::q0(bool filter_mimics)
        {
            if (filter_mimics)
                return slice_vec(q0_, non_mimic_indexes_);
            else
                return q0_;
        }

        Eigen::VectorXd TalosBaseController::filter_cmd(const Eigen::VectorXd& cmd)
        {
            return slice_vec(cmd, non_mimic_indexes_);
        }

        std::vector<double> TalosBaseController::pinocchio_model_masses()
        {
            std::vector<double> masses;
            for (int i = 0; i < robot_->model().inertias.size(); i++) {
                //order corresponds to robot_->model().names
                masses.push_back(robot_->model().inertias[i].mass());
            }
            return masses;
        }

        std::vector<double> TalosBaseController::pinocchio_model_cumulated_masses()
        {
            return tsid_->data().mass;
        }

        std::vector<std::string> TalosBaseController::pinocchio_joint_names()
        {
            return robot_->model().names;
        }

        inria_wbc::controllers::TalosBaseController::Params parse_params(YAML::Node config)
        {
            std::string urdf_path = "";
            std::string srdf_path = "";
            std::string sot_config_path = "";
            std::string floating_base_joint_name = "";
            float dt = 0.001;
            bool verbose = false;
            std::vector<std::string> mimic_dof_names = {};
            parse(urdf_path, "urdf_path", config, false, "PARAMS");
            parse(srdf_path, "srdf_path", config, false, "PARAMS");
            parse(sot_config_path, "sot_config_path", config, false, "PARAMS");
            parse(floating_base_joint_name, "floating_base_joint_name", config, false, "PARAMS");
            parse(dt, "dt", config, false, "PARAMS");
            parse(verbose, "verbose", config, false, "PARAMS");
            parse(mimic_dof_names, "mimic_dof_names", config, false, "PARAMS");

            TalosBaseController::Params params = {urdf_path,
                srdf_path,
                sot_config_path,
                floating_base_joint_name,
                dt,
                verbose,
                mimic_dof_names};

            return params;
        }

        const pinocchio::SE3& TalosBaseController::right_ankle() const
        {
            return robot_->position(tsid_->data(), robot_->model().getJointId("leg_right_6_joint"));
        }

        const pinocchio::SE3& TalosBaseController::left_ankle() const
        {
            return robot_->position(tsid_->data(), robot_->model().getJointId("leg_left_6_joint"));
        }

        const Eigen::Vector3d& TalosBaseController::com() const
        {
            return robot_->com(tsid_->data());
        }

    } // namespace controllers
} // namespace inria_wbc
