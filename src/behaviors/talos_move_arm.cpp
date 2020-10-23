#include "inria_wbc/behaviors/talos_move_arm.hpp"

namespace inria_wbc {
    namespace behaviors {
        static AutoRegister<TalosMoveArm> __talos_move_arm("talos-move-arm");

        TalosMoveArm::TalosMoveArm(const inria_wbc::controllers::TalosBaseController::Params& params) : Behavior(std::make_shared<inria_wbc::controllers::TalosPosTracking>(params))
        {

            //////////////////// DEFINE COM TRAJECTORIES  //////////////////////////////////////
            traj_selector_ = 0;
            auto lh_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_se3_ref("lh");

            YAML::Node config = YAML::LoadFile(controller_->params().sot_config_path);
            inria_wbc::utils::parse(trajectory_duration_, "trajectory_duration", config, false, "BEHAVIOR");
            inria_wbc::utils::parse(motion_size_, "motion_size", config, false, "BEHAVIOR");

            auto lh_final = lh_init;
            lh_final.translation()(2) += motion_size_;

            trajectories_.push_back(trajectory_handler::compute_traj(lh_init, lh_final, params.dt, trajectory_duration_));
            trajectories_.push_back(trajectory_handler::compute_traj(lh_final, lh_init, params.dt, trajectory_duration_));
            current_trajectory_ = trajectories_[traj_selector_];
        }

        bool TalosMoveArm::update(const controllers::SensorData& sensor_data)
        {
            auto ref = current_trajectory_[time_];
            std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(ref, "lh");
            if (controller_->update(sensor_data)) {
                time_++;
                if (time_ == current_trajectory_.size()) {
                    time_ = 0;
                    traj_selector_ = ++traj_selector_ % trajectories_.size();
                    current_trajectory_ = trajectories_[traj_selector_];
                }
                return true;
            }
            else {
                return false;
            }
        }

    } // namespace behaviors
} // namespace inria_wbc
