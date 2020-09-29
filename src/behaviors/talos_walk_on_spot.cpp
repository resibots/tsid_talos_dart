#include "inria_wbc/behaviors/talos_walk_on_spot.hpp"

namespace inria_wbc
{
    namespace behaviors
    {

        static AutoRegister<WalkOnSpot> __talos_squat("walk-on-spot");;
         
        WalkOnSpot::WalkOnSpot(const inria_wbc::controllers::TalosBaseController::Params &params) :
            Behavior(std::make_shared<inria_wbc::controllers::TalosPosTracking>(params))
        {
            //////////////////// DEFINE COM TRAJECTORIES  //////////////////////////////////////
            traj_selector_ = 0;

            YAML::Node config = YAML::LoadFile(controller_->params().sot_config_path);
            inria_wbc::utils::parse(trajectory_duration_, "trajectory_duration", config, false, "BEHAVIOR");
            inria_wbc::utils::parse(motion_size_, "motion_size", config, false, "BEHAVIOR");

            state_ = States::MOVE_COM_RIGHT;
            dt_ = params.dt;
        }

        bool WalkOnSpot::update()
        {
            switch (state_)
            {
            case States::MOVE_COM_RIGHT:
                if(first_run_){
                    first_run_ = false;
                    std::cout << "Move CoM to right foot" << std::endl;
                    auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->com_init();
                    rf_init_ = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_foot_SE3("rf");
                    auto right_foot_pos = rf_init_.translation();
                    Eigen::VectorXd com_right = com_init;
                    com_right(0) = right_foot_pos(0);
                    com_right(1) = right_foot_pos(1);
                    current_trajectory_ = trajectory_handler::compute_traj(com_init, com_right, dt_, trajectory_duration_/2);
                }
                break;
            case States::LIFT_UP_LF:
                if(first_run_){
                    first_run_ = false;
                    std::cout << "Lift up left foot" << std::endl;
                    std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->remove_contact("contact_lfoot");
                }
                break;
            case States::LIFT_DOWN_LF:
                if(first_run_){
                    first_run_ = false;
                    std::cout << "Lift down left foot" << std::endl;
                    std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->add_contact("contact_lfoot");
                }
                break;
            default:
                break;
            }
            auto ref = current_trajectory_[time_];
            std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_com_ref(ref);
            if (controller_->solve())
            {
                time_++;
                if (time_ == current_trajectory_.size())
                {
                    time_ = 0;
                    first_run_ = true;
                    traj_selector_ = ++traj_selector_ % trajectories_.size();
                    state_ = cycle[traj_selector_];
                    current_trajectory_ = trajectories_[traj_selector_];
                }
                return true;
            }
            else
            {
                return false;
            }
        }

    } // namespace behaviors
} // namespace inria_wbc
