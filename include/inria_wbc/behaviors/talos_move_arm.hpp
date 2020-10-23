#ifndef IWBC_MOVE_ARM_HPP
#define IWBC_MOVE_ARM_HPP
#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/factory.hpp>
#include <inria_wbc/controllers/talos_pos_tracking.hpp>
#include <inria_wbc/utils/trajectory_handler.hpp>

namespace inria_wbc {
    namespace behaviors {
        class TalosMoveArm : public Behavior {
        public:
            TalosMoveArm(const inria_wbc::controllers::TalosBaseController::Params& params);
            TalosMoveArm() = delete;
            TalosMoveArm(const TalosMoveArm&) = delete;

            bool update(const controllers::SensorData&) override;
            virtual ~TalosMoveArm() {}

        private:
            int time_ = 0;
            int traj_selector_ = 0;
            std::vector<std::vector<pinocchio::SE3>> trajectories_;
            std::vector<pinocchio::SE3> current_trajectory_;
            float trajectory_duration_ = 3; //will be changed if specified in yaml
            float motion_size_ = 0.05; //will be changed if specified in yaml
        };
    } // namespace behaviors
} // namespace inria_wbc
#endif