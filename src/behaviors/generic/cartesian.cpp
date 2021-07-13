#include "inria_wbc/behaviors/generic/cartesian.hpp"

namespace inria_wbc {
    namespace behaviors {
        namespace generic {
            static Register<Cartesian> __talos_move_arm("generic::cartesian");

            Cartesian::Cartesian(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller, config),
                                                                                                 traj_selector_(0)
            {
                auto c = IWBC_CHECK(config["BEHAVIOR"]);
                trajectory_duration_ = IWBC_CHECK(c["trajectory_duration"].as<float>());
                behavior_type_ = this->behavior_type();
                controller_->set_behavior_type(behavior_type_);

                loop_ = IWBC_CHECK(c["loop"].as<bool>());
                task_names_ = IWBC_CHECK(c["task_names"].as<std::vector<std::string>>());
                auto ts = IWBC_CHECK(c["relative_targets"].as<std::vector<std::vector<double>>>());

                if (task_names_.size() != ts.size())
                    IWBC_ERROR("cartesian behavior needs the same number of tasks and targets");

                for (uint i = 0; i < task_names_.size(); i++) {
                    auto task_init = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref(task_names_[i]);
                    auto task_final = task_init;

                    relative_targets_.push_back(Eigen::Vector3d::Map(ts[i].data()));
                    task_final.translation() = relative_targets_[i] + task_init.translation();

                    std::vector<std::vector<pinocchio::SE3>> trajectory;
                    trajectory.push_back(trajectory_handler::compute_traj(task_init, task_final, controller_->dt(), trajectory_duration_));
                    if (loop_)
                        trajectory.push_back(trajectory_handler::compute_traj(task_final, task_init, controller_->dt(), trajectory_duration_));
                    trajectories_.push_back(trajectory);
                }
            }

            void Cartesian::update(const controllers::SensorData& sensor_data)
            {

                for (uint i = 0; i < task_names_.size(); i++) {
                    if (traj_selector_ < trajectories_[i].size()) {
                        auto ref = trajectories_[i][traj_selector_][time_];
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_se3_ref(ref, task_names_[i]);
                    }
                }

                controller_->update(sensor_data);
                time_++;
                if (trajectories_.size() > 0) {
                    if (time_ == trajectories_[0][traj_selector_].size()) {
                        time_ = 0;
                        traj_selector_ = ++traj_selector_;
                        if (loop_)
                            traj_selector_ = traj_selector_ % trajectories_[0].size();
                    }
                }
            }
        } // namespace generic
    } // namespace behaviors
} // namespace inria_wbc