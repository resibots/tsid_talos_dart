
#ifndef IWBC_ESTIMATOR_COP_HPP
#define IWBC_ESTIMATOR_COP_HPP

#include <Eigen/Core>
#include <algorithm>
#include <deque>

namespace inria_wbc {
    namespace estimators {
        // computes the CoP with the FT sensors + a filter
        class Cop {
        public:
            Cop(size_t history_size = 50) : _history_size(history_size) {}

            // returns the filtered CoP
            const Eigen::Vector2d& update(
                const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
                const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
                const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force);
            const Eigen::Vector2d& cop_filtered() const { return _cop_filtered; }
            const Eigen::Vector2d& cop_raw() const { return _cop_raw; }

        protected:
            Eigen::Vector2d _cop_raw; // last computed CoP
            Eigen::Vector2d _cop_filtered; // filtered CoP
            size_t _history_size;
            std::deque<Eigen::Vector2d> _history;
            Eigen::Vector2d _compute_cop(
                const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
                const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
                const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force);
        };
    } // namespace estimators
} // namespace inria_wbc

#endif