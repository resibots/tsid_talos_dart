
#ifndef IWBC_ESTIMATOR_COP_HPP
#define IWBC_ESTIMATOR_COP_HPP

#include <Eigen/Core>

namespace inria_wbc {
    namespace estimators {
        // computes the CoP with the FT sensors + a filter
        class Cop {
        public:
            const Eigen::Vector2d& update(
                const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
                const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
                const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force);
            const Eigen::Vector2d& cop() const { return _cop; }

        protected:
            Eigen::Vector2d _cop;
            Eigen::Vector2d _compute_cop(
                const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
                const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
                const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force);
        };
    } // namespace estimators
} // namespace inria_wbc

#endif