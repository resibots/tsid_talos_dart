#include <numeric>

#include <inria_wbc/estimators/cop.hpp>

namespace inria_wbc {
    namespace estimators {

        const Eigen::Vector2d& Cop::update(const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
            const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
            const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force)
        {
            _cop_raw = _compute_cop(lf_pos, rf_pos, lf_torque, lf_force, rf_torque, rf_force);
            // store history
            _history.push_back(_cop_raw);
            if (_history.size() > _history_size)
                _history.pop_front();
            // moving average filtering : average the values in history
            _cop_filtered
                = std::accumulate(_history.begin(), _history.end(), (Eigen::Vector2d)Eigen::Vector2d::Zero()) / _history.size();
            return _cop_filtered;
        }

        // Intro to humanoid robotics (Kajita et al.), p80, 3.26
        Eigen::Vector2d Cop::_compute_cop(const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
            const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
            const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force)
        {
            // CoP left foot
            Eigen::Vector2d lcop_raw = lf_pos.head(2);
            lcop_raw(0) += (-lf_torque(1) - lf_pos(2) * lf_force(0)) / lf_force(2);
            lcop_raw(1) += (lf_torque(0) - lf_pos(2) * lf_force(1)) / lf_force(2);

            // CoP right foot
            Eigen::Vector2d rcop_raw = rf_pos.head(2);
            rcop_raw(0) += (-rf_torque(1) - rf_pos(2) * rf_force(0)) / rf_force(2);
            rcop_raw(1) += (rf_torque(0) - rf_pos(2) * rf_force(1)) / rf_force(2);

            double Fz_ratio_l = lf_force(2) / (lf_force(2) + rf_force(2));
            double Fz_ratio_r = rf_force(2) / (lf_force(2) + rf_force(2));

            Eigen::Vector2d cop_in_lft_raw = Fz_ratio_l * lcop_raw + Fz_ratio_r * (rcop_raw + rf_pos.head(2) - lf_pos.head(2));
            Eigen::Vector2d cop_in_rft_raw = Fz_ratio_l * (lcop_raw + lf_pos.head(2) - rf_pos.head(2)) + Fz_ratio_r * rcop_raw;
            Eigen::Vector2d cop = cop_in_lft_raw * Fz_ratio_l + cop_in_rft_raw * Fz_ratio_r;
            return cop;
        }
    } // namespace estimators
} // namespace inria_wbc