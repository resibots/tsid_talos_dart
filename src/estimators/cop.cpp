#include <cmath>
#include <inria_wbc/estimators/cop.hpp>
#include <iostream>
#include <numeric>

namespace inria_wbc {
    namespace estimators {

        bool Cop::update(
            const Eigen::Vector2d& ref,
            const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
            const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
            const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force)
        {
            // if we are in the air, returns false
            if (lf_force.norm() < FMIN && rf_force.norm() < FMIN)
                return false;
            _cop_raw = _compute_cop(lf_pos, rf_pos, lf_torque, lf_force, rf_torque, rf_force);

            // store in _cop_buffer for filtering
            _store(_cop_raw, _cop_buffer, _history_size);

            // moving average filtering : average the values in history
            _cop_filtered
                = std::accumulate(_cop_buffer.begin(), _cop_buffer.end(), (Eigen::Vector2d)Eigen::Vector2d::Zero()) / _cop_buffer.size();

            // store the last filtered value to compute the derivative
            Eigen::Vector2d prev_cop_filtered = _cop_filtered_buffer.empty() ? Eigen::Vector2d::Zero() : _cop_filtered_buffer.back();
            _store(_cop_filtered, _cop_filtered_buffer, _history_size);

            // derivative of the error using the filtered values
            Eigen::Vector2d prev_error = (prev_cop_filtered - _prev_ref);
            Eigen::Vector2d error = (_cop_filtered - ref);
            _derror_raw = (prev_cop_filtered - _cop_filtered) / _sample_time; //(error - prev_error) / _sample_time;
            if (_cop_buffer.size() > 1) // first value does not make sense
                _store(_derror_raw, _derror_buffer, _history_size);
            _derror_filtered
                = std::accumulate(_derror_buffer.begin(), _derror_buffer.end(), (Eigen::Vector2d)Eigen::Vector2d::Zero()) / _derror_buffer.size();
            _prev_ref = ref;
            // note: IIT uses weighted averages... here we use standard moving average.
            // we are not ready until we have filled the buffer
            if (_derror_buffer.size() < _history_size)
                return false;
            return true;
        }

        // Intro to humanoid robotics (Kajita et al.), p80, 3.26
        // see also: https://github.com/stack-of-tasks/sot-dynamic-pinocchio/blob/master/src/zmp-from-forces.cpp

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