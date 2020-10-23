
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
            Cop(double sample_time = 0.001, size_t history_size = 5)
                : _sample_time(sample_time),
                  _history_size(history_size) {}

            // returns the filtered CoP
            bool update(
                const Eigen::Vector2d& ref,
                const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
                const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
                const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force);
            void set_history_size(size_t h) { _history_size = h; }
            size_t history_size() const { return _history_size; }
            void set_sample_time(size_t t) { _sample_time = t; }
            // estimates of cop
            const Eigen::Vector2d& cop_filtered() const { return _cop_filtered; }
            const Eigen::Vector2d& cop_raw() const { return _cop_raw; }
            // derivatives of the error (cop - ref)
            const Eigen::Vector2d& derror_filtered() const { return _derror_filtered; }
            const Eigen::Vector2d& derror_raw() const { return _derror_raw; }

        protected:
            static constexpr float FMIN = 30;
            double _sample_time;
            size_t _history_size;

            Eigen::Vector2d _cop_raw; // last computed CoP
            Eigen::Vector2d _cop_filtered; // filtered CoP
            Eigen::Vector2d _derror_raw; // last computed CoP
            Eigen::Vector2d _derror_filtered; // filtered CoP
            Eigen::Vector2d _prev_ref; // last computed CoP

            std::deque<Eigen::Vector2d> _cop_buffer; // previous values of cop
            std::deque<Eigen::Vector2d> _cop_filtered_buffer; // previous filtered values
            std::deque<Eigen::Vector2d> _derror_buffer; // previous derivatives

            void _store(const Eigen::Vector2d& v, std::deque<Eigen::Vector2d>& buffer, size_t h_size)
            {
                buffer.push_back(v);
                if (buffer.size() > h_size)
                    buffer.pop_front();
            }
            Eigen::Vector2d _compute_cop(
                const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
                const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
                const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force);
        };
    } // namespace estimators
} // namespace inria_wbc

#endif