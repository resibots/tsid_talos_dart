#ifndef TORQUE_SAFETY_HPP
#define TORQUE_SAFETY_HPP

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "inria_wbc/exceptions.hpp"
#include "inria_wbc/estimators/filtering.hpp"


namespace inria_wbc {
namespace safety {


class TorqueCollisionDetection
{
public:

    typedef Eigen::Array<bool,-1,-1> ArrayXb;

    TorqueCollisionDetection() = default;
    TorqueCollisionDetection(int nvar, double threshold=1.0, int buffer_len=1);
    TorqueCollisionDetection(Eigen::VectorXd threshold, int buffer_len=1);
    ~TorqueCollisionDetection() = default;

    bool check(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors);

    void set_max_consecutive_invalid(unsigned int counter);

    void set_offset(const Eigen::VectorXd& offset);
    Eigen::VectorXd get_offset() const;
    void reset_offset();

    void set_filter(estimators::Filter::Ptr filter_ptr);
    estimators::Filter::Ptr get_filter();
    void remove_filter();

    void set_threshold(double threshold);
    void set_threshold(const Eigen::VectorXd& threshold);
    Eigen::VectorXd get_threshold() const;

    Eigen::VectorXd get_discrepancy() const;

    Eigen::VectorXd get_filtered_sensors() const;

    std::vector<int> get_invalid_ids() const;


protected:

    void _compute_validity(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors);
    void _compute_validity_over_steps();


private:

    int _nvar;
    int _step_count;

    Eigen::VectorXd _offset;
    Eigen::VectorXd _threshold;
    Eigen::VectorXd _discrepancy;
    Eigen::VectorXd _filtered_sensors;
    ArrayXb _validity;

    bool _add_offset = false;

    u_int32_t _invalid_threshold;
    Eigen::MatrixXi _previous_signs;
    Eigen::VectorXi _invalid_steps_threshold;

    estimators::Filter::Ptr _filter_ptr;

};


} // namespace safety
} // namespace inria_wbc

#endif
