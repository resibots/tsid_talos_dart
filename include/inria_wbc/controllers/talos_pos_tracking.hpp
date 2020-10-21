#ifndef IWBC_POS_TRACKING_HPP
#define IWBC_POS_TRACKING_HPP

#include <inria_wbc/controllers/talos_base_controller.hpp>

namespace inria_wbc {
    namespace controllers {
        class TalosPosTracking : public TalosBaseController {
        public:
            TalosPosTracking(const Params& params);
            TalosPosTracking(const TalosPosTracking& other) = delete;
            TalosPosTracking& operator=(const TalosPosTracking& o) const = delete;
            virtual ~TalosPosTracking(){};

            std::shared_ptr<tsid::tasks::TaskComEquality> com_task() { return com_task_; }
            tsid::math::Vector3 com_init() { return com_init_; }

            pinocchio::SE3 get_se3_ref(const std::string& task_name);
            tsid::math::Vector3 get_pinocchio_com();

            void set_com_ref(const tsid::math::Vector3& ref);
            void set_se3_ref(const pinocchio::SE3& ref, const std::string& task_name);
            void set_posture_ref(const tsid::math::Vector& ref, const std::string& task_name);

            void remove_contact(const std::string& contact_name);
            void add_contact(const std::string& contact_name);
            pinocchio::SE3 get_RF_SE3();
            pinocchio::SE3 get_LF_SE3();

            virtual const opt_params_t& opt_params() const override { return params_.opt_params; }

        private:
            void parse_configuration_yaml(const std::string& sot_config_path) override;
            void set_stack_configuration() override;
            void init_references() override;
            void set_task_traj_map() override;
            void set_default_opt_params(std::map<std::string, double>& p);

            // TALOS CONFIG
            double lxp_ = 0.1; // foot length in positive x direction
            double lxn_ = 0.11; // foot length in negative x direction
            double lyp_ = 0.069; // foot length in positive y direction
            double lyn_ = 0.069; // foot length in negative y direction
            double lz_ = 0.107; // foot sole height with respect to ankle joint
            double mu_ = 0.3; // friction coefficient
            double fMin_ = 5.0; // minimum normal force
            double fMax_ = 1500.0; // maximum normal force
            std::string rf_frame_name_ = "leg_right_6_joint"; // right foot joint name
            std::string lf_frame_name_ = "leg_left_6_joint"; // left foot joint name
            std::string torso_frame_name_ = "torso_2_link"; // left foot joint name

            tsid::math::Vector3 contactNormal_ = tsid::math::Vector3::UnitZ(); // direction of the normal to the contact surface
            std::map<std::string, double> opt_params_; // the parameters that we can tune with an optimizer (e.g., task weights)

            // contacts
            tsid::math::Matrix3x contact_points_;
            pinocchio::SE3 contact_rf_ref_, contact_lf_ref_;
            std::shared_ptr<tsid::contacts::Contact6d> contactRF_;
            std::shared_ptr<tsid::contacts::Contact6d> contactLF_;

            // tasks
            std::shared_ptr<tsid::tasks::TaskComEquality> com_task_;
            std::shared_ptr<tsid::tasks::TaskJointPosture> posture_task_;
            std::shared_ptr<tsid::tasks::TaskSE3Equality> floatingb_task_;
            std::shared_ptr<tsid::tasks::TaskSE3Equality> lf_task_;
            std::shared_ptr<tsid::tasks::TaskSE3Equality> rf_task_;
            std::shared_ptr<tsid::tasks::TaskSE3Equality> lh_task_;
            std::shared_ptr<tsid::tasks::TaskSE3Equality> rh_task_;
            std::shared_ptr<tsid::tasks::TaskSE3Equality> vert_torso_task_;

            std::shared_ptr<tsid::tasks::TaskJointPosVelAccBounds> bounds_task_;

            // limits (position, velocity, acceleration)
            tsid::math::Vector q_lb_; // lower position bound
            tsid::math::Vector q_ub_; // upper position bound
            tsid::math::Vector dq_max_; // max velocity bound
            tsid::math::Vector ddq_max_; // max acceleration bound

            // com ref
            tsid::math::Vector3 com_init_, com_ref_;
            std::shared_ptr<tsid::trajectories::TrajectoryEuclidianConstant> traj_com_;

            // posture ref
            tsid::math::Vector posture_init_, posture_ref_;
            std::shared_ptr<tsid::trajectories::TrajectoryEuclidianConstant> traj_posture_;

            // floatingb ref
            pinocchio::SE3 floatingb_init_, floatingb_ref_;
            std::shared_ptr<tsid::trajectories::TrajectorySE3Constant> traj_floatingb_;

            // Left Foot ref
            pinocchio::SE3 lf_init_, lf_ref_;
            std::shared_ptr<tsid::trajectories::TrajectorySE3Constant> traj_lf_;

            // Right Foot ref
            pinocchio::SE3 rf_init_, rf_ref_;
            std::shared_ptr<tsid::trajectories::TrajectorySE3Constant> traj_rf_;

            // Left Hand ref
            pinocchio::SE3 lh_init_, lh_ref_;
            std::shared_ptr<tsid::trajectories::TrajectorySE3Constant> traj_lh_;

            // Right Hand ref
            pinocchio::SE3 rh_init_, rh_ref_;
            std::shared_ptr<tsid::trajectories::TrajectorySE3Constant> traj_rh_;

            std::shared_ptr<tsid::trajectories::TrajectorySE3Constant> traj_torso_;

            std::unordered_map<std::string, TaskTrajReferenceSE3> se3_task_traj_map_;
        };
    } // namespace controllers
} // namespace inria_wbc
#endif
