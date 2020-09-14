/* Pinocchio !!!! NEED TO BE INCLUDED BEFORE BOOST*/
#include <pinocchio/algorithm/joint-configuration.hpp> // integrate
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "Eigen/Core"
#include <iomanip>
#include <memory>
#include <utility>
#include "map"
#include "vector"

#include <tsid/contacts/contact-6d.hpp>
#include <tsid/contacts/contact-point.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include <tsid/trajectories/trajectory-base.hpp>
#include <tsid/trajectories/trajectory-se3.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/solvers/utils.hpp>
#include <tsid/utils/stop-watch.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/math/utils.hpp>
#include <tsid/robots/fwd.hpp>
#include <tsid/robots/robot-wrapper.hpp>

#include "inria_wbc/controllers/talos_pos_tracking.hpp"

using namespace tsid;
using namespace tsid::trajectories;
using namespace tsid::math;
using namespace tsid::contacts;
using namespace tsid::tasks;
using namespace tsid::solvers;
using namespace tsid::robots;
using namespace std;
using namespace inria_wbc::utils;
namespace inria_wbc
{
  namespace controllers
  {
    
    TalosPosTracking::TalosPosTracking(const Params &params) : TalosBaseController(params)
    {
      set_default_opt_params(params_.opt_params);

      if (!params.sot_config_path.empty())
        parse_configuration_yaml(params.sot_config_path);

      set_stack_configuration();
      init_references();
    }

    TalosPosTracking::TalosPosTracking(const TalosPosTracking &other) : TalosBaseController(other)
    {
      set_default_opt_params(params_.opt_params); // prefill to get the size
      assert(other.params_.opt_params.size() == params_.opt_params.size());

      params_.opt_params = other.params_.opt_params;
      set_stack_configuration();
      init_references();
    }
    TalosPosTracking::TalosPosTracking(const TalosPosTracking &other, const Params& p) : TalosBaseController(other, p)
    {
      // params will be set in the base class constructor
      set_default_opt_params(params_.opt_params); // prefill to get the size
      assert(other.params_.opt_params.size() == params_.opt_params.size());

      params_.opt_params = other.params_.opt_params;
      set_stack_configuration();
      init_references();
    }

   void TalosPosTracking::set_default_opt_params(std::map<std::string, double>& p) {      
      p["w_com"] = 10.0;           //# weight of center of mass task
      p["w_posture"] = 0.75;        //# weight of joint posture task
      p["w_forceRef_feet"] = 1e-3;  //# weight of force regularization task
      p["w_forceRef_hands"] = 1e-3; //# weight of force regularization task
      p["w_floatingb"] = 20.0;      //# weight of floatingb task
      p["w_velocity"] = 1.0;        //# weight of velocity bounds
      p["w_rh"] = 10.0;             //# weight of right hand  task
      p["w_lh"] = 10.0;             //# weight of left hand  task
      p["w_rf"] = 1.0;              //# weight of right foot  task
      p["w_lf"] = 1.0;              //# weight of left foot  task
      p["kp_contact"] = 30.0;       //# proportional gain of contact constraint
      p["kp_com"] = 3000.0;         //# proportional gain of center of mass task
      p["kp_posture"] = 30.0;       //# proportional gain of joint posture task
      p["kp_floatingb"] = 3000.0;   //# proportional gain of floatingb task
      p["kp_rh"] = 300.0;           //# proportional gain of right hand task
      p["kp_lh"] = 300.0;           //# proportional gain of left hand task
      p["kp_rf"] = 30.0;            //# proportional gain of right foot task
      p["kp_lf"] = 30.0;            //# proportional gain of left foot task
    }

 void TalosPosTracking::parse_configuration_yaml(const std::string &sot_config_path)
    {
      std::ifstream yamlConfigFile(sot_config_path);
      if (!yamlConfigFile.good())
      {
        if (verbose_)
        {
          std::cout << sot_config_path << " not found" << std::endl;
          std::cout << "Taking default stack of tasks weights" << std::endl;
        }
      }
      else
      {
        if (verbose_)
          std::cout << "Taking the stack of tasks parameters in " << sot_config_path << std::endl;

        YAML::Node config = YAML::LoadFile(sot_config_path);
        for (auto &x : params_.opt_params)
            parse(x.second, x.first, config, verbose_);
      }
    }

    void TalosPosTracking::set_stack_configuration()
    {
      const opt_params_t& p = params_.opt_params;
      ////////////////////Gather Initial Pose //////////////////////////////////////
      q_tsid_ = robot_->model().referenceConfigurations["pal_start"];
      Eigen::Quaterniond quat(q_tsid_(6), q_tsid_(3), q_tsid_(4), q_tsid_(5));
      Eigen::AngleAxisd aaxis(quat);
      q0_ << q_tsid_.head(3), aaxis.angle() * aaxis.axis(), q_tsid_.tail(robot_->na()); //q_tsid_ is of size 37 (pos+quat+nactuated)

      ////////////////////Create the inverse-dynamics formulation///////////////////
      tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);

      ////////////////////Create an HQP solver /////////////////////////////////////
      using solver_t = std::shared_ptr<tsid::solvers::SolverHQPBase>;
      solver_ = solver_t(SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST, "solver-eiquadprog"));
      solver_->resize(tsid_->nVar(), tsid_->nEq(), tsid_->nIn());

      ////////////////////Compute Problem Data at init /////////////////////////////
      const uint nv = robot_->nv();
      tsid_->computeProblemData(dt_, q_tsid_, Vector::Zero(nv));
      const pinocchio::Data &data = tsid_->data();

      ////////////////////Compute Tasks, Bounds and Contacts ///////////////////////
      ////////// Add the contacts
      Matrix3x contact_points_(3, 4);
      contact_points_ << -lxn_, -lxn_, +lxp_, +lxp_,
          -lyn_, +lyp_, -lyn_, +lyp_,
          lz_, lz_, lz_, lz_;

      contactRF_ = std::make_shared<Contact6d>("contact_rfoot", *robot_, rf_frame_name_,
                                               contact_points_, contactNormal_,
                                               mu_, fMin_, fMax_);
      contactRF_->Kp(p.at("kp_contact") * Vector::Ones(6));
      contactRF_->Kd(2.0 * contactRF_->Kp().cwiseSqrt());
      contact_rf_ref_ = robot_->position(data, robot_->model().getJointId(rf_frame_name_));
      contactRF_->setReference(contact_rf_ref_);
      tsid_->addRigidContact(*contactRF_, p.at("w_forceRef_feet"));

      contactLF_ = std::make_shared<Contact6d>("contact_lfoot", *robot_, lf_frame_name_,
                                               contact_points_, contactNormal_,
                                               mu_, fMin_, fMax_);
      contactLF_->Kp(p.at("kp_contact") * Vector::Ones(6));
      contactLF_->Kd(2.0 * contactLF_->Kp().cwiseSqrt());
      contact_lf_ref_ = robot_->position(data, robot_->model().getJointId(lf_frame_name_));
      contactLF_->setReference(contact_lf_ref_);
      tsid_->addRigidContact(*contactLF_, p.at("w_forceRef_feet"));

      ////////// Add the com task
      com_task_ = std::make_shared<TaskComEquality>("task-com", *robot_);
      com_task_->Kp(p.at("kp_com") * Vector::Ones(3));
      com_task_->Kd(2.0 * com_task_->Kp().cwiseSqrt());
      tsid_->addMotionTask(*com_task_, p.at("w_com"), 1);

      ////////// Add the posture task
      posture_task_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);
      posture_task_->Kp(p.at("kp_posture") * Vector::Ones(nv - 6));
      posture_task_->Kd(2.0 * posture_task_->Kp().cwiseSqrt());
      Vector mask_post = Vector::Ones(nv - 6);
      // for(int i =0; i < 11; i++){
      //   mask_post[i] = 0;
      // }
      posture_task_->setMask(mask_post);
      tsid_->addMotionTask(*posture_task_, p.at("w_posture"), 1);

      ////////// Add the floatingb task
      floatingb_task_ = std::make_shared<TaskSE3Equality>("task-floatingb", *robot_, fb_joint_name_);
      floatingb_task_->Kp(p.at("kp_floatingb") * Vector::Ones(6));
      floatingb_task_->Kd(2.0 * floatingb_task_->Kp().cwiseSqrt());
      Vector mask_floatingb = Vector::Ones(6);
      for (int i = 0; i < 3; i++)
      {
        mask_floatingb[i] = 0; // DO NOT CONSTRAIN floatingb POSITION IN SOT
      }
      floatingb_task_->setMask(mask_floatingb);
      tsid_->addMotionTask(*floatingb_task_, p.at("w_floatingb"), 1);

      ////////// Add the left hand  task
      lh_task_ = std::make_shared<TaskSE3Equality>("task-lh", *robot_, "gripper_left_joint");
      lh_task_->Kp(p.at("kp_lh") * Vector::Ones(6));
      lh_task_->Kd(2.0 * lh_task_->Kp().cwiseSqrt());
      Vector mask_lh = Vector::Ones(6);
      for (int i = 3; i < 6; i++)
      {
        mask_lh[i] = 0; // DO NOT CONSTRAIN HAND ORIENTATION IN SOT
      }
      lh_task_->setMask(mask_lh);
      tsid_->addMotionTask(*lh_task_, p.at("w_lh"), 1);

      ////////// Add the right hand task
      rh_task_ = std::make_shared<TaskSE3Equality>("task-rh", *robot_, "gripper_right_joint");
      rh_task_->Kp(p.at("kp_rh") * Vector::Ones(6));
      rh_task_->Kd(2.0 * rh_task_->Kp().cwiseSqrt());
      Vector mask_rh = Vector::Ones(6);
      for (int i = 3; i < 6; i++)
      {
        mask_rh[i] = 0; // DO NOT CONSTRAIN HAND ORIENTATION IN SOT
      }
      rh_task_->setMask(mask_rh);
      tsid_->addMotionTask(*rh_task_, p.at("w_rh"), 1);

      ////////// Add the left foot  task
      lf_task_ = std::make_shared<TaskSE3Equality>("task-lf", *robot_, "leg_left_6_joint");
      lf_task_->Kp(p.at("kp_lf") * Vector::Ones(6));
      lf_task_->Kd(2.0 * lf_task_->Kp().cwiseSqrt());
      Vector maskLf = Vector::Ones(6);
      lf_task_->setMask(maskLf);
      tsid_->addMotionTask(*lf_task_, p.at("w_lf"), 1);

      ////////// Add the right foot  task
      rf_task_ = std::make_shared<TaskSE3Equality>("task-rf", *robot_, "leg_right_6_joint");
      rf_task_->Kp(p.at("kp_rf") * Vector::Ones(6));
      rf_task_->Kd(2.0 * rf_task_->Kp().cwiseSqrt());
      Vector maskRf = Vector::Ones(6);
      rf_task_->setMask(maskRf);
      tsid_->addMotionTask(*rf_task_, p.at("w_rf"), 1);

      ////////// Add the position, velocity and acceleration limits
      bounds_task_ = std::make_shared<TaskJointPosVelAccBounds>("task-posVelAcc-bounds", *robot_, dt_, verbose_);
      dq_max_ = robot_->model().velocityLimit.tail(robot_->na());
      ddq_max_ = dq_max_ / dt_;
      bounds_task_->setVelocityBounds(dq_max_);
      bounds_task_->setAccelerationBounds(ddq_max_);
      q_lb_ = robot_->model().lowerPositionLimit.tail(robot_->na());
      q_ub_ = robot_->model().upperPositionLimit.tail(robot_->na());
      bounds_task_->setPositionBounds(q_lb_, q_ub_);
      tsid_->addMotionTask(*bounds_task_, p.at("w_velocity"), 0); //add pos vel acc bounds
    }

    void TalosPosTracking::init_references()
    {
      com_init_ = robot_->com(tsid_->data());
      posture_init_ = q_tsid_.tail(robot_->na());
      floatingb_init_ = robot_->position(tsid_->data(), robot_->model().getJointId(fb_joint_name_));
      lf_init_ = robot_->position(tsid_->data(), robot_->model().getJointId("leg_left_6_joint"));
      rf_init_ = robot_->position(tsid_->data(), robot_->model().getJointId("leg_right_6_joint"));
      lh_init_ = robot_->position(tsid_->data(), robot_->model().getJointId("gripper_left_joint"));
      rh_init_ = robot_->position(tsid_->data(), robot_->model().getJointId("gripper_right_joint"));

      com_ref_ = com_init_;
      posture_ref_ = posture_init_;
      floatingb_ref_ = floatingb_init_;
      lf_ref_ = lf_init_;
      rf_ref_ = rf_init_;
      lh_ref_ = lh_init_;
      rh_ref_ = rh_init_;

      traj_com_ = std::make_shared<TrajectoryEuclidianConstant>("traj_com", com_ref_);
      traj_posture_ = std::make_shared<TrajectoryEuclidianConstant>("traj_posture", posture_ref_);
      traj_floatingb_ = std::make_shared<TrajectorySE3Constant>("traj_floatingb", floatingb_ref_);
      traj_lf_ = std::make_shared<TrajectorySE3Constant>("traj_lf", lf_ref_);
      traj_rf_ = std::make_shared<TrajectorySE3Constant>("traj_rf", rf_ref_);
      traj_lh_ = std::make_shared<TrajectorySE3Constant>("traj_lh", lh_ref_);
      traj_rh_ = std::make_shared<TrajectorySE3Constant>("traj_rh", rh_ref_);

      TrajectorySample sample_com_ = traj_com_->computeNext();
      TrajectorySample sample_posture_ = traj_posture_->computeNext();
      TrajectorySample sample_floatingb_ = traj_floatingb_->computeNext();
      TrajectorySample sample_lf_ = traj_lf_->computeNext();
      TrajectorySample sample_rf_ = traj_rf_->computeNext();
      TrajectorySample sample_lh_ = traj_lh_->computeNext();
      TrajectorySample sample_rh_ = traj_rh_->computeNext();

      com_task_->setReference(sample_com_);
      posture_task_->setReference(sample_posture_);
      floatingb_task_->setReference(sample_floatingb_);
      lf_task_->setReference(sample_lf_);
      rf_task_->setReference(sample_rf_);
      lh_task_->setReference(sample_lh_);
      rh_task_->setReference(sample_rh_);

      set_task_traj_map();
    }

    void TalosPosTracking::set_task_traj_map()
    {
      TaskTrajReferenceSE3 lh = {.task = lh_task_, .ref = lh_init_, .traj = traj_lh_};
      TaskTrajReferenceSE3 rh = {.task = rh_task_, .ref = rh_init_, .traj = traj_rh_};
      TaskTrajReferenceSE3 lf = {.task = lf_task_, .ref = lf_init_, .traj = traj_lf_};
      TaskTrajReferenceSE3 rf = {.task = rf_task_, .ref = rf_init_, .traj = traj_rf_};
      TaskTrajReferenceSE3 floatingb = {.task = floatingb_task_, .ref = floatingb_init_, .traj = traj_floatingb_};

      se3_task_traj_map_["lh"] = lh;
      se3_task_traj_map_["rh"] = rh;
      se3_task_traj_map_["lf"] = lf;
      se3_task_traj_map_["rf"] = rf;
      se3_task_traj_map_["floatingb"] = floatingb;
    }

    pinocchio::SE3 TalosPosTracking::get_se3_ref(const std::string &task_name)
    {
      auto it = se3_task_traj_map_.find(task_name);
      assert(it != se3_task_traj_map_.end());
      auto task_traj = it->second;
      return task_traj.ref;
    }

    tsid::math::Vector3 TalosPosTracking::get_pinocchio_com()
    {
      return robot_->com(tsid_->data());
    }

    void TalosPosTracking::set_se3_ref(const pinocchio::SE3 &ref, const std::string &task_name)
    {
      auto it = se3_task_traj_map_.find(task_name);
      assert(it != se3_task_traj_map_.end());
      auto task_traj = it->second;
      task_traj.ref = ref;
      task_traj.traj->setReference(task_traj.ref);
      tsid::trajectories::TrajectorySample sample = task_traj.traj->computeNext();
      task_traj.task->setReference(sample);
    }

    void TalosPosTracking::set_com_ref(const Vector3 &ref)
    {
      traj_com_->setReference(ref);
      TrajectorySample sample_com_ = traj_com_->computeNext();
      com_task_->setReference(sample_com_);
    }

    void TalosPosTracking::set_posture_ref(const tsid::math::Vector &ref, const std::string &task_name)
    {
      traj_posture_->setReference(ref);
      TrajectorySample sample_posture_ = traj_posture_->computeNext();
      posture_task_->setReference(sample_posture_);
    }

  } // namespace controllers
} // namespace inria_wbc
