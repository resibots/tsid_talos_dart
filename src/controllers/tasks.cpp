#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-se3-equality.hpp>

#include "inria_wbc/controllers/tasks.hpp"
#include "tsid/tasks/task-self-collision.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace tasks {
        // note on levels: everything is level 1 except the constraints:
        // - the bounds (level 0)
        // - the contacts (level 0)
        // future versions of TSID might have more levels (but not for now, 2020)

        template <int S>
        Eigen::Array<double, S, 1> convert_mask(const std::string& mask_str)
        {
            assert(S == Eigen::Dynamic || mask_str.size() == S);
            Eigen::Array<double, S, 1> mask;
            mask.resize(S); // useful for dynamic
            for (int i = 0; i < mask_str.size(); ++i)
                mask[i] = mask_str[i] == '0' ? 0 : 1;
            return mask;
        }

        ////// SE3 (warning: add the task to TSID!) //////
        std::shared_ptr<tsid::tasks::TaskBase> make_se3(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node)
        {

            // retrieve parameters from YAML
            double kp = node["kp"].as<double>();
            auto tracked = node["tracked"].as<std::string>();
            auto mask_str = node["mask"].as<std::string>();
            auto weight = node["weight"].as<double>();

            // convert the mask
            IWBC_ASSERT(mask_str.size() == 6, "SE3 masks needs to be 6D (x y z r p y), here:", mask_str);
            auto mask = convert_mask<6>(mask_str);

            // create the task
            assert(tsid);
            assert(robot);
            auto task = std::make_shared<tsid::tasks::TaskSE3Equality>(task_name, *robot, tracked);
            task->Kp(kp * Vector::Ones(6));
            task->Kd(2.0 * task->Kp().cwiseSqrt());
            task->setMask(mask.matrix());

            // set the reference
            // we need to check if this is a joint or a frame
            bool joint = robot->model().existJointName(tracked);
            bool body = robot->model().existBodyName(tracked);
            if (joint && body)
                throw IWBC_EXCEPTION("Ambiguous name to track for task ", task_name, ": this is both a joint and a frame [", tracked, "]");
            if (!joint && !body)
                throw IWBC_EXCEPTION("Unknown frame or joint [", tracked, "]");
            pinocchio::SE3 ref;

            if (joint)
                ref = robot->position(tsid->data(), robot->model().getJointId(tracked));
            else
                ref = robot->framePosition(tsid->data(), robot->model().getFrameId(tracked));
            auto sample = to_sample(ref);
            task->setReference(sample);

            // add the task to TSID (side effect, be careful)
            tsid->addMotionTask(*task, weight, 1);

            return task;
        }

        RegisterYAML<tsid::tasks::TaskSE3Equality> __register_se3_equality("se3", make_se3);

        ////// CoM (center of mass) //////
        std::shared_ptr<tsid::tasks::TaskBase> make_com(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node)
        {
            assert(tsid);
            assert(robot);

            // parse yaml
            double kp = node["kp"].as<double>();
            auto mask_str = node["mask"].as<std::string>();
            auto weight = node["weight"].as<double>();

            IWBC_ASSERT(mask_str.size() == 3, "CoM masks needs to be 3D (x y z), here:", mask_str);
            auto mask = convert_mask<3>(mask_str);

            // create the task
            auto task = std::make_shared<tsid::tasks::TaskComEquality>(task_name, *robot);
            task->Kp(kp * Vector::Ones(3));
            task->Kd(2.0 * task->Kp().cwiseSqrt());
            task->setMask(mask);

            // set the reference
            task->setReference(to_sample(robot->com(tsid->data())));

            // add to TSID
            tsid->addMotionTask(*task, weight, 1);

            return task;
        }

        RegisterYAML<tsid::tasks::TaskComEquality> __register_com_equality("com", make_com);

        ////// Posture //////
        std::shared_ptr<tsid::tasks::TaskBase> make_posture(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node)
        {
            assert(tsid);
            assert(robot);

            // parse yaml
            double kp = node["kp"].as<double>();
            auto weight = node["weight"].as<double>();
            auto ref_name = node["ref"].as<std::string>();

            IWBC_ASSERT(robot->model().referenceConfigurations.count(ref_name) == 1, "Reference name ", ref_name, " not found");
            auto ref_q = robot->model().referenceConfigurations[ref_name];

            // create the task
            auto task = std::make_shared<tsid::tasks::TaskJointPosture>(task_name, *robot);
            task->Kp(kp * Vector::Ones(robot->nv() - 6));
            task->Kd(2.0 * task->Kp().cwiseSqrt());
            Vector mask_post(robot->nv() - 6);
            if (!node["mask"]) {
                mask_post = Vector::Ones(robot->nv() - 6);
            }
            else {
                auto mask = node["mask"].as<std::string>();
                IWBC_ASSERT(mask.size() == mask_post.size(), "wrong size in posture mask, expected:", mask_post.size(), " got:", mask.size());
                mask_post = convert_mask<Eigen::Dynamic>(mask);
            }
            task->setMask(mask_post);

            // set the reference to the current position of the robot
            task->setReference(to_sample(ref_q.tail(robot->na())));

            // add the task
            tsid->addMotionTask(*task, weight, 1);

            return task;
        }
        RegisterYAML<tsid::tasks::TaskComEquality> __register_posture("posture", make_posture);

        ////// Bounds //////
        std::shared_ptr<tsid::tasks::TaskBase> make_bounds(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node)
        {
            assert(tsid);
            assert(robot);

            // parse yaml
            auto weight = node["weight"].as<double>();
            auto dt = node["dt"].as<double>(); // used to compute accelerations

            // create the task
            auto task = std::make_shared<tsid::tasks::TaskJointPosVelAccBounds>(task_name, *robot, dt, false);
            auto dq_max = robot->model().velocityLimit.tail(robot->na());
            auto ddq_max = dq_max / dt;
            task->setVelocityBounds(dq_max);
            task->setAccelerationBounds(ddq_max);
            auto q_lb = robot->model().lowerPositionLimit.tail(robot->na());
            auto q_ub = robot->model().upperPositionLimit.tail(robot->na());
            task->setPositionBounds(q_lb, q_ub);

            // add the task
            tsid->addMotionTask(*task, weight, 0);

            return task;
        }
        RegisterYAML<tsid::tasks::TaskComEquality> __register_bounds("bounds", make_bounds);

        ////// Contacts //////
        /// this looks like a task, but this does not derive from tsid::task::TaskBase
        std::shared_ptr<tsid::contacts::Contact6d> make_contact_task(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node)
        {
            assert(tsid);
            assert(robot);

            // parse yaml
            auto kp = node["kp"].as<double>();
            auto joint_name = node["joint"].as<std::string>();

            auto lxn = node["lxn"].as<double>();
            auto lyn = node["lyn"].as<double>();
            auto lxp = node["lxp"].as<double>();
            auto lyp = node["lyp"].as<double>();
            auto lz = node["lz"].as<double>();
            auto mu = node["mu"].as<double>();
            auto normal = node["normal"].as<std::vector<double>>();
            auto fmin = node["fmin"].as<double>();
            auto fmax = node["fmax"].as<double>();
            IWBC_ASSERT(normal.size() == 3, "normal size:", normal.size());
            IWBC_ASSERT(robot->model().existJointName(joint_name), joint_name, " does not exist!");

            // create the task
            Matrix3x contact_points(3, 4);
            contact_points << -lxn, -lxn, lxp, lxp,
                -lyn, lyp, -lyn, lyp,
                lz, lz, lz, lz;
            Eigen::Vector3d contact_normal(normal.data());
            auto contact_task = std::make_shared<tsid::contacts::Contact6d>(task_name, *robot, joint_name, contact_points, contact_normal, mu, fmin, fmax);
            contact_task->Kp(kp * Vector::Ones(6));
            contact_task->Kd(2.0 * contact_task->Kp().cwiseSqrt());
            auto contact_ref = robot->position(tsid->data(), robot->model().getJointId(joint_name));
            contact_task->setReference(contact_ref);

            // add the task
            tsid->addRigidContact(*contact_task, cst::w_force_feet);

            return contact_task;
        }

        ////// Self-collision (warning: add the task to TSID!) //////
        std::shared_ptr<tsid::tasks::TaskBase> make_self_collision(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node)
        {

            // retrieve parameters from YAML
            double kp = node["kp"].as<double>();
            auto tracked = node["tracked"].as<std::string>();
            auto weight = node["weight"].as<double>();
            auto coef = node["coef"].as<double>();

            std::unordered_map<std::string, double> avoided;
            for (const auto& a : node["avoided"])
                avoided[a.first.as<std::string>()] = a.second.as<double>();

            // create the task
            assert(tsid);
            assert(robot);
            auto task = std::make_shared<tsid::tasks::TaskSelfCollision>(task_name, *robot, tracked, avoided, coef);
            task->Kp(kp);
            task->Kd(2.0 * sqrt(task->Kp()));
            
            // add the task to TSID (side effect, be careful)
            tsid->addMotionTask(*task, weight, 1);

            return task;
        }
        RegisterYAML<tsid::tasks::TaskSelfCollision> __register_self_collision("self-collision", make_self_collision);

    } // namespace tasks
} // namespace inria_wbc