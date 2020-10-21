
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/sensor/force_torque.hpp>

#include <signal.h>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include "inria_wbc/behaviors/factory.hpp"

Eigen::Vector2d evaluate_cop(
    const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
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
    std::cout << "Global raw CoP :   x : " << cop(0) << "  y : " << cop(1) << std::endl;
    return cop;
}

int main(int argc, char* argv[])
{
    // take the name of the behavior as a argument
    std::string sot_config_path = argc > 1 ? argv[1] : "../etc/squat.yaml";
    std::cout << "using configuration:" << sot_config_path << std::endl;
    // dt of the simulation and the controller
    float dt = 0.001;

    //////////////////// INIT DART ROBOT //////////////////////////////////////
    std::srand(std::time(NULL));
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    auto robot = std::make_shared<robot_dart::Robot>("talos/talos.urdf", packages);
    robot->set_position_enforced(true);
    robot->set_actuator_types("velocity");

    //////////////////// INIT DART SIMULATION WORLD //////////////////////////////////////
    robot_dart::RobotDARTSimu simu(dt);
    simu.set_collision_detector("fcl");

#ifdef GRAPHIC
    robot_dart::gui::magnum::GraphicsConfiguration configuration;
    configuration.width = 1280;
    configuration.height = 960;
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(configuration);
    simu.set_graphics(graphics);
    graphics->look_at({3.5, -2, 2.2}, {0., 0., 1.4});
#endif
    simu.add_robot(robot);
    simu.add_checkerboard_floor();

    //////////////////// INIT STACK OF TASK //////////////////////////////////////

    inria_wbc::controllers::TalosBaseController::Params params = {robot->model_filename(),
        "../etc/talos_configurations.srdf",
        sot_config_path,
        "",
        dt,
        false,
        robot->mimic_dof_names()};

    std::string behavior_name;
    YAML::Node config = YAML::LoadFile(sot_config_path);
    inria_wbc::utils::parse(behavior_name, "name", config, false, "BEHAVIOR");
    // params = inria_wbc::controllers::parse_params(config);

    auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, params);

    auto controller = behavior->controller();
    auto all_dofs = controller->all_dofs();
    auto controllable_dofs = controller->controllable_dofs();
    robot->set_positions(controller->q0(), all_dofs);
    uint ncontrollable = controllable_dofs.size();
    std::pair<Eigen::Vector6d, Eigen::Vector6d> lf_torque_force;
    std::pair<Eigen::Vector6d, Eigen::Vector6d> rf_torque_force;

    auto ft_sensor_left = simu.add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_left_6_joint");
    auto ft_sensor_right = simu.add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_right_6_joint");

    auto t_from_child_body = robot->joint("leg_left_6_joint")->getTransformFromChildBodyNode().rotation();
    //////////////////// START SIMULATION //////////////////////////////////////
    simu.set_control_freq(1000); // 1000 Hz
    while (!simu.graphics()->done()) {
        if (simu.schedule(simu.control_freq())) {
            lf_torque_force = robot->force_torque(robot->joint_index("leg_left_6_joint"));
            std::cout << "FT pose" << ft_sensor_left->pose().translation().transpose() << std::endl;
            std::cout << "FT pose" << ft_sensor_right->pose().translation().transpose() << std::endl;
            std::cout << "pos from child:" << t_from_child_body.transpose() << std::endl;
            std::cout << "pinocchio:" << controller->right_ankle().translation().transpose() << std::endl;
            auto lf2 = ft_sensor_left->force();
            rf_torque_force = robot->force_torque(robot->joint_index("leg_right_6_joint"));
            evaluate_cop(controller->left_ankle().translation(),
                controller->right_ankle().translation(),
                ft_sensor_left->torque(), ft_sensor_left->force(),
                ft_sensor_right->torque(), ft_sensor_right->force());
            std::cout << "CoM:" << robot->com().transpose() << std::endl;
        }
        simu.step_world();
    }

    return 0;
}
