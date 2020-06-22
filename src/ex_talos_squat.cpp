#include <iostream>
#include <chrono>
#include <robot_dart/robot_dart_simu.hpp>
#include <signal.h>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include "talos_pos_tracking.hpp"
#include "trajectory_handler.hpp"

Eigen::VectorXd compute_spd(dart::dynamics::SkeletonPtr robot, Eigen::VectorXd targetpos)
{
    Eigen::VectorXd q = robot->getPositions();
    Eigen::VectorXd dq = robot->getVelocities();

    float stiffness = 10000;
    float damping = 100;
    int ndofs = robot->getNumDofs();
    Eigen::MatrixXd Kp = Eigen::MatrixXd::Identity(ndofs, ndofs);
    Eigen::MatrixXd Kd = Eigen::MatrixXd::Identity(ndofs, ndofs);

    for (std::size_t i = 0; i < robot->getNumDofs(); ++i)
    {
        Kp(i, i) = stiffness;
        Kd(i, i) = damping;
    }
    for (std::size_t i = 0; i < 6; ++i)
    {
        Kp(i, i) = 0;
        Kd(i, i) = 0;
    }

    Eigen::MatrixXd invM = (robot->getMassMatrix() + Kd * robot->getTimeStep()).inverse();
    Eigen::VectorXd p = -Kp * (q + dq * robot->getTimeStep() - targetpos);
    Eigen::VectorXd d = -Kd * dq;
    Eigen::VectorXd qddot = invM * (-robot->getCoriolisAndGravityForces() + p + d + robot->getConstraintForces());
    Eigen::VectorXd commands = p + d - Kd * qddot * robot->getTimeStep();
    return commands;
}

volatile sig_atomic_t stop;
void stopsig(int signum) {
    stop = 1;
}

int main()
{
    //////////////////// INIT DART ROBOT //////////////////////////////////////
    std::srand(std::time(NULL));
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    auto robot = std::make_shared<robot_dart::Robot>("talos/talos.urdf", packages);
    robot->set_position_enforced(true);
    robot->set_actuator_types(dart::dynamics::Joint::FORCE);
    // First 6-DOFs should always be FORCE if robot is floating base

    //////////////////// INIT STACK OF TASK //////////////////////////////////////
    float dt = 0.001;
    tsid_sot::TalosPosTracking::Params params = {robot->model_filename(),
                                                 "../res/models/talos_configurations.srdf",
                                                 dt};
    auto talos_sot = tsid_sot::TalosPosTracking(params, "../res/yaml/sot-squat.yaml", "", robot->mimic_dof_names());
    auto all_dofs = talos_sot.all_dofs();
    auto controllable_dofs = talos_sot.controllable_dofs();
    uint ncontrollable = controllable_dofs.size();
    Eigen::VectorXd cmd = Eigen::VectorXd::Zero(ncontrollable);
    robot->set_positions(talos_sot.q0(), all_dofs);

    //////////////////// INIT DART SIMULATION WORLD //////////////////////////////////////
    robot_dart::RobotDARTSimu simu(dt);
    simu.set_collision_detector("fcl");
#ifdef GRAPHIC
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(&simu);
    simu.set_graphics(graphics);
    graphics->look_at({0., 3.5, 2.}, {0., 0., 0.25});
    graphics->record_video("talos_squat.mp4");
#endif
    simu.add_robot(robot);
    simu.add_checkerboard_floor();

    //////////////////// DEFINE COM REFERENCES  //////////////////////////////////////
    auto com_init = talos_sot.com_init();
    auto com_final = com_init;
    com_final(2) -= 0.2;
    float trajectory_duration = 3;
    auto trajectory1 = trajectory_handler::compute_traj(com_init, com_final, dt, trajectory_duration);
    auto trajectory2 = trajectory_handler::compute_traj(com_final, com_init, dt, trajectory_duration);
    
    tsid::math::Vector3 ref;
    std::chrono::high_resolution_clock::time_point timer_start_, timer_end_;
    std::vector<double> timer_data;

    //////////////////// PLAY SIMULATION //////////////////////////////////////
    int k = 0;
    signal(SIGINT, stopsig); // to stop the simulation loop
    while (!stop)    
    {
        ++k;
        for (uint i = 0; i < trajectory1.size() && !stop; i++)
        {
            ref = (k % 2 == 0) ? trajectory1[i] : trajectory2[i];
            timer_start_ = std::chrono::high_resolution_clock::now();
            talos_sot.set_com_ref(ref);
            talos_sot.solve();
            auto cmd = compute_spd(robot->skeleton(), talos_sot.q(false));
            robot->set_commands(talos_sot.filter_cmd(cmd).tail(ncontrollable), controllable_dofs);
            timer_end_ = std::chrono::high_resolution_clock::now();
            auto t_diff =  std::chrono::duration_cast<std::chrono::microseconds>(timer_end_ - timer_start_).count();
            timer_data.push_back(static_cast<double>(t_diff) / 1000000.0);
            simu.step_world();
        }
    }

    double average = std::accumulate(timer_data.begin(), timer_data.end(), 0.0) / timer_data.size(); 
    std::cout << "Average Loop Time " << average << " seconds" << std::endl;
    return 0;
}
