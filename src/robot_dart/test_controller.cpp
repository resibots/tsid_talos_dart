
#include <algorithm>
#include <boost/program_options.hpp>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <signal.h>
#include <tsid/robots/robot-wrapper.hpp>

#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/sensor/force_torque.hpp>
#include <robot_dart/sensor/imu.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include "inria_wbc/behaviors/factory.hpp"
#include "inria_wbc/estimators/cop.hpp"

Eigen::VectorXd compute_spd(dart::dynamics::SkeletonPtr robot, const Eigen::VectorXd& targetpos)
{
    Eigen::VectorXd q = robot->getPositions();
    Eigen::VectorXd dq = robot->getVelocities();

    float stiffness = 10000;
    float damping = 100;
    int ndofs = robot->getNumDofs();
    Eigen::MatrixXd Kp = Eigen::MatrixXd::Identity(ndofs, ndofs);
    Eigen::MatrixXd Kd = Eigen::MatrixXd::Identity(ndofs, ndofs);

    for (std::size_t i = 0; i < robot->getNumDofs(); ++i) {
        Kp(i, i) = stiffness;
        Kd(i, i) = damping;
    }
    for (std::size_t i = 0; i < 6; ++i) {
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

Eigen::VectorXd compute_velocities(dart::dynamics::SkeletonPtr robot, const Eigen::VectorXd& targetpos, double dt)
{
    Eigen::VectorXd q = robot->getPositions();
    Eigen::VectorXd vel = (targetpos - q) / dt;
    return vel;
}

int main(int argc, char* argv[])
{
    // program options
    namespace po = boost::program_options;
    po::options_description desc("Test_controller options");
    // clang-format off
    desc.add_options()
    ("help,h", "produce help message")
    ("conf,c", po::value<std::string>()->default_value("../etc/squat.yaml"), "Configuration file of the tasks (yaml) [default: ../etc/squat.yaml]")
    ("fast,f", "fast (simplified) Talos [default: false]")
    ("big_window,b", "use a big window (nicer but slower) [default:true]")
    ("actuators,a", po::value<std::string>()->default_value("torque"), "actuator model torque/velocity/servo (always for position control) [default:torque]")
    ("enforce_position,e", po::value<bool>()->default_value(true), "enforce the positions of the URDF [default:true]")
    ("collision,k", po::value<std::string>()->default_value("fcl"), "collision engine [default:fcl]")
    ("mp4,m", po::value<std::string>(), "save the display to a mp4 video [filename]")
    ("duration,d", po::value<int>()->default_value(20), "duration in seconds [20]")
    ("push,p", po::value<std::vector<float>>(), "push the robot at t=x1 0.25 s")
    ("ghost,g", "display the ghost (Pinocchio model)")
    ("verbose,v", "verbose mode (controller)")
    ("log,l", po::value<std::vector<std::string>>()->default_value(std::vector<std::string>(),""), "log the trajectory of a dart body [with urdf names] or timing or CoM, example: -l timing -l com -l lf")
    ;
    // clang-format on
    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    }
    catch (po::too_many_positional_options_error& e) {
        // A positional argument like `opt2=option_value_2` was given
        std::cerr << e.what() << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    }
    catch (po::error_with_option_name& e) {
        // Another usage error occurred
        std::cerr << e.what() << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    }

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }

    // clang-format off
    std::cout<< "------ CONFIGURATION ------" << std::endl;
    std::ostringstream oss_conf;
    for (const auto& kv : vm){
        oss_conf << kv.first << " ";
        try { oss_conf << kv.second.as<std::string>();
        } catch(...) {/* do nothing */ }
        try { oss_conf << kv.second.as<bool>();
        } catch(...) {/* do nothing */ }
        try { oss_conf << kv.second.as<int>();
        } catch(...) {/* do nothing */ }
        oss_conf << std::endl;
    }
    std::cout << oss_conf.str();
    std::cout << "--------------------------" << std::endl;
    // clang-format on

    bool verbose = (vm.count("verbose") != 0);
    std::map<std::string, std::shared_ptr<std::ofstream>> log_files;
    for (auto& x : vm["log"].as<std::vector<std::string>>())
        log_files[x] = std::make_shared<std::ofstream>((x + ".dat").c_str());

    // dt of the simulation and the controller
    float dt = 0.001;
    std::cout << "dt:" << dt << std::endl;

    //////////////////// INIT DART ROBOT //////////////////////////////////////
    std::srand(std::time(NULL));
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    std::string urdf = vm.count("fast") ? "talos/talos_fast.urdf" : "talos/talos.urdf";
    auto robot = std::make_shared<robot_dart::Robot>(urdf, packages);
    robot->set_position_enforced(vm["enforce_position"].as<bool>());
    robot->set_actuator_types(vm["actuators"].as<std::string>());

    //////////////////// INIT DART SIMULATION WORLD //////////////////////////////////////
    robot_dart::RobotDARTSimu simu(dt);
    simu.set_collision_detector(vm["collision"].as<std::string>());

#ifdef GRAPHIC
    robot_dart::gui::magnum::GraphicsConfiguration configuration;
    if (vm.count("big_window")) {
        configuration.width = 1280;
        configuration.height = 960;
    }
    else {
        configuration.width = 800;
        configuration.height = 500;
    }

    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(configuration);
    simu.set_graphics(graphics);
    graphics->look_at({3.5, -2, 2.2}, {0., 0., 1.4});
    if (vm.count("mp4"))
        graphics->record_video(vm["mp4"].as<std::string>());
#endif
    simu.add_robot(robot);
    simu.add_checkerboard_floor();

    //////////////////// INIT STACK OF TASK //////////////////////////////////////
    std::string sot_config_path = vm["conf"].as<std::string>();
    inria_wbc::controllers::TalosBaseController::Params params = {robot->model_filename(),
        "../etc/talos_configurations.srdf",
        sot_config_path,
        "",
        0.001,
        verbose,
        robot->mimic_dof_names()};

    std::string behavior_name;
    YAML::Node config = YAML::LoadFile(sot_config_path);
    inria_wbc::utils::parse(behavior_name, "name", config, false, "BEHAVIOR");

    auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, params);

    auto controller = behavior->controller();
    auto all_dofs = controller->all_dofs();
    auto floating_base = all_dofs;
    floating_base.resize(6);

    auto controllable_dofs = controller->controllable_dofs();
    robot->set_positions(controller->q0(), all_dofs);

    uint ncontrollable = controllable_dofs.size();

    if (vm.count("log")) {
        std::ofstream ofs("all_dofs.dat");
        for (auto& c : all_dofs)
            ofs << c << std::endl;
    }

    //////////////////// START SIMULATION //////////////////////////////////////
    simu.set_control_freq(1000); // 1000 Hz
    double time_simu = 0, time_cmd = 0, time_solver = 0;
    int it_simu = 0, it_cmd = 0;

    std::shared_ptr<robot_dart::Robot> ghost;
    if (vm.count("ghost")) {
        ghost = robot->clone_ghost();
        ghost->skeleton()->setPosition(4, -1.57);
        ghost->skeleton()->setPosition(5, 1.1);
        simu.add_robot(ghost);
    }

    inria_wbc::estimators::Cop cop_estimator;
    auto ft_sensor_left = simu.add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_left_6_joint");
    auto ft_sensor_right = simu.add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_right_6_joint");
    robot_dart::sensor::IMUConfig imu_config;
    imu_config.body = robot->body_node("imu_link"); // choose which body the sensor is attached to
    imu_config.frequency = 1000; // update rate of the sensor
    auto imu = simu.add_sensor<robot_dart::sensor::IMU>(imu_config);
    if (log_files.find("q0_dart") != log_files.end())
        (*log_files["q0_dart"]) << controller->q0().transpose() << std::endl;
    if (log_files.find("q0_pinocchio") != log_files.end())
        (*log_files["q0_pinocchio"]) << controller->robot()->model().referenceConfigurations["pal_start"].transpose() << std::endl;

    // the main loop
    using namespace std::chrono;
    while (simu.scheduler().next_time() < vm["duration"].as<int>() && !simu.graphics()->done()) {
        double time_step_solver = 0, time_step_cmd = 0, time_step_simu = 0;

        inria_wbc::controllers::SensorData SensorData = {
            ft_sensor_left->torque(),
            ft_sensor_left->force(),
            ft_sensor_right->torque(),
            ft_sensor_right->force(),
            imu->linear_acceleration(),
            robot->com_velocity().tail<3>(),
            robot->skeleton()->getPositions().tail(ncontrollable)};

        // step the command
        Eigen::VectorXd cmd;
        if (simu.schedule(simu.control_freq())) {
            auto t1_solver = high_resolution_clock::now();
            bool solution_found = behavior->update(SensorData);
            auto q = controller->q(false);
            auto t2_solver = high_resolution_clock::now();
            time_step_solver = duration_cast<microseconds>(t2_solver - t1_solver).count();

            if (solution_found) {
                auto t1_cmd = high_resolution_clock::now();
                if (vm["actuators"].as<std::string>() == "velocity" || vm["actuators"].as<std::string>() == "servo")
                    cmd = compute_velocities(robot->skeleton(), q, dt);
                else // torque
                    cmd = compute_spd(robot->skeleton(), q);
                auto t2_cmd = high_resolution_clock::now();
                time_step_cmd = duration_cast<microseconds>(t2_cmd - t1_cmd).count();

                robot->set_commands(controller->filter_cmd(cmd).tail(ncontrollable), controllable_dofs);
                if (ghost) {
                    Eigen::VectorXd translate_ghost = Eigen::VectorXd::Zero(6);
                    translate_ghost(0) -= 1;
                    ghost->set_positions(controller->filter_cmd(q).tail(ncontrollable), controllable_dofs);
                    ghost->set_positions(q.head(6) + translate_ghost, floating_base);
                }
            }
            else {
                std::cerr << "Solver failed! aborting" << std::endl;
                return -1;
            }
            ++it_cmd;
        }
        bool push = false;
        if (vm.count("push")) {
            auto pv = vm["push"].as<std::vector<float>>();
            for (auto& p : pv) {
                if (simu.scheduler().current_time() > p && simu.scheduler().current_time() < p + 0.5) {
                    robot->set_external_force("base_link", Eigen::Vector3d(-150, 0, 0));
                    push = true;
                }
                if (simu.scheduler().current_time() > p + 0.25)
                    robot->clear_external_forces();
            }
        }

        // step the simulation
        {
            auto t1_simu = high_resolution_clock::now();
            simu.step_world();
            auto t2_simu = high_resolution_clock::now();
            time_step_simu = duration_cast<microseconds>(t2_simu - t1_simu).count();
            ++it_simu;
        }

        // log if needed
        for (auto& x : log_files) {
            if (x.first == "timing")
                (*x.second) << time_step_solver / 1000.0 << "\t" << time_step_cmd / 1000.0 << "\t" << time_step_simu / 1000.0 << std::endl;
            else if (x.first == "cmd")
                (*x.second) << cmd.transpose() << std::endl;
            else if (x.first == "com_dart")
                (*x.second) << robot->com().transpose() << std::endl;
            else if (x.first == "com_tsid")
                (*x.second) << controller->com().transpose() << std::endl;
            else if (x.first == "cop")
                (*x.second) << controller->cop().transpose() << std::endl;
            else if (x.first == "ft")
                (*x.second) << ft_sensor_left->torque().transpose() << " " << ft_sensor_left->force().transpose() << " "
                            << ft_sensor_right->torque().transpose() << " " << ft_sensor_right->force().transpose() << std::endl;
            else if (x.first != "q0_dart" && x.first != "q0_pinocchio")
                (*x.second) << robot->body_pose(x.first).translation().transpose() << std::endl;
        }
        // print timing information
        time_simu += time_step_simu;
        time_cmd += time_step_cmd;
        time_solver += time_step_solver;
        if (it_simu == 100) {
            double t_sim = time_simu / it_simu / 1000.;
            double t_cmd = time_cmd / it_cmd / 1000.;
            double t_solver = time_solver / it_cmd / 1000.;

            std::cout << "t=" << simu.scheduler().current_time()
                      << "\tit. simu: " << t_sim << " ms"
                      << "\tit. solver:" << t_solver << " ms"
                      << "\tit. cmd:" << t_cmd << " ms"
                      << std::endl;
            std::ostringstream oss;
            oss.precision(3);
            oss << "[Sim: " << t_sim << " ms]" << std::endl;
            oss << "[Solver: " << t_solver << " ms]" << std::endl;
            oss << "[Cmd: " << t_cmd << " ms]" << std::endl;
            if (push)
                oss << "pushing..." << std::endl;

                //oss << oss_conf.str();
#ifdef GRAPHIC
            if (!vm.count("mp4"))
                simu.set_text_panel(oss.str());
#endif
            it_simu = 0;
            it_cmd = 0;
            time_cmd = 0;
            time_simu = 0;
            time_solver = 0;
        }
    }
    return 0;
}
