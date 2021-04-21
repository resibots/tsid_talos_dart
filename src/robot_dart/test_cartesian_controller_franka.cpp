
#include <algorithm>
#include <boost/program_options.hpp>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <signal.h>

#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/sensor/force_torque.hpp>
#include <robot_dart/sensor/imu.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include "inria_wbc/behaviors/behavior.hpp"
#include "inria_wbc/controllers/pos_tracker.hpp"
#include "inria_wbc/exceptions.hpp"
#include "inria_wbc/robot_dart/cmd.hpp"


int main(int argc, char* argv[])
{
    try {
        // program options
        namespace po = boost::program_options;
        po::options_description desc("Test_controller options");
        // clang-format off
        desc.add_options()
        ("actuators,a", po::value<std::string>()->default_value("servo"), "actuator model torque/velocity/servo (always for position control) [default:servo]")
        ("behavior,b", po::value<std::string>()->default_value("../etc/circular_cartesian_trajectory.yaml"), "Configuration file of the tasks (yaml) [default: ../etc/circular_cartesian_trajectory.yaml]")
        ("big_window,b", "use a big window (nicer but slower) [default:true]")
        ("check_self_collisions", "check the self collisions (print if a collision)")
        ("collision,k", po::value<std::string>()->default_value("fcl"), "collision engine [default:fcl]")
        ("controller,c", po::value<std::string>()->default_value("../etc/franka_pos_tracker.yaml"), "Configuration file of the tasks (yaml) [default: ../etc/franka_pos_tracker.yaml]")
        ("duration,d", po::value<int>()->default_value(20), "duration in seconds [20]")
        ("enforce_position,e", po::value<bool>()->default_value(true), "enforce the positions of the URDF [default:true]")
        ("control_freq", po::value<int>()->default_value(1000), "set the control frequency")
        ("sim_freq", po::value<int>()->default_value(1000), "set the simulation frequency")
        ("ghost,g", "display the ghost (Pinocchio model)")
        ("help,h", "produce help message")
        ("mp4,m", po::value<std::string>(), "save the display to a mp4 video [filename]")
        ("push,p", po::value<std::vector<float>>(), "push the robot at t=x1 0.25 s")
        ("verbose,v", "verbose mode (controller)")
        ("log,l", po::value<std::vector<std::string>>()->default_value(std::vector<std::string>(),""), 
            "log the trajectory of a dart body [with urdf names] or timing or CoM or cost, example: -l timing -l com -l lf -l cost_com -l cost_lf")
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
        int sim_freq = vm["sim_freq"].as<int>();
        float dt = 1.0f/sim_freq;
        std::cout << "dt:" << dt << std::endl;

        //////////////////// INIT DART ROBOT //////////////////////////////////////
        std::srand(std::time(NULL));

        std::vector<std::pair<std::string, std::string>> packages = {{"franka_description", "franka/franka_description"}};
        std::string urdf = "franka/franka.urdf";
        auto robot = std::make_shared<robot_dart::Robot>(urdf, packages);
        robot->fix_to_world();
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
        graphics->look_at({0., 3., 1.}, {0., 0., 0.});
        if (vm.count("mp4"))
            graphics->record_video(vm["mp4"].as<std::string>());
#endif
        simu.add_robot(robot);
        simu.add_checkerboard_floor();

        ///// CONTROLLER
        auto controller_path = vm["controller"].as<std::string>();
        auto controller_config = IWBC_CHECK(YAML::LoadFile(controller_path));

        controller_config["CONTROLLER"]["base_path"] = "../etc";// we assume that we run in ./build
        controller_config["CONTROLLER"]["urdf"] = robot->model_filename();
        controller_config["CONTROLLER"]["mimic_dof_names"] = robot->mimic_dof_names();
        controller_config["CONTROLLER"]["verbose"] = verbose;
        int control_freq = vm["control_freq"].as<int>();

        auto controller_name = IWBC_CHECK(controller_config["CONTROLLER"]["name"].as<std::string>());
        auto controller = inria_wbc::controllers::Factory::instance().create(controller_name, controller_config);
        auto controller_pos = std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller);
        IWBC_ASSERT(controller_pos, "we expect a PosTracker here");

        auto behavior_path = vm["behavior"].as<std::string>();
        auto behavior_config = IWBC_CHECK(YAML::LoadFile(behavior_path));
        auto behavior_name = IWBC_CHECK(behavior_config["BEHAVIOR"]["name"].as<std::string>());
        auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller, behavior_config);
        IWBC_ASSERT(behavior, "invalid behavior");


        auto all_dofs = controller->all_dofs();
        auto controllable_dofs = controller->controllable_dofs();
        robot->set_positions(controller->q0(), all_dofs);

        uint ncontrollable = controllable_dofs.size();

        if (vm.count("log")) {
            std::ofstream ofs("all_dofs.dat");
            for (auto& c : all_dofs)
                ofs << c << std::endl;
        }

        //////////////////// START SIMULATION //////////////////////////////////////
        simu.set_control_freq(control_freq); // 1000 Hz
        double time_simu = 0, time_cmd = 0, time_solver = 0, max_time_solver = 0, min_time_solver = 1e10;
        int it_simu = 0, it_cmd = 0;


        // the main loop
        using namespace std::chrono;
        while (simu.scheduler().next_time() < vm["duration"].as<int>() && !simu.graphics()->done()) {
            double time_step_solver = 0, time_step_cmd = 0, time_step_simu = 0;

            // update the sensors
            inria_wbc::controllers::SensorData sensor_data;
            // left foot
            sensor_data["lf_torque"] = Eigen::Vector3d::Zero();
            sensor_data["lf_force"] = Eigen::Vector3d::Zero();
            // right foot
            sensor_data["rf_torque"] = Eigen::Vector3d::Zero();
            sensor_data["rf_force"] = Eigen::Vector3d::Zero();
            // accelerometer
            sensor_data["acceleration"] = Eigen::Vector3d::Zero();
            sensor_data["velocity"] = Eigen::Vector3d::Zero();
            // joint positions (excluding floating base)
            sensor_data["positions"] = robot->skeleton()->getPositions().tail(ncontrollable);


            // step the command
            Eigen::VectorXd cmd;
            if (simu.schedule(simu.control_freq())) {
                auto t1_solver = high_resolution_clock::now();
                behavior->update(sensor_data);
                auto q = controller->q(false);
                auto t2_solver = high_resolution_clock::now();
                time_step_solver = duration_cast<microseconds>(t2_solver - t1_solver).count();

                auto t1_cmd = high_resolution_clock::now();
                if (vm["actuators"].as<std::string>() == "velocity" || vm["actuators"].as<std::string>() == "servo")
                    cmd = inria_wbc::robot_dart::compute_velocities(robot->skeleton(), q, 1./control_freq);
                else // torque
                    cmd = inria_wbc::robot_dart::compute_spd(robot->skeleton(), q, 1./control_freq);
                auto t2_cmd = high_resolution_clock::now();
                time_step_cmd = duration_cast<microseconds>(t2_cmd - t1_cmd).count();

                robot->set_commands(controller->filter_cmd(cmd).tail(ncontrollable), controllable_dofs);
                ++it_cmd;

                max_time_solver = std::max(time_step_solver, max_time_solver);
                min_time_solver = std::min(time_step_solver, min_time_solver);
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
                else
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
                          << "\tit. solver:" << t_solver << " ms [" << min_time_solver / 1000. << " " << max_time_solver / 1000. << "]"
                          << "\tit. cmd:" << t_cmd << " ms"
                          << std::endl;
                std::ostringstream oss;
                oss.precision(3);
                oss << "[Sim: " << t_sim << " ms]" << std::endl;
                oss << "[Solver: " << t_solver << " ms]" << std::endl;
                oss << "[Cmd: " << t_cmd << " ms]" << std::endl;
#ifdef GRAPHIC
                if (!vm.count("mp4"))
                    simu.set_text_panel(oss.str());
#endif
                it_simu = 0;
                it_cmd = 0;
                time_cmd = 0;
                time_simu = 0;
                time_solver = 0;
                min_time_solver = 1e10;
                max_time_solver = 0;
            }
        }
    }
    catch (std::exception& e) {
        std::string red = "\x1B[31m";
        std::string rst = "\x1B[0m";
        std::string bold = "\x1B[1m";
        std::cout << red << bold << "Error (exception): " << rst << e.what() << std::endl;
    }
    return 0;
}
