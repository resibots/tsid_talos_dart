#include <chrono>
#include <ctime>
#include <iostream>
#include <map>
#include <stdio.h>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/sensor/force_torque.hpp>
#include <robot_dart/sensor/imu.hpp>
#include <robot_dart/sensor/torque.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include "inria_wbc/behaviors/behavior.hpp"
#include "inria_wbc/robot_dart/cmd.hpp"

#include "inria_wbc/behaviors/behavior.hpp"
#include "inria_wbc/controllers/pos_tracker.hpp"
#include "inria_wbc/exceptions.hpp"
#include "inria_wbc/robot_dart/cmd.hpp"
#include "inria_wbc/robot_dart/external_collision_detector.hpp"
#include "inria_wbc/robot_dart/self_collision_detector.hpp"
#include "inria_wbc/robot_dart/utils.hpp"

#include "utest.hpp"

namespace cst {
    static constexpr double dt = 0.001;
    static constexpr double duration = 10;
    static constexpr double frequency = 1000;
    static constexpr double tolerance = 5e-3;
    static const std::string ref_path = "../../tests/ref_test_franka.yaml";
    static const std::string base_path = "../../etc/franka/";

} // namespace cst

namespace inria_wbc {
    namespace tests {
        struct SE3TaskData {
            std::string name;
            std::string tracked;
            std::string mask;
            double weight;
            double error_dart = 0.0;
            double error_tsid = 0.0;
        };
    } // namespace tests
} // namespace inria_wbc

namespace y = YAML;

// for timestamps
inline std::string date()
{
    char date[30];
    time_t date_time;
    time(&date_time);
    strftime(date, 30, "%Y-%m-%d_%H-%M-%S", localtime(&date_time));
    return std::string(date);
}

// we parse again:
// - to check that the controller parses as expected
// - and to get additional info (mask, etc.)
std::vector<inria_wbc::tests::SE3TaskData> parse_tasks(const y::Node& config)
{
    std::vector<inria_wbc::tests::SE3TaskData> tasks;
    auto c = IWBC_CHECK(config["CONTROLLER"]);
    auto path = IWBC_CHECK(c["base_path"].as<std::string>());
    auto task_file = IWBC_CHECK(c["tasks"].as<std::string>());
    auto p = path + "/" + task_file;
    auto task_list = IWBC_CHECK(y::LoadFile(p));
    for (auto it = task_list.begin(); it != task_list.end(); ++it) {
        auto name = IWBC_CHECK(it->first.as<std::string>());
        auto type = IWBC_CHECK(it->second["type"].as<std::string>());
        if (type == "se3") {
            auto weight = IWBC_CHECK(it->second["weight"].as<double>());
            auto mask = IWBC_CHECK(it->second["mask"].as<std::string>());
            auto tracked = IWBC_CHECK(it->second["tracked"].as<std::string>());
            tasks.push_back({name, tracked, mask, weight});
        }
    }

    // std::cout << "SE3 TASKS (frames): ";
    // for (auto& x : tasks)
    //     std::cout << x.name << "[" << x.tracked << "] ";
    // std::cout << std::endl;

    return tasks;
}

void test_behavior(utest::test_t test,
    const std::string& controller_path,
    const std::string& behavior_path,
    robot_dart::RobotDARTSimu& simu,
    const std::shared_ptr<robot_dart::Robot>& robot,
    const std::string& actuator_type,
    const y::Node& ref,
    std::shared_ptr<y::Emitter> yout,
    bool verbose = false)
{
    // ----------------------- init -----------------------

    y::Node c_config = IWBC_CHECK(y::LoadFile(controller_path));
    c_config["CONTROLLER"]["base_path"] = cst::base_path;
    c_config["CONTROLLER"]["urdf"] = robot->model_filename();
    c_config["CONTROLLER"]["mimic_dof_names"] = robot->mimic_dof_names();
    c_config["CONTROLLER"]["verbose"] = verbose;

    // get the controller
    auto controller_name = IWBC_CHECK(c_config["CONTROLLER"]["name"].as<std::string>());
    if (actuator_type == "torque") // force closed-loop
        c_config["CONTROLLER"]["closed_loop"] = true;
    auto controller = inria_wbc::controllers::Factory::instance().create(controller_name, c_config);
    UTEST_CHECK(test, controller.get() != 0);
    auto p_controller = std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller);
    UTEST_CHECK(test, p_controller.get() != 0);
    UTEST_CHECK(test, !p_controller->tasks().empty());

    // get the list of SE3 tasks (to test the tracking)
    auto se3_tasks = parse_tasks(c_config);
    UTEST_CHECK(test, !se3_tasks.empty());

    // get the behavior (trajectories)
    y::Node b_config = IWBC_CHECK(y::LoadFile(behavior_path));
    auto behavior_name = IWBC_CHECK(b_config["BEHAVIOR"]["name"].as<std::string>());
    auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller, b_config);
    UTEST_CHECK(test, behavior);

    // a few useful variables
    auto all_dofs = controller->all_dofs();
    auto floating_base = all_dofs;
    floating_base.resize(6);
    auto controllable_dofs = controller->controllable_dofs();
    uint ncontrollable = controllable_dofs.size();
    double time_simu = 0, time_cmd = 0, time_solver = 0;
    int it_simu = 0, it_cmd = 0;

    // init the robot
    robot->set_positions(controller->q0(), all_dofs);

    // ----------------------- the main loop -----------------------
    using namespace std::chrono;
    inria_wbc::controllers::SensorData sensor_data;

    Eigen::VectorXd cmd;
    while (simu.scheduler().next_time() < cst::duration) {
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
        sensor_data["positions"] = robot->positions(controller->controllable_dofs(false));
        sensor_data["joint_velocities"] = robot->velocities(controller->controllable_dofs(false));


        // command
        if (simu.schedule(simu.control_freq())) {
            auto t1_solver = high_resolution_clock::now();
            behavior->update(sensor_data);
            auto q = controller->q(false);
            auto t2_solver = high_resolution_clock::now();
            time_step_solver = duration_cast<microseconds>(t2_solver - t1_solver).count();

            auto t1_cmd = high_resolution_clock::now();
            if (actuator_type == "velocity" || actuator_type == "servo")
                cmd = inria_wbc::robot_dart::compute_velocities(robot, q, cst::dt);
            else if (actuator_type == "spd")
                cmd = inria_wbc::robot_dart::compute_spd(robot, q, cst::dt);
            else // torque
                cmd = controller->tau(false);

            auto t2_cmd = high_resolution_clock::now();
            time_step_cmd = duration_cast<microseconds>(t2_cmd - t1_cmd).count();

            robot->set_commands(controller->filter_cmd(cmd).tail(ncontrollable), controllable_dofs);
            ++it_cmd;
        }

        // step the simulation
        {
            auto t1_simu = high_resolution_clock::now();
            simu.step_world();
            auto t2_simu = high_resolution_clock::now();
            time_step_simu = duration_cast<microseconds>(t2_simu - t1_simu).count();
            ++it_simu;
        }

        // tracking information
        for (auto& t : se3_tasks) {
            Eigen::VectorXd pos_dart, pos_tsid;
            if (robot->joint(t.tracked)) {
                auto node = robot->joint(t.tracked)->getParentBodyNode();
                auto tf_bd_world = node->getWorldTransform();
                auto tf_bd_joint = robot->joint(t.tracked)->getTransformFromParentBodyNode();
                pos_dart = (tf_bd_world * tf_bd_joint).translation();
                pos_tsid = controller->model_joint_pos(t.tracked).translation();
            }
            else {
                pos_dart = robot->body_pose(t.tracked).translation();
                // Q: doesn't TSID create a frame for each joint?
                pos_tsid = controller->model_frame_pos(t.tracked).translation();
            }
            // masks? // TODO also compute angular error
            auto pos_ref = p_controller->get_se3_ref(t.name).translation();
            if (t.weight > 0) {
                double error_dart = 0.;
                double error_tsid = 0.;
                IWBC_ASSERT(t.mask.size() == 6, "mask size=", t.mask.size());
                for (int i = 0; i < 3; ++i) {
                    if (t.mask[i] == '1') {
                        error_dart += (pos_dart[i] - pos_ref[i]) * (pos_dart[i] - pos_ref[i]);
                        error_tsid += (pos_tsid[i] - pos_ref[i]) * (pos_tsid[i] - pos_ref[i]);
                    }
                }
                t.error_dart += sqrt(error_dart);
                t.error_tsid += sqrt(error_tsid);
            }
        }

        // timing information
        time_simu += time_step_simu;
        time_cmd += time_step_cmd;
        time_solver += time_step_solver;
    }
    // time report
    double t_sim = time_simu / it_simu / 1000.;
    double t_cmd = time_cmd / it_cmd / 1000.;
    double t_solver = time_solver / it_cmd / 1000.;

    UTEST_INFO(test, std::string("TIME: ") + "\ttotal:" + std::to_string((time_simu + time_cmd + time_solver) / 1e6) + " s" + "\t[" + std::to_string((t_sim + t_cmd + t_solver)) + " ms / it.]" + "\tsimu: " + std::to_string(t_sim) + " ms/it" + "\tsolver:" + std::to_string(t_solver) + " ms/it" + "\tcmd:" + std::to_string(t_cmd) + " ms/it");
    if (yout)
        (*yout) << y::Key << "time" << y::Value
                << std::vector<double>{(time_simu + time_cmd + time_solver) / 1e6, (t_sim + t_cmd + t_solver), t_sim, t_solver, t_cmd};

    // error report for se3 tasks
    for (auto& t : se3_tasks) {
        t.error_tsid /= simu.scheduler().current_time() * 1000;
        t.error_dart /= simu.scheduler().current_time() * 1000;
        UTEST_INFO(test, std::string("task: ") + t.name + " [" + t.tracked + "]" + "pos error TSID:" + std::to_string(t.error_tsid) + " " + "pos error DART:" + std::to_string(t.error_dart) + " " + "\t(mask =" + t.mask + ",weight=" + std::to_string(t.weight) + ")");
        if (yout)
            (*yout) << y::Key << t.name << y::Value << std::vector<double>{t.error_tsid, t.error_dart};
    }

    // comparisons to the reference values
    try {
        // SE3 tasks
        for (auto& t : se3_tasks) {
            auto e = IWBC_CHECK(ref[t.name].as<std::vector<double>>());
            UTEST_CHECK_MESSAGE(test, t.name + " tolerance --> " + std::to_string(t.error_tsid - e[0]) + " > " + std::to_string(cst::tolerance), t.error_tsid <= e[0] + cst::tolerance);
            UTEST_CHECK_MESSAGE(test, t.name + " tolerance --> " + std::to_string(t.error_dart - e[1]) + " > " + std::to_string(cst::tolerance), t.error_dart <= e[1] + cst::tolerance);
        }
    }
    catch (std::exception& e) {
        UTEST_ERROR(test, std::string("error in ref comparison:") + e.what());
    }
}

void test_behavior(utest::test_t test,
    std::string controller_path,
    std::string behavior_path,
    std::string actuators,
    std::string coll,
    std::string urdf,
    y::Node ref,
    std::shared_ptr<y::Emitter> yout)
{
    if (yout)
        (*yout) << y::Key << urdf << y::BeginMap;

    y::Node node;
    try {
        node = ref[controller_path][behavior_path][actuators][coll][urdf];
    }
    catch (std::exception& e) {
        UTEST_ERROR(test, std::string("error in getting the ref node:") + e.what());
        node = ref;
    }
    // create the simulator and the robot
    std::vector<std::pair<std::string, std::string>> packages = {{"franka_description", "franka/franka_description"}};
    auto robot = std::make_shared<robot_dart::Robot>(urdf, packages);
    if (actuators == "spd")
        robot->set_actuator_types("torque");
    else
        robot->set_actuator_types(actuators);
    robot->fix_to_world();
    robot->set_position_enforced(true);

    robot_dart::RobotDARTSimu simu(cst::dt);
    simu.set_collision_detector(coll);
    simu.set_control_freq(cst::frequency);
    simu.add_robot(robot);
    simu.add_checkerboard_floor();

#ifdef GRAPHIC
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>();
    simu.set_graphics(graphics);
    graphics->look_at({3.5, -2, 2.2}, {0., 0., 1.4});
    // we always save the video (useful for bug reports)
    graphics->record_video("test_result_franka.mp4");
#endif

    try {
        test_behavior(test, controller_path, behavior_path, simu, robot, actuators, node, yout);
    }
    catch (std::exception& e) {
        UTEST_ERROR(test, std::string("exception when running the behavior:") + e.what());
    }
    if (yout)
        (*yout) << y::EndMap;
}

int main(int argc, char** argv)
{

    // program options
    namespace po = boost::program_options;
    po::options_description desc("Test_controller options");
    // clang-format off
        desc.add_options()
        ("generate_ref,g", "generate a reference file")
        ("n_threads,n", po::value<int>()->default_value(-1), "run tests in parallel (default = number of cores)")
        ("single,s", po::value<std::vector<std::string> >()->multitoken(), "run a single test, args: controller_path behavior_path actuators coll enable_stabilizer urdf")
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

    int n_threads = vm["n_threads"].as<int>();
#ifdef GRAPHIC
    n_threads = 1;
#endif

    utest::TestSuite test_suite;

    // we load the reference
    y::Node ref;

    try {
        ref = IWBC_CHECK(y::LoadFile(cst::ref_path));
    }
    catch (std::exception& e) {
        std::cerr << "cannot load reference file, path=" << cst::ref_path << " what:" << e.what() << std::endl;
    }
    // we write the results in yaml file, for future comparisons and analysis
    auto yout = std::make_shared<y::Emitter>();
    (*yout) << y::BeginMap;
    // the YAMl file has a timestamp (so that we can archive them)
    (*yout) << y::Key << "timestamp" << y::Value << date();

    if (vm.count("single")) {
        auto args = vm["single"].as<std::vector<std::string>>();
        if (args.size() != 5) {
            std::cerr << " Wrong number of arguments: (" << args.size() << ")"
                      << "controller_path behavior_path actuators coll urdf"
                      << std::endl;
            return 1;
        }
        std::string name = args[0] + " "
            + args[1] + " "
            + args[2] + " "
            + args[3] + " "
            + args[4] + " ";
        std::cout << "Single test:" << name << std::endl;
        auto test1 = utest::make_test(name);
#ifdef GRAPHIC
        std::cout<<"running without the test suite" << std::endl;
        test_behavior(test1, args[0], args[1], args[2], args[3], args[4], ref, std::shared_ptr<y::Emitter>());
#else
        UTEST_REGISTER(test_suite, test1, test_behavior(test1, args[0], args[1], args[2], args[3], args[4], ref, std::shared_ptr<y::Emitter>()));
        test_suite.run(1, true);
        utest::write_report(test_suite, std::cout, true);
#endif
        return 0;
    } else {
#ifdef GRAPHIC
    IWBC_ERROR("GRAPHICS is possible only in single mode (-s ...)");
#endif
    }

    ///// the default behavior is to run all the combinations in different threads

    // this is relative to the "tests" directory
    std::string controller = "../../etc/franka/pos_tracker.yaml";
    auto behaviors = {"../../etc/franka/cartesian_line.yaml"};
    auto collision = {"fcl"};
    auto actuators = {"servo", "velocity", "spd", "torque"};
    std::string urdf = "franka/franka.urdf";

    (*yout) << y::Key << controller << y::Value << y::BeginMap;
    for (auto& b : behaviors) {
        (*yout) << y::Key << b << y::Value << y::BeginMap;
        for (auto& a : actuators) {
            (*yout) << y::Key << a << y::Value << y::BeginMap;
            for (auto& c : collision) {
                (*yout) << y::Key << c << y::Value << y::BeginMap;
                std::string name = controller + " "
                    + b + " "
                    + a + " "
                    + c;
                auto test1 = utest::make_test(name + " " + urdf);

                if (vm.count("generate_ref")) {
                    std::cout << "generating..." << std::endl;
                    test_behavior(test1, controller, b, a, c, urdf, ref, yout);
                }
                else {
                    UTEST_REGISTER(test_suite, test1, test_behavior(test1, controller, b, a, c, urdf, ref, std::shared_ptr<y::Emitter>()));
                }
                (*yout) << y::EndMap;
            }
            (*yout) << y::EndMap;
        }
    }
    (*yout) << y::EndMap;
    (*yout) << y::EndMap;

    auto fname = std::string("test_results-franka-") + date() + ".yaml";
    std::ofstream ofs(fname.c_str());
    ofs << yout->c_str();

    if (!vm.count("generate_ref")) {
        test_suite.run(n_threads);
        utest::write_report(test_suite, std::cout, true);
        std::cout << "------------ SUMMARY ------------" << std::endl;
        utest::write_report(test_suite, std::cout, false);
        if(test_suite.success())
            return 0;
        else 
            return 1;
    }
    return 0;
}
