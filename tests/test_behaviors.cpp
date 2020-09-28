#define BOOST_TEST_MODULE clone test
#include <boost/test/unit_test.hpp>

#include <inria_wbc/behaviors/factory.hpp>
#include <iostream>
#include <robot_dart/robot.hpp>
#include <vector>

#include "test_behavior.hpp"

BOOST_AUTO_TEST_CASE(running)
{
    // we simply check that things are running without error / segv
    srand(time(0));
    auto behaviors = {"../etc/squat.yaml", "../etc/arm.yaml"};
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    auto robot = std::make_shared<robot_dart::Robot>("talos/talos.urdf", packages);

    std::cout << "robot:" << robot->model_filename() << std::endl;

    for (auto &sot_config_path : behaviors)
    {
        std::cout << "configuration:" << sot_config_path << std::endl;

        inria_wbc::controllers::TalosBaseController::Params params = {robot->model_filename(), "../etc/talos_configurations.srdf", sot_config_path, "",
                                                                      0.001, false, robot->mimic_dof_names()};

        std::string behavior_name;
        auto config = YAML::LoadFile(sot_config_path);
        inria_wbc::utils::parse(behavior_name, "name", config, false, "BEHAVIOR");
        auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, params);
        auto cmds = test_behavior(behavior);
    }
}


BOOST_AUTO_TEST_CASE(set_opt_params)
{
    // we check that the beahvior changes when we use different params
    srand(time(0));
    auto behaviors = {"../etc/squat.yaml", "../etc/arm.yaml"};
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    auto robot = std::make_shared<robot_dart::Robot>("talos/talos.urdf", packages);

    std::cout << "robot:" << robot->model_filename() << std::endl;

    for (auto &sot_config_path : behaviors)
    {
        std::cout << "configuration:" << sot_config_path << std::endl;

        inria_wbc::controllers::TalosBaseController::opt_params_t opt_p;
        opt_p["w_lh"] = 0.0;
        inria_wbc::controllers::TalosBaseController::Params params = {robot->model_filename(), "../etc/talos_configurations.srdf", sot_config_path, "",
                                                                      0.001, false, robot->mimic_dof_names(),
                                                                      opt_p };

        std::string behavior_name;
        auto config = YAML::LoadFile(sot_config_path);
        inria_wbc::utils::parse(behavior_name, "name", config, false, "BEHAVIOR");

        for (size_t i = 0; i < 10; ++i)
        {
            auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, params);
            auto cmds = test_behavior(behavior);
        }
    }
}