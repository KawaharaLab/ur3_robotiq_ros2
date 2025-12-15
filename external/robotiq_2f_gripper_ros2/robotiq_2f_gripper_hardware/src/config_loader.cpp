#include <robotiq_2f_gripper_hardware/robotiq_2f_gripper_node.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace robotiq_2f_gripper_hardware
{

    void GripperNode::loadConfig(const std::string &config_file)
    {
        try
        {
            YAML::Node config = YAML::LoadFile(config_file);

            OPEN_THRESHOLD = config["open_threshold"].as<double>(0.2);
            CLOSE_THRESHOLD = config["close_threshold"].as<double>(-0.2);

            RCLCPP_INFO(get_logger(), "Constants loaded from config file: %s", config_file.c_str());

            // Loaded parameters
            RCLCPP_INFO(get_logger(), "OPEN_THRESHOLD: %f, CLOSE_THRESHOLD: %f", OPEN_THRESHOLD, CLOSE_THRESHOLD);
        }

        catch (const YAML::ParserException &e)
        {
            RCLCPP_ERROR(get_logger(), "Error parsing YAML file: %s - %s", config_file.c_str(), e.what());
            // Use defaults in case of error
            OPEN_THRESHOLD = 0.2;
            CLOSE_THRESHOLD = -0.2;
        }
    }
} // namespace robotiq_2f_gripper_hardware
