#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <thread>
#include <string>
#include <map>
#include <yaml-cpp/yaml.h>

#include <robotiq_2f_gripper_msgs/action/move_two_finger_gripper.hpp>
#include <robotiq_2f_gripper_interfaces/default_driver.hpp>
#include <robotiq_2f_gripper_interfaces/default_serial.hpp>

namespace robotiq_2f_gripper_hardware
{

    using SetPosition = robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper;
    using robotiq_2f_gripper_interfaces::DefaultDriver;
    using robotiq_2f_gripper_interfaces::DefaultSerial;

    class GripperNode : public rclcpp::Node
    {
    public:
        GripperNode();
        ~GripperNode();

    private:
        std::string serial_port_;
        int baudrate_;
        double timeout_;
        int action_timeout_;
        int slave_address_;
        bool fake_hardware_;
        std::unique_ptr<DefaultDriver> driver_;

        // Topic names
        std::string gripper_action_topic = "robotiq_2f_gripper_action";
        std::string joint_state_topic = "robotiq_2f_gripper/joint_states";
        std::string object_grasped_topic = "robotiq_2f_gripper/object_grasped";
        std::string finger_distance_mm_topic = "robotiq_2f_gripper/finger_distance_mm";
        std::string confidence_command_topic = "robotiq_2f_gripper/confidence_command";
        std::string binary_command_topic = "robotiq_2f_gripper/binary_command";

        // Constants (loaded from config)
        double OPEN_THRESHOLD;  // threshold for opening the gripper during confidence-based control
        double CLOSE_THRESHOLD; // threshold for closing the gripper during confidence-based control

        // Constants
        const double MAX_GRIPPER_POSITION_METER = 0.142; // distance between fingers in meters
        const int FULLY_CLOSED_THRESHOLD = 226;          // gripper position value (from hexadecimal gripper system interpreted as int) when first fully closed (226 to 255 is fully closed)
        const double MAX_GRIPPER_POSITION_RAD = 0.7;     // upper limit of the gripper in radians (0.7 is fully closed)

        // State variables
        double gripper_position_;
        double gripper_speed_;
        double gripper_force_;

        // Variables for confidence-based control with hysteresis
        double previous_confidence_{0.0};
        bool initial_command_{true};

        rclcpp_action::Server<SetPosition>::SharedPtr action_server_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr object_grasped_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr finger_distance_mm_publisher_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gripper_confidence_command_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gripper_binary_command_subscriber_;

        std::atomic<bool> running_{false};
        sensor_msgs::msg::JointState joint_state_;
        rclcpp::TimerBase::SharedPtr timer_1_, timer_2_;

        rclcpp_action::GoalResponse handle_move_goal(
            const rclcpp_action::GoalUUID & /*uuid*/,
            std::shared_ptr<const SetPosition::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel_move(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetPosition>> /*goal_handle*/);

        void loadConfig(const std::string &config_file);
        void handle_move_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetPosition>> goal_handle);
        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetPosition>> goal_handle);
        void update_joint_state_callback();
        void update_object_grasped_callback();
        void gripper_command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void gripper_binary_command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

        uint8_t decimalToHex(int value);
        int convertToGripperSystemPosition(double position);
        double convertToMeters(int value);
        int convertToGripperSystem(double value);
    };

} // namespace robotiq_2f_gripper_hardware