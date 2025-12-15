#include <chrono>
#include <thread>
#include <iostream>
#include <memory>
#include <cmath>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/bool.hpp>

#include <robotiq_2f_gripper_hardware/robotiq_2f_gripper_node.hpp>

using namespace robotiq_2f_gripper_hardware;
using namespace std::placeholders;
using robotiq_2f_gripper_interfaces::DefaultDriver;
using robotiq_2f_gripper_interfaces::DefaultSerial;

GripperNode::GripperNode() : Node("robotiq_2f_gripper_node")
{
    // Declare parameter for config file path
    declare_parameter<std::string>("config_file", "");
    std::string config_file = get_parameter("config_file").as_string();

    if (config_file.empty())
    {
        // Use package share directory to get the default config file
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("robotiq_2f_gripper_hardware");
        config_file = pkg_share_dir + "/config/config.yml";
        RCLCPP_INFO(get_logger(), "No config file specified, using default: %s", config_file.c_str());
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Using config file: %s", config_file.c_str());
    }

    // Load configuration from file
    loadConfig(config_file);

    // Still declare parameters for backward compatibility and to allow override
    declare_parameter<std::string>("serial_port", serial_port_);
    serial_port_ = get_parameter("serial_port").as_string();
    RCLCPP_INFO(get_logger(), "Using serial port: %s", serial_port_.c_str());

    declare_parameter<int>("baudrate", baudrate_);
    baudrate_ = get_parameter("baudrate").as_int();
    RCLCPP_INFO(get_logger(), "Using baudrate: %d", baudrate_);

    declare_parameter<double>("timeout", timeout_);
    timeout_ = get_parameter("timeout").as_double();
    RCLCPP_INFO(get_logger(), "Using timeout: %f", timeout_);

    declare_parameter<int>("action_timeout", action_timeout_);
    action_timeout_ = get_parameter("action_timeout").as_int();
    RCLCPP_INFO(get_logger(), "Using action timeout: %d", action_timeout_);

    declare_parameter<int>("slave_address", slave_address_);
    slave_address_ = get_parameter("slave_address").as_int();
    RCLCPP_INFO(get_logger(), "Using slave address: %d", slave_address_);

    declare_parameter<bool>("fake_hardware", fake_hardware_);
    fake_hardware_ = get_parameter("fake_hardware").as_bool();
    RCLCPP_INFO(get_logger(), "Using fake hardware: %s", fake_hardware_ ? "true" : "false");

    if (!fake_hardware_)
    {
        auto serial = std::make_unique<DefaultSerial>();
        serial->set_port(serial_port_);
        serial->set_baudrate(baudrate_);
        serial->set_timeout(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(timeout_)));

        driver_ = std::make_unique<DefaultDriver>(std::move(serial));
        driver_->set_slave_address(slave_address_);

        const bool connected = driver_->connect();
        if (!connected)
        {
            RCLCPP_ERROR(get_logger(), "The gripper is not connected");
            return;
        }
        RCLCPP_INFO(get_logger(), "The gripper is connected.");

        if (!(driver_->is_gripper_active()))
        {
            driver_->deactivate();
            driver_->activate();
        }
        RCLCPP_INFO(get_logger(), "The gripper is activated.");
    }

    action_server_ = rclcpp_action::create_server<SetPosition>(
        this, gripper_action_topic,
        std::bind(&GripperNode::handle_move_goal, this, _1, _2),
        std::bind(&GripperNode::handle_cancel_move, this, _1),
        std::bind(&GripperNode::handle_move_accepted, this, _1));

    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 1);
    finger_distance_mm_publisher_ = create_publisher<std_msgs::msg::Float32>(finger_distance_mm_topic, 1);
    timer_1_ = create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&GripperNode::update_joint_state_callback, this));

    object_grasped_publisher_ = create_publisher<std_msgs::msg::Bool>(object_grasped_topic, 1);
    timer_2_ = create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&GripperNode::update_object_grasped_callback, this));

    // Create subscriber for confidence-based gripper control with hysteresis
    gripper_confidence_command_subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        confidence_command_topic, 10,
        std::bind(&GripperNode::gripper_command_callback, this, _1));
    RCLCPP_INFO(get_logger(), "Gripper confidence command subscriber created on topic: %s", confidence_command_topic.c_str());

    // Create subscriber for direct gripper control (binary open/close)
    gripper_binary_command_subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        binary_command_topic, 10,
        std::bind(&GripperNode::gripper_binary_command_callback, this, _1));
    RCLCPP_INFO(get_logger(), "Gripper binary command subscriber created on topic: %s", binary_command_topic.c_str());

    RCLCPP_INFO(get_logger(), "Gripper node initialized");
}

GripperNode::~GripperNode()
{
    if (!fake_hardware_)
    {
        driver_->deactivate();
        driver_->disconnect();
    }
}

rclcpp_action::GoalResponse GripperNode::handle_move_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const SetPosition::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received goal request with %f meters", goal->target_position);

    if (running_)
    {
        RCLCPP_INFO(get_logger(), "Preempting previous gripper goal with new request");
        running_ = false;
    }

    if (!fake_hardware_)
    {
        if (!driver_->is_gripper_active())
        {
            RCLCPP_ERROR(get_logger(), "Gripper is not activated");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    // Check if the goal is valid
    if (goal->target_position < 0 || goal->target_position > 0.142)
    {
        RCLCPP_ERROR(get_logger(), "Invalid goal request, target position (in meters [m]) must be between 0 (fully closed) and 0.14 (fully open)");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->target_speed < 0 || goal->target_speed > 1)
    {
        RCLCPP_ERROR(get_logger(), "Invalid goal request, target speed must be between 0 and 1");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->target_force < 0 || goal->target_force > 1)
    {
        RCLCPP_ERROR(get_logger(), "Invalid goal request, target force must be between 0 and 1");
        return rclcpp_action::GoalResponse::REJECT;
    }

    running_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperNode::handle_cancel_move(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetPosition>> /*goal_handle*/)
{
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    running_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperNode::handle_move_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetPosition>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Accepted goal");
    std::thread{std::bind(&GripperNode::execute, this, _1), goal_handle}.detach();
}

void GripperNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetPosition>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SetPosition::Feedback>();
    auto result = std::make_shared<SetPosition::Result>();

    try
    {
        if (fake_hardware_)
        {
            gripper_position_ = goal->target_position;
            gripper_speed_ = goal->target_speed;
            gripper_force_ = goal->target_force;
            RCLCPP_INFO(get_logger(), "[Fake Hardware] Gripper position command accepted.");
        }
        else
        {
            driver_->set_force(convertToGripperSystem(goal->target_force));
            driver_->set_speed(convertToGripperSystem(goal->target_speed));
            driver_->set_gripper_position(convertToGripperSystemPosition(goal->target_position));
            RCLCPP_INFO(get_logger(), "Gripper command sent (non-blocking).");
        }

        result->success = true;
        goal_handle->succeed(result);
    }
    catch (const std::runtime_error &e)
    {
        RCLCPP_ERROR(get_logger(), "Error executing goal: %s", e.what());
        result->success = false;
        goal_handle->abort(result);
    }

    running_ = false;
}

void GripperNode::update_joint_state_callback()
{
    if (running_)
    {
        return;
    }

    double curr_gripper_position;
    double curr_gripper_position_rad;
    double finger_distance_mm;
    if (!fake_hardware_)
    {
        curr_gripper_position = static_cast<int>(driver_->get_gripper_position());

        // Calculate the finger distance in millimeters using the existing function
        finger_distance_mm = convertToMeters(curr_gripper_position) * 1000; // Convert from meters to mm

        if (curr_gripper_position > FULLY_CLOSED_THRESHOLD)
        {
            curr_gripper_position_rad = MAX_GRIPPER_POSITION_RAD;
        }
        else
        {
            curr_gripper_position_rad = curr_gripper_position / FULLY_CLOSED_THRESHOLD * MAX_GRIPPER_POSITION_RAD;
        }
    }
    else
    {
        curr_gripper_position_rad = ((-1 * gripper_position_) + MAX_GRIPPER_POSITION_METER) / MAX_GRIPPER_POSITION_METER * MAX_GRIPPER_POSITION_RAD;
        finger_distance_mm = gripper_position_ * 1000; // Convert from meters to mm
    }

    // Publish joint state
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = now();
    message.name = {"finger_joint"};
    message.position = {curr_gripper_position_rad};
    joint_state_publisher_->publish(message);

    // Publish finger distance in millimeters
    auto distance_msg = std_msgs::msg::Float32();
    distance_msg.data = finger_distance_mm;
    finger_distance_mm_publisher_->publish(distance_msg);
}

void GripperNode::update_object_grasped_callback()
{
    if (running_)
    {
        return;
    }

    auto message = std_msgs::msg::Bool();
    if (!fake_hardware_)
    {
        message.data = {static_cast<bool>(driver_->is_object_grasped())};
    }
    else
    {
        message.data = {false};
    }
    object_grasped_publisher_->publish(message);
}

uint8_t GripperNode::decimalToHex(int value)
{
    return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

int GripperNode::convertToGripperSystemPosition(double position)
{
    if (position >= 0.015)
    {
        double a = -1399.78;
        double b = -1328.11;
        double c = 218.384;
        return decimalToHex(static_cast<int>(a * pow(position, 2) + b * position + c));
    }
    else if (position >= 0.001)
    {
        double a = 18315;
        double b = -1849.82;
        double c = 223.626;
        return decimalToHex(static_cast<int>(a * pow(position, 2) + b * position + c));
    }
    else
    {
        return decimalToHex(255);
    }
}

// inverse of convertToGripperSystemPosition
double GripperNode::convertToMeters(int value)
{
    if (value <= 200)
    {
        double a = -3.84615e-07;
        double b = -5.67622e-04;
        double c = 0.142692;
        return a * pow(value, 2) + b * value + c;
    }
    else if (value <= 226)
    {
        double a = 8.92857e-06;
        double b = -4.38036e-03;
        double c = 0.533911;
        return a * pow(value, 2) + b * value + c;
    }
    else
    {
        return 0.0;
    }
}

int GripperNode::convertToGripperSystem(double value)
{
    return decimalToHex(static_cast<int>(value * 255));
}

void GripperNode::gripper_command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (running_)
    {
        RCLCPP_WARN(get_logger(), "Ignoring command, gripper is already running an action");
        return;
    }

    if (msg->data.size() < 1)
    {
        RCLCPP_ERROR(get_logger(), "Received empty command array. Expected a confidence value.");
        return;
    }

    // Get the confidence value from the message
    double confidence = msg->data[0];

    // Validate input range
    if (confidence < -1.0 || confidence > 1.0)
    {
        RCLCPP_ERROR(get_logger(), "Invalid confidence value: %f. Must be between -1.0 and 1.0", confidence);
        return;
    }

    // Using class member variables for hysteresis thresholds and state tracking

    // Default speed and force for the gripper
    double speed = 0.5; // Default speed 0.5
    double force = 0.5; // Default force 0.5

    // Calculate gripper position based on confidence with hysteresis
    double position;

    // If it's the first command or if confidence crosses thresholds
    if (initial_command_ ||
        (confidence > OPEN_THRESHOLD && previous_confidence_ <= OPEN_THRESHOLD) ||
        (confidence < CLOSE_THRESHOLD && previous_confidence_ >= CLOSE_THRESHOLD))
    {
        // Determine position based on confidence
        if (confidence > OPEN_THRESHOLD)
        {
            position = MAX_GRIPPER_POSITION_METER; // Fully open (0.142m)
            RCLCPP_INFO(get_logger(), "Confidence value %f exceeds open threshold, opening gripper", confidence);
        }
        else if (confidence < CLOSE_THRESHOLD)
        {
            position = 0.0; // Fully closed
            RCLCPP_INFO(get_logger(), "Confidence value %f below close threshold, closing gripper", confidence);
        }
        else
        {
            // In the deadband - keep previous state if not the first command
            if (initial_command_)
            {
                position = MAX_GRIPPER_POSITION_METER / 2.0; // Middle position for first command
                RCLCPP_INFO(get_logger(), "Initial confidence in deadband %f, setting to middle position", confidence);
            }
            else
            {
                // No change due to hysteresis
                previous_confidence_ = confidence;
                RCLCPP_INFO(get_logger(), "Confidence value %f in hysteresis zone, maintaining current state", confidence);
                return;
            }
        }

        RCLCPP_INFO(get_logger(), "Setting gripper position to %f based on confidence %f", position, confidence);

        // Remember for hysteresis
        previous_confidence_ = confidence;
        initial_command_ = false;

        running_ = true;

        if (fake_hardware_)
        {
            gripper_position_ = position;
            gripper_speed_ = speed;
            gripper_force_ = force;
            RCLCPP_INFO(get_logger(), "[Fake Hardware] Gripper position set to %f", position);
            running_ = false;
            return;
        }

        try
        {
            driver_->set_force(convertToGripperSystem(force));
            driver_->set_speed(convertToGripperSystem(speed));
            driver_->set_gripper_position(convertToGripperSystemPosition(position));

            auto start_time = std::chrono::steady_clock::now();
            auto timeout = std::chrono::seconds(action_timeout_);
            while (std::chrono::steady_clock::now() - start_time < timeout)
            {
                if (!driver_->gripper_is_moving())
                {
                    RCLCPP_INFO(get_logger(), "Gripper movement completed.");
                    running_ = false;
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            RCLCPP_INFO(get_logger(), "Gripper movement timed out.");
        }
        catch (const std::runtime_error &e)
        {
            RCLCPP_ERROR(get_logger(), "Error executing command: %s", e.what());
        }

        running_ = false;
    }
    else
    {
        // Update previous confidence but don't send new command (within hysteresis band)
        previous_confidence_ = confidence;
        RCLCPP_DEBUG(get_logger(), "Confidence value %f within hysteresis zone, no action taken", confidence);
    }
}

void GripperNode::gripper_binary_command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (running_)
    {
        RCLCPP_WARN(get_logger(), "Ignoring binary command, gripper is already running an action");
        return;
    }

    if (msg->data.size() < 1)
    {
        RCLCPP_ERROR(get_logger(), "Received empty binary command array. Expected a value.");
        return;
    }

    // Get the command value from the message
    double value = msg->data[0];

    // Validate input value (should be 1.0 for open or -1.0 for close)
    if (std::abs(std::abs(value) - 1.0) > 0.01)
    {
        RCLCPP_ERROR(get_logger(), "Invalid binary command value: %f. Must be 1.0 (open) or -1.0 (close)", value);
        return;
    }

    // Default speed and force for the gripper
    double speed = 0.5; // Default speed 0.5
    double force = 0.5; // Default force 0.5

    // Calculate gripper position based on binary command
    double position;

    if (value > 0.0) // Positive value (1.0) means open
    {
        position = MAX_GRIPPER_POSITION_METER; // Fully open (0.142m)
        RCLCPP_INFO(get_logger(), "Binary open command received, opening gripper");
    }
    else // Negative value (-1.0) means close
    {
        position = 0.0; // Fully closed
        RCLCPP_INFO(get_logger(), "Binary close command received, closing gripper");
    }

    RCLCPP_INFO(get_logger(), "Setting gripper position to %f based on binary command %f", position, value);

    running_ = true;

    if (fake_hardware_)
    {
        gripper_position_ = position;
        gripper_speed_ = speed;
        gripper_force_ = force;
        RCLCPP_INFO(get_logger(), "[Fake Hardware] Gripper position set to %f", position);
        running_ = false;
        return;
    }

    try
    {
        driver_->set_force(convertToGripperSystem(force));
        driver_->set_speed(convertToGripperSystem(speed));
        driver_->set_gripper_position(convertToGripperSystemPosition(position));

        auto start_time = std::chrono::steady_clock::now();
        auto timeout = std::chrono::seconds(action_timeout_);
        while (std::chrono::steady_clock::now() - start_time < timeout)
        {
            if (!driver_->gripper_is_moving())
            {
                RCLCPP_INFO(get_logger(), "Gripper movement completed.");
                running_ = false;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_INFO(get_logger(), "Gripper movement timed out.");
    }
    catch (const std::runtime_error &e)
    {
        RCLCPP_ERROR(get_logger(), "Error executing binary command: %s", e.what());
    }

    running_ = false;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}