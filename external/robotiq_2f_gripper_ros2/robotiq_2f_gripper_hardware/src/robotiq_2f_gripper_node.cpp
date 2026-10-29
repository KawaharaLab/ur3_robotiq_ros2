#include <chrono>
#include <cstdint>
#include <thread>
#include <iostream>
#include <memory>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <sstream>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/bool.hpp>

#include <robotiq_2f_gripper_hardware/robotiq_2f_gripper_node.hpp>

using namespace robotiq_2f_gripper_hardware;
using namespace std::placeholders;
using robotiq_2f_gripper_interfaces::DefaultDriver;
using robotiq_2f_gripper_interfaces::DefaultSerial;

namespace
{
std::string make_initialization_record(
    int64_t start_ros, int64_t start_steady, int64_t ready_ros,
    const std::string &port, int baud, int slave, bool fake,
    bool active_at_start, bool activated, bool deactivate_on_shutdown)
{
    std::ostringstream out;
    out << std::boolalpha
        << "{\"schema_version\":2,\"startup_id\":"
        << std::quoted(std::to_string(start_ros) + "-" + std::to_string(start_steady))
        << ",\"node_start_ros_ns\":" << start_ros
        << ",\"node_start_monotonic_ns\":" << start_steady
        << ",\"ready_ros_ns\":" << ready_ros
        << ",\"port\":" << std::quoted(port)
        << ",\"baud\":" << baud << ",\"slave\":" << slave
        << ",\"fake_hardware\":" << fake
        << ",\"active_at_start\":" << active_at_start
        << ",\"activation_performed\":" << activated
        << ",\"deactivate_on_shutdown\":" << deactivate_on_shutdown
        << ",\"cancel_stops_motion\":true"
        << ",\"command_event_topic\":\"/robotiq_2f_gripper/command_event\""
        << ",\"conversion_model\":\"legacy_piecewise_polynomial_v1_uncalibrated\"}";
    return out.str();
}
std::string make_command_record(const std::string &startup_id, uint64_t sequence,
    const std::string &stage, const std::string &source, int64_t ros_ns, int64_t steady_ns,
    int position, int speed, int force, double width_m)
{
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\"schema_version\":1,\"startup_id\":" << std::quoted(startup_id)
        << ",\"command_seq\":" << sequence << ",\"stage\":" << std::quoted(stage)
        << ",\"source\":" << std::quoted(source) << ",\"ros_ns\":" << ros_ns
        << ",\"monotonic_ns\":" << steady_ns << ",\"position_count\":" << position
        << ",\"speed_count\":" << speed << ",\"force_count\":" << force
        << ",\"requested_width_m\":";
    if (std::isfinite(width_m)) out << width_m; else out << "null";
    out << "}";
    return out.str();
}

}

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

    auto shutdown_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    shutdown_descriptor.description = "Keep activation only for a stationary exclusive-port probe handoff when false";
    shutdown_descriptor.read_only = true;
    declare_parameter<bool>("deactivate_on_shutdown", true, shutdown_descriptor);
    deactivate_on_shutdown_ = get_parameter("deactivate_on_shutdown").as_bool();

    auto poll_rate_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    poll_rate_descriptor.description = "Startup-only status polling rate in Hz; valid range is (0, 100]";
    poll_rate_descriptor.read_only = true;
    declare_parameter<double>("status_poll_rate_hz", 20.0, poll_rate_descriptor);
    status_poll_rate_hz_ = get_parameter("status_poll_rate_hz").as_double();
    if (!std::isfinite(status_poll_rate_hz_) || status_poll_rate_hz_ <= 0.0 || status_poll_rate_hz_ > 100.0)
    {
        throw std::invalid_argument("status_poll_rate_hz must be finite and in the range (0, 100]");
    }
    RCLCPP_INFO(get_logger(), "Using status polling rate: %.3f Hz", status_poll_rate_hz_);

    const auto startup_ros_ns = now().nanoseconds();
    const auto startup_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    startup_id_ = std::to_string(startup_ros_ns) + "-" + std::to_string(startup_steady_ns);
    bool activation_performed = false;
    bool active_at_start = false;
    initialization_publisher_ = create_publisher<std_msgs::msg::String>(
        "/robotiq_2f_gripper/initialization", rclcpp::QoS(1).reliable().transient_local());
    command_event_publisher_ = create_publisher<std_msgs::msg::String>(
        "/robotiq_2f_gripper/command_event", rclcpp::QoS(100).reliable());
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

        active_at_start = driver_->is_gripper_active();
        if (!active_at_start)
        {
            driver_->deactivate();
            driver_->activate();
            activation_performed = true;
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
    status_publisher_ = create_publisher<robotiq_2f_gripper_msgs::msg::GripperStatus>(status_topic, 10);
    const auto status_poll_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / status_poll_rate_hz_));
    timer_1_ = create_wall_timer(
        status_poll_period, std::bind(&GripperNode::update_joint_state_callback, this));

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

    force_press_control_service_ = create_service<std_srvs::srv::SetBool>(
        "~/set_force_press_control",
        std::bind(&GripperNode::set_force_press_control, this, _1, _2));
    auto initialization = std_msgs::msg::String();
    initialization.data = make_initialization_record(
        startup_ros_ns, startup_steady_ns, now().nanoseconds(), serial_port_, baudrate_,
        slave_address_, fake_hardware_, active_at_start, activation_performed,
        deactivate_on_shutdown_);
    initialization_publisher_->publish(initialization);
    RCLCPP_INFO(get_logger(), "Gripper node initialized");
}

void GripperNode::send_recorded_position(
    uint8_t raw_position, double width_m, double speed, double force, const std::string &source)
{
    const auto sequence = ++command_sequence_;
    auto record = [&](const char *stage) {
        try {
            auto message = std_msgs::msg::String();
            const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            message.data = make_command_record(startup_id_, sequence, stage, source,
                now().nanoseconds(), steady_ns, raw_position,
                convertToGripperSystem(speed), convertToGripperSystem(force), width_m);
            command_event_publisher_->publish(message);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Command event publication failed: %s", e.what());
        }
    };
    record("write_start");
    try {
        driver_->set_gripper_position(raw_position);
    } catch (...) {
        record("write_exception");
        throw;
    }
    record("write_return"); // returned from Modbus write, not physical completion
}

GripperNode::~GripperNode()
{
    if (!fake_hardware_ && !serial_leased_)
    {
        if (deactivate_on_shutdown_) {
            driver_->deactivate();
        } else {
            RCLCPP_WARN(get_logger(), "Leaving gripper activated for handoff; no motion must be in progress.");
        }
        driver_->disconnect();
    }
}

void GripperNode::set_force_press_control(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    // Services, timers, goal acceptance and direct-command callbacks use the
    // same mutually-exclusive default callback group. A detached action worker
    // is excluded by running_, retained until its final serial operation ends.
    bool acquired_by_request = false;
    auto reply = [&](bool success, const std::string &reason) {
        std::ostringstream out;
        out << "{\"reason\":" << std::quoted(reason)
            << ",\"leased\":" << (serial_leased_ ? "true" : "false")
            << ",\"request_acquired\":" << (acquired_by_request ? "true" : "false")
            << ",\"port\":" << std::quoted(serial_port_)
            << ",\"baud\":" << baudrate_ << ",\"slave\":" << slave_address_ << "}";
        response->success = success;
        response->message = out.str();
    };
    if (fake_hardware_ || !driver_) { reply(false, "Real connected gripper required"); return; }
    if (running_) { reply(false, "Action or direct command still running"); return; }
    if (request->data)
    {
        if (serial_leased_) { reply(false, "Serial already leased; no second owner allowed"); return; }
        try
        {
            if (!driver_->is_gripper_active()) { reply(false, "Gripper not active"); return; }
            const auto status = driver_->read_status();
            if (status.g_flt != 0 || driver_->motion_command_in_progress())
            {
                reply(false, "Gripper must be healthy and stationary before handoff");
                return;
            }
            serial_leased_ = true;
            acquired_by_request = true;
            driver_->disconnect(); // Never deactivate, activate, move or release fingers here.
            reply(true, "Serial released for force collection");
        }
        catch (const std::exception &e) { reply(false, e.what()); }
        return;
    }
    if (!serial_leased_) { reply(true, "Normal control already active"); return; }
    try
    {
        // The collection owner must have exited and closed its TIOCEXCL port.
        // Reconnect and read only; never run an activation/reset sequence.
        if (!driver_->connect()) throw std::runtime_error("Could not reopen serial port");
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        auto status = driver_->read_status();
        while (status.g_flt == 9 && std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            status = driver_->read_status();
        }
        if (status.g_flt != 0 || !driver_->is_gripper_active())
            throw std::runtime_error("Gripper did not return healthy and active; no automatic reset performed");
        status = driver_->get_last_status();
        if (status.g_flt != 0 || driver_->motion_command_in_progress())
            throw std::runtime_error("Gripper is faulted or still moving; normal commands remain blocked");
        gripper_position_ = convertToMeters(status.g_po);
        serial_leased_ = false;
        reply(true, "Normal serial control restored without activation or motion");
    }
    catch (const std::exception &e)
    {
        try { driver_->disconnect(); } catch (...) {}
        // Keep ordinary commands/timers disabled after an uncertain return.
        serial_leased_ = true;
        reply(false, e.what());
    }
}

rclcpp_action::GoalResponse GripperNode::handle_move_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const SetPosition::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received goal request with %f meters", goal->target_position);

    if (running_ || serial_leased_)
    {
        RCLCPP_WARN(get_logger(), "Rejecting goal: gripper busy or serial assigned to force collection");
        return rclcpp_action::GoalResponse::REJECT;
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
    // The worker owns the serial transaction until it observes cancellation.
    // Keep timers, new goals and handoff blocked until the worker clears running_.
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
            const auto requested_position = static_cast<uint8_t>(
                convertToGripperSystemPosition(goal->target_position));
            send_recorded_position(requested_position, goal->target_position,
                                   goal->target_speed, goal->target_force, "action");
            RCLCPP_INFO(get_logger(), "Gripper command sent; waiting for physical completion.");

            const auto start_time = std::chrono::steady_clock::now();
            const auto timeout = std::chrono::seconds(action_timeout_);
            const auto poll_period = std::chrono::duration<double>(1.0 / status_poll_rate_hz_);
            while (rclcpp::ok() && std::chrono::steady_clock::now() - start_time < timeout)
            {
                if (goal_handle->is_canceling())
                {
                    driver_->stop_motion();
                    running_ = false;
                    result->success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(get_logger(), "Gripper action canceled.");
                    return;
                }

                const auto read_start = std::chrono::steady_clock::now();
                const auto status = driver_->read_status();
                const auto read_end = std::chrono::steady_clock::now();
                const auto acquisition_stamp = now();
                publish_hardware_status(
                    status, acquisition_stamp,
                    std::chrono::duration<float, std::milli>(read_end - read_start).count());

                if (status.g_flt != 0)
                {
                    driver_->stop_motion();
                    running_ = false;
                    result->success = false;
                    goal_handle->abort(result);
                    RCLCPP_ERROR(get_logger(), "Gripper fault while moving: gFLT=%u", status.g_flt);
                    return;
                }

                // gOBJ=0 means motion is still in progress. Values 1/2 are
                // object-detected stops. For gOBJ=3, also require the measured
                // position to reach the target; this prevents a stale gOBJ=3
                // response immediately after updating gPR from ending the
                // action before physical motion has completed.
                const bool object_detected = status.g_obj == 1 || status.g_obj == 2;
                const bool reached_position =
                    status.g_obj == 3 &&
                    std::abs(static_cast<int>(status.g_po) - static_cast<int>(requested_position)) <= 1;
                if (status.g_pr == requested_position && (object_detected || reached_position))
                {
                    running_ = false;
                    result->success = true;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(
                        get_logger(),
                        "Gripper physical completion: gOBJ=%u gPR=%u gPO=%u",
                        status.g_obj, status.g_pr, status.g_po);
                    return;
                }
                std::this_thread::sleep_for(poll_period);
            }

            driver_->stop_motion();
            running_ = false;
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(get_logger(), "Gripper action timed out before physical completion.");
            return;
        }

        running_ = false;
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
    if (running_ || serial_leased_)
    {
        return;
    }

    if (!fake_hardware_)
    {
        // All fields and the derived width below come from this one Modbus read.
        const auto read_start = std::chrono::steady_clock::now();
        const auto status = driver_->read_status();
        const auto read_end = std::chrono::steady_clock::now();
        const auto acquisition_stamp = now();
        publish_hardware_status(
            status, acquisition_stamp,
            std::chrono::duration<float, std::milli>(read_end - read_start).count());
        return;
    }

    const double curr_gripper_position_rad =
        ((-1 * gripper_position_) + MAX_GRIPPER_POSITION_METER) /
        MAX_GRIPPER_POSITION_METER * MAX_GRIPPER_POSITION_RAD;
    const double finger_distance_mm = gripper_position_ * 1000;

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

void GripperNode::publish_hardware_status(
    const DefaultDriver::StatusRegisters &status,
    const rclcpp::Time &acquisition_stamp,
    float read_duration_ms)
{
    auto status_msg = robotiq_2f_gripper_msgs::msg::GripperStatus();
    status_msg.header.stamp = acquisition_stamp;
    status_msg.g_obj = status.g_obj;
    status_msg.g_flt = status.g_flt;
    status_msg.g_pr = status.g_pr;
    status_msg.g_po = status.g_po;
    status_msg.g_cu = status.g_cu;
    status_msg.read_duration_ms = read_duration_ms;
    status_publisher_->publish(status_msg);

    const double raw_position = static_cast<double>(status.g_po);
    const double finger_distance_mm = convertToMeters(status.g_po) * 1000.0;
    const double position_rad = raw_position > FULLY_CLOSED_THRESHOLD
        ? MAX_GRIPPER_POSITION_RAD
        : raw_position / FULLY_CLOSED_THRESHOLD * MAX_GRIPPER_POSITION_RAD;

    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.header.stamp = acquisition_stamp;
    joint_state.name = {"finger_joint"};
    joint_state.position = {position_rad};
    joint_state_publisher_->publish(joint_state);

    auto distance = std_msgs::msg::Float32();
    distance.data = static_cast<float>(finger_distance_mm);
    finger_distance_mm_publisher_->publish(distance);
}

void GripperNode::update_object_grasped_callback()
{
    if (running_ || serial_leased_)
    {
        return;
    }

    auto message = std_msgs::msg::Bool();
    if (!fake_hardware_)
    {
        // Reuse the last complete status snapshot; do not issue a second read
        // whose register values would have no corresponding stamped message.
        const auto status = driver_->get_last_status();
        message.data = (status.g_obj == 1 || status.g_obj == 2);
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
    if (running_ || serial_leased_)
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
            send_recorded_position(convertToGripperSystemPosition(position), position, speed, force, "confidence");

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
    if (running_ || serial_leased_)
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
        send_recorded_position(convertToGripperSystemPosition(position), position, speed, force, "binary");

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
