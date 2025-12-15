/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : joystick_servo_example.cpp
 *      Project   : moveit_servo
 *      Created   : 08/07/2020
 *      Author    : Adam Pettinger
 */

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <unordered_map>
#include <map>
#include <thread>
#include <cmath>
#include <array>
#include <algorithm>

#include <rclcpp_action/rclcpp_action.hpp>
#include <robotiq_2f_gripper_msgs/action/move_two_finger_gripper.hpp>

namespace
{

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "wrist_3_link";
const std::string BASE_FRAME_ID = "base_link";
const std::string GRIPPER_ACTION_NAME = "/robotiq_2f_gripper_action";
const std::string GRIPPER_GOAL_LOG_TOPIC = "/robotiq_2f_gripper_action_goal";  // logging-only, for bagging
const double GRIPPER_OPEN_POSITION = 0.140;
const double GRIPPER_CLOSE_POSITION = 0.0;
const double GRIPPER_TARGET_SPEED = 0.15;
const double GRIPPER_TARGET_FORCE = 1.0;
const double GRIPPER_STEP = 0.008;  // Smaller increment per joystick event for smoother motion
const double GRIPPER_HOME_POSITION = 0.100;  // Desired opening when sending robot to home via D-Pad

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_X = 6,
  D_PAD_Y = 7
};
enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6,
  MENU = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10,
  F310_DPAD_UP = 11,
  F310_DPAD_RIGHT = 12,
  F310_DPAD_DOWN = 13,
  F310_DPAD_LEFT = 14,
  BUTTON_COUNT
};

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };
std::map<Button, double> BUTTON_DEFAULTS;

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     std::unique_ptr<control_msgs::msg::JointJog>& joint)
{
  auto getDpadAxis = [&axes, &buttons](Axis axis_index, int positive_button, int negative_button) {
    double value = 0.0;
    if (static_cast<std::size_t>(axis_index) < axes.size())
    {
      value = axes[axis_index];
    }

    // Logitech F310 (DirectInput mode) exposes the D-Pad as buttons.
    if (std::fabs(value) < 1e-3)
    {
      const double positive = (static_cast<std::size_t>(positive_button) < buttons.size()) ? buttons[positive_button] : 0;
      const double negative = (static_cast<std::size_t>(negative_button) < buttons.size()) ? buttons[negative_button] : 0;
      value = positive - negative;
    }
    return value;
  };

  const double dpad_x = getDpadAxis(D_PAD_X, F310_DPAD_RIGHT, F310_DPAD_LEFT);
  const double dpad_y = getDpadAxis(D_PAD_Y, F310_DPAD_UP, F310_DPAD_DOWN);

  // Give joint jogging priority because it is only buttons
  // If any joint jog command is requested, we are only publishing joint commands
  if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || std::fabs(dpad_x) > 1e-3 || std::fabs(dpad_y) > 1e-3)
  {
    // Map the D_PAD to the proximal joints
    joint->joint_names.push_back("elbow_joint");
    joint->velocities.push_back(dpad_x);
    joint->joint_names.push_back("shoulder_lift_joint");
    joint->velocities.push_back(dpad_y);

    // Map the diamond to the distal joints
    joint->joint_names.push_back("wrist_3_joint");
    joint->velocities.push_back(buttons[B] - buttons[X]);
    joint->joint_names.push_back("wrist_2_joint");
    joint->velocities.push_back(buttons[Y] - buttons[A]);
    return false;
  }

  // The bread and butter: map buttons to twist commands
  // X/Y: use right stick with intuitive mapping (up = forward, right = right).
  twist->twist.linear.x = -axes[RIGHT_STICK_X];
  twist->twist.linear.y = -axes[RIGHT_STICK_Y];

  double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
  double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
  twist->twist.linear.z = -lin_x_right - lin_x_left;

  twist->twist.angular.y = axes[LEFT_STICK_Y];
  twist->twist.angular.x = axes[LEFT_STICK_X];

  double roll_positive = (static_cast<std::size_t>(RIGHT_STICK_CLICK) < buttons.size()) ? buttons[RIGHT_STICK_CLICK] : 0.0;
  double roll_negative = (static_cast<std::size_t>(LEFT_STICK_CLICK) < buttons.size()) ? buttons[LEFT_STICK_CLICK] : 0.0;
  twist->twist.angular.z = roll_positive - roll_negative;

  return true;
}

/** \brief // This should update the frame_to_publish_ as needed for changing command frame via controller
 * @param frame_name Set the command frame to this
 * @param buttons The vector of discrete controller button values
 */
void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons)
{
  if (buttons[CHANGE_VIEW] && frame_name == EEF_FRAME_ID)
    frame_name = BASE_FRAME_ID;
  else if (buttons[MENU] && frame_name == BASE_FRAME_ID)
    frame_name = EEF_FRAME_ID;
}

}  // namespace

namespace teleop_servo
{
class JoyToServoPubUr : public rclcpp::Node
{
  using GripperAction = robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperAction>;
public:
  JoyToServoPubUr(const rclcpp::NodeOptions& options)
    : Node("joy_to_twist_publisher", options), frame_to_publish_(BASE_FRAME_ID)
  {
    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) { jointStateCB(msg); });

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
    collision_pub_ =
        this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", rclcpp::SystemDefaultsQoS());

    // Logging-only: publish every gripper goal so it can be recorded in bag files.
    gripper_goal_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(GRIPPER_GOAL_LOG_TOPIC, rclcpp::SystemDefaultsQoS());
    gripper_goal_timer_ = this->create_wall_timer(
        std::chrono::microseconds(8000),
        [this]() { publishGripperGoalLog(); });

    gripper_action_client_ = rclcpp_action::create_client<GripperAction>(this, GRIPPER_ACTION_NAME);
    gripper_position_ = GRIPPER_OPEN_POSITION;
    last_gripper_target_ = gripper_position_;

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    // Load the collision scene asynchronously
    // collision_pub_thread_ = std::thread([this]() {
    //   rclcpp::sleep_for(std::chrono::seconds(3));
    //   // Create collision object, in the way of servoing
    //   moveit_msgs::msg::CollisionObject collision_object;
    //   collision_object.header.frame_id = "base_link";
    //   collision_object.id = "box";

    //   shape_msgs::msg::SolidPrimitive table_1;
    //   table_1.type = table_1.BOX;
    //   table_1.dimensions = { 0.4, 0.6, 0.03 };

    //   geometry_msgs::msg::Pose table_1_pose;
    //   table_1_pose.position.x = 0.6;
    //   table_1_pose.position.y = 0.0;
    //   table_1_pose.position.z = 0.4;

    //   shape_msgs::msg::SolidPrimitive table_2;
    //   table_2.type = table_2.BOX;
    //   table_2.dimensions = { 0.6, 0.4, 0.03 };

    //   geometry_msgs::msg::Pose table_2_pose;
    //   table_2_pose.position.x = 0.0;
    //   table_2_pose.position.y = 0.5;
    //   table_2_pose.position.z = 0.25;

    //   collision_object.primitives.push_back(table_1);
    //   collision_object.primitive_poses.push_back(table_1_pose);
    //   collision_object.primitives.push_back(table_2);
    //   collision_object.primitive_poses.push_back(table_2_pose);
    //   collision_object.operation = collision_object.ADD;

    //   moveit_msgs::msg::PlanningSceneWorld psw;
    //   psw.collision_objects.push_back(collision_object);

    //   auto ps = std::make_unique<moveit_msgs::msg::PlanningScene>();
    //   ps->world = psw;
    //   ps->is_diff = true;
    //   collision_pub_->publish(std::move(ps));
    // });
  }

  ~JoyToServoPubUr() override
  {
    if (collision_pub_thread_.joinable())
      collision_pub_thread_.join();
  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // This call updates the frame for twist commands
    updateCmdFrame(frame_to_publish_, msg->buttons);

    handleGripperButtons(msg->buttons);

    // If any D-Pad input is pressed, drive toward the predefined home pose using joint jogs.
    if (handleHomeDpad(msg))
    {
      return;
    }

    // Convert the joystick message to Twist or JointJog and publish
    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
    {
      // Publish twist in the end-effector frame so X/Y are relative to the tool orientation.
      twist_msg->header.frame_id = EEF_FRAME_ID;
      twist_msg->header.stamp = this->now();
      twist_pub_->publish(std::move(twist_msg));
    }
    else
    {
      // publish the JointJog
      joint_msg->header.stamp = this->now();
      joint_msg->header.frame_id = "base_link"; // Not used? Just filling for completeness now
      joint_pub_->publish(std::move(joint_msg));
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gripper_goal_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp_action::Client<GripperAction>::SharedPtr gripper_action_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr gripper_goal_timer_;

  std::string frame_to_publish_;
  double gripper_position_;
  double last_gripper_target_;
  rclcpp::Time last_gripper_send_time_;
  bool gripper_goal_active_ {false};

  bool home_active_ {false};

  std::unordered_map<std::string, double> joint_positions_;
  bool joint_state_ready_ {false};
  bool warned_joint_state_missing_ {false};

  const std::map<std::string, double> home_pose_rad_ {
    {"shoulder_pan_joint", 45.0 * M_PI / 180.0},
    {"shoulder_lift_joint", -90.0 * M_PI / 180.0},
    {"elbow_joint", 90.0 * M_PI / 180.0},
    {"wrist_1_joint", -90.0 * M_PI / 180.0},
    {"wrist_2_joint", -90.0 * M_PI / 180.0},
    {"wrist_3_joint", 0.0},
  };

  std::thread collision_pub_thread_;

  void handleGripperButtons(const std::vector<int>& buttons)
  {
    auto get_button = [&buttons](Button button) {
      const auto idx = static_cast<std::size_t>(button);
      return (idx < buttons.size()) ? buttons[idx] : 0;
    };

    const bool rb_pressed = get_button(RIGHT_BUMPER) > 0;   // open
    const bool lb_pressed = get_button(LEFT_BUMPER) > 0;     // close

    const double delta = (rb_pressed ? 1.0 : 0.0) - (lb_pressed ? 1.0 : 0.0);
    if (std::fabs(delta) < 1e-3)
    {
      return;  // No gripper request this cycle
    }

    const double new_target = std::clamp(
        gripper_position_ + delta * GRIPPER_STEP,
        GRIPPER_CLOSE_POSITION,
        GRIPPER_OPEN_POSITION);

    // Throttle commands to avoid spamming the action server
    const auto now = this->now();
    const bool position_changed = std::fabs(new_target - last_gripper_target_) > 5e-5;
    const bool time_elapsed = (last_gripper_send_time_.nanoseconds() == 0) ||
                  ((now - last_gripper_send_time_).seconds() > 0.004);

    gripper_position_ = new_target;
    if (position_changed && time_elapsed)
    {
      sendGripperCommand(gripper_position_);
      last_gripper_target_ = gripper_position_;
      last_gripper_send_time_ = now;
    }
  }

  bool handleHomeDpad(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    const auto& buttons = msg->buttons;
    const auto& axes = msg->axes;

    auto get_button = [&buttons](Button button) {
      const auto idx = static_cast<std::size_t>(button);
      return (idx < buttons.size()) ? buttons[idx] : 0;
    };

    const bool dpad_button_pressed =
        get_button(F310_DPAD_UP) || get_button(F310_DPAD_DOWN) ||
        get_button(F310_DPAD_LEFT) || get_button(F310_DPAD_RIGHT);

    auto get_axis = [&axes](Axis axis_index) {
      if (static_cast<std::size_t>(axis_index) < axes.size())
      {
        return axes[axis_index];
      }
      return 0.0f;
    };
    const double dpad_x_axis = get_axis(D_PAD_X);
    const double dpad_y_axis = get_axis(D_PAD_Y);
    const bool dpad_axis_pressed = (std::fabs(dpad_x_axis) > 0.5) || (std::fabs(dpad_y_axis) > 0.5);

    const bool go_home = dpad_button_pressed || dpad_axis_pressed;

    // Latch a home request on the first press; keep running until the arm reaches the deadband.
    const bool start_home = go_home && !home_active_;
    if (start_home)
    {
      home_active_ = true;

      // Kick the gripper to its home opening once per request.
      gripper_position_ = GRIPPER_HOME_POSITION;
      last_gripper_target_ = GRIPPER_HOME_POSITION;
      sendGripperCommand(GRIPPER_HOME_POSITION);
    }

    // If no request is active, nothing to do.
    if (!home_active_)
    {
      return false;
    }

    if (!joint_state_ready_)
    {
      if (!warned_joint_state_missing_)
      {
        RCLCPP_WARN(this->get_logger(), "Joint state not yet received; cannot drive to home pose");
        warned_joint_state_missing_ = true;
      }
      return true;  // consume while we wait for state
    }

    auto jog = control_msgs::msg::JointJog();
    jog.header.stamp = this->now();
    jog.header.frame_id = BASE_FRAME_ID;

    const double max_vel = 1.5;  // rad/s cap for fast homing (avoid overshoot)
    const double deadband = 0.01;  // rad

    for (const auto& [name, target] : home_pose_rad_)
    {
      auto it = joint_positions_.find(name);
      if (it == joint_positions_.end())
      {
        continue;
      }
      const double current = it->second;
      const double error = target - current;
      if (std::fabs(error) < deadband)
      {
        continue;
      }
      const double vel = std::clamp(error, -max_vel, max_vel);
      jog.joint_names.push_back(name);
      jog.velocities.push_back(vel);
    }

    if (!jog.joint_names.empty())
    {
      joint_pub_->publish(jog);
      return true;  // still moving toward home
    }

    // Reached within the deadband of home. Publish a zero-velocity halt to avoid lingering motion.
    auto halt = control_msgs::msg::JointJog();
    halt.header.stamp = this->now();
    halt.header.frame_id = BASE_FRAME_ID;
    for (const auto& [name, _] : home_pose_rad_)
    {
      halt.joint_names.push_back(name);
      halt.velocities.push_back(0.0);
    }
    joint_pub_->publish(halt);

    home_active_ = false;
    return true;
  }

  void jointStateCB(const sensor_msgs::msg::JointState::ConstSharedPtr& msg)
  {
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
    {
      joint_positions_[msg->name[i]] = msg->position[i];
    }
    joint_state_ready_ = true;
  }

  void sendGripperCommand(double target_position)
  {
    if (!gripper_action_client_)
      return;

    // Avoid spamming the gripper server with overlapping goals.
    if (gripper_goal_active_)
      return;

    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(0)))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Gripper action server not available on %s", GRIPPER_ACTION_NAME.c_str());
      return;
    }

    auto goal = GripperAction::Goal();
    goal.target_position = target_position;
    goal.target_speed = GRIPPER_TARGET_SPEED;
    goal.target_force = GRIPPER_TARGET_FORCE;

    // Log the goal for bagging/analysis; publish [position, speed, force].
    if (gripper_goal_pub_)
    {
      std_msgs::msg::Float32MultiArray log_msg;
      log_msg.data = {static_cast<float>(goal.target_position),
                      static_cast<float>(goal.target_speed),
                      static_cast<float>(goal.target_force)};
      gripper_goal_pub_->publish(log_msg);
    }

    rclcpp_action::Client<GripperAction>::SendGoalOptions options;
    options.goal_response_callback = [this](auto future)
    {
      const auto handle = future.get();
      if (!handle)
      {
        gripper_goal_active_ = false;
      }
    };
    options.result_callback = [this](auto /*result*/)
    {
      gripper_goal_active_ = false;
    };

    gripper_goal_active_ = true;
    gripper_action_client_->async_send_goal(goal, options);
  }

  void publishGripperGoalLog()
  {
    if (!gripper_goal_pub_)
      return;

    std_msgs::msg::Float32MultiArray log_msg;
    log_msg.data = {static_cast<float>(last_gripper_target_),
                    static_cast<float>(GRIPPER_TARGET_SPEED),
                    static_cast<float>(GRIPPER_TARGET_FORCE)};
    gripper_goal_pub_->publish(log_msg);
  }
};  // class JoyToServoPubUr

}  // namespace teleop_servo

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(teleop_servo::JoyToServoPubUr)
