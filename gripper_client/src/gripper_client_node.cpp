#include <array>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_tolerance.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "robotiq_2f_gripper_msgs/action/move_two_finger_gripper.hpp"

using MoveGripper = robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
using GoalHandleGripper = rclcpp_action::ClientGoalHandle<MoveGripper>;

enum class StepType {
    ArmTrajectory,
    GripperMove
};

// 動作ステップの構造体
struct SequenceStep {
    StepType type{StepType::GripperMove};
    int phase_id{0}; // データ収集時のタイムスタンプ用ID
    FollowJointTrajectory::Goal arm_goal;
    MoveGripper::Goal gripper_goal;
    std::chrono::nanoseconds post_delay{std::chrono::seconds(1)};
};

struct RawTrajectoryPoint {
    std::array<double, 6> positions{};
    std::array<double, 6> velocities{};
    std::chrono::seconds time_from_start{0};
};

// --- ヘルパー関数群 ---
SequenceStep make_arm_step(FollowJointTrajectory::Goal goal, std::chrono::nanoseconds delay, int phase = 0) {
    SequenceStep step;
    step.type = StepType::ArmTrajectory;
    step.phase_id = phase;
    step.arm_goal = std::move(goal);
    step.post_delay = delay;
    return step;
}

SequenceStep make_gripper_step(MoveGripper::Goal goal, std::chrono::nanoseconds delay, int phase = 0) {
    SequenceStep step;
    step.type = StepType::GripperMove;
    step.phase_id = phase;
    step.gripper_goal = std::move(goal);
    step.post_delay = delay;
    return step;
}

FollowJointTrajectory::Goal create_follow_joint_goal(
    const std::vector<std::string> &joint_names,
    const std::vector<RawTrajectoryPoint> &points)
{
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joint_names;
    goal.trajectory.header.stamp = rclcpp::Clock().now(); // タイムスタンプ拒否対策

    for (const auto &raw_point : points) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.assign(raw_point.positions.begin(), raw_point.positions.end());
        point.velocities.assign(raw_point.velocities.begin(), raw_point.velocities.end());
        point.time_from_start.sec = static_cast<int32_t>(raw_point.time_from_start.count());
        goal.trajectory.points.push_back(point);
    }
    return goal;
}

class PickPlaceClient : public rclcpp::Node {
public:
    PickPlaceClient(std::string controller_name)
    : rclcpp::Node("pick_place_client"),
      controller_action_name_(std::move(controller_name) + "/follow_joint_trajectory"),
      arm_client_(rclcpp_action::create_client<FollowJointTrajectory>(this, controller_action_name_)),
      gripper_client_(rclcpp_action::create_client<MoveGripper>(this, "/robotiq_2f_gripper_action")),
      current_step_index_(0),
      action_in_progress_(false)
    {
        // 外部（Python）からのコマンド受信
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_cmd", 10, std::bind(&PickPlaceClient::on_command_received, this, std::placeholders::_1));

        // 現在のフェーズIDの通知用
        phase_pub_ = this->create_publisher<std_msgs::msg::Int32>("/current_phase", 10);

        RCLCPP_INFO(this->get_logger(), "PickPlaceClient準備完了。'/robot_cmd' を待機中...");
    }

private:
    // 2. コマンド受信部分で文字列をパースする
    void on_command_received(const std_msgs::msg::String::SharedPtr msg) {
        if (action_in_progress_) return;

        std::string cmd = msg->data;
        if (cmd.find("init") == 0) {
            std::vector<double> joints_pos = {1.23128, -0.982256, 0.955627, -1.57, -1.57, 0.0}; // デフォルト
            
            if (cmd.length() > 5) {
                std::stringstream ss(cmd.substr(5));
                std::vector<double> parsed_pos;
                double val;
                while (ss >> val) parsed_pos.push_back(val);
                
                if (parsed_pos.size() == 6) {
                    joints_pos = parsed_pos;
                } else {
                    RCLCPP_WARN(this->get_logger(), "関節数が正しくありません(6つ必要)。デフォルトを使用。");
                }
            }
            prepare_init_sequence(joints_pos);
        }
        else if (cmd.find("run") == 0) { // "run" で始まる場合
            double target_pos = 0.05; // デフォルト値
            
            // "run 0.045" のように数値が含まれていれば抽出
            if (cmd.length() > 4) {
                try {
                    target_pos = std::stod(cmd.substr(4));
                } catch (...) {
                    RCLCPP_ERROR(this->get_logger(), "数値のパースに失敗しました。デフォルト値を使用します。");
                }
            }
            prepare_run_sequence(target_pos);
        } else {
            return;
        }

        current_step_index_ = 0;
        send_next_step();
    }

    void prepare_init_sequence(const std::vector<double>& joints_pos) {
        steps_.clear();
        const std::vector<std::string> joints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        
        // 1. グリッパを開く
        MoveGripper::Goal open_goal;
        open_goal.target_position = 0.1f; open_goal.target_speed = 0.1f; open_goal.target_force = 0.1f;
        steps_.push_back(make_gripper_step(open_goal, std::chrono::seconds(1), 9));
        
        // 受け取った角度をセット
        RawTrajectoryPoint p;
        std::copy(joints_pos.begin(), joints_pos.end(), p.positions.begin());
        p.velocities = {0,0,0,0,0,0};
        p.time_from_start = std::chrono::seconds(5);

        // 2. アームを初期位置へ移動
        // const std::vector<RawTrajectoryPoint> point = {{{1.03128, -0.982256, 0.955627, -1.57, -1.57, 0.0}, {0,0,0,0,0,0}, std::chrono::seconds(5)}};
        // const std::vector<RawTrajectoryPoint> points = {{{1.23128, -0.982256, 0.955627, -1.57, -1.57, 0.0}, {0,0,0,0,0,0}, std::chrono::seconds(5)}};
        // steps_.push_back(make_arm_step(create_follow_joint_goal(joints, point), std::chrono::seconds(2), 0));
        steps_.push_back(make_arm_step(create_follow_joint_goal(joints, {p}), std::chrono::seconds(2), 9));
        RCLCPP_INFO(this->get_logger(), "初期化シーケンスを準備しました。");
    }

    void prepare_run_sequence(double target_pos) {
        steps_.clear();
        // const std::vector<std::string> joints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        
        auto make_g = [](float pos) {
            MoveGripper::Goal g; g.target_position = pos; g.target_speed = 0.1f; g.target_force = 0.1f; return g;
        };
        // 開(1) -> 閉(2) -> 開(3) のサイクル
        steps_.push_back(make_gripper_step(make_g(0.1f), std::chrono::seconds(1), 1));
        // const std::vector<RawTrajectoryPoint> points = {{{1.23128, -0.982256, 0.955627, -1.57, -1.57, 0.0}, {0,0,0,0,0,0}, std::chrono::seconds(5)}};
        // steps_.push_back(make_arm_step(create_follow_joint_goal(joints, points), std::chrono::seconds(2), 0));
        steps_.push_back(make_gripper_step(make_g(static_cast<float>(target_pos)), std::chrono::seconds(3), 2));
        steps_.push_back(make_gripper_step(make_g(0.1f), std::chrono::seconds(1), 3));
        RCLCPP_INFO(this->get_logger(), "データ収集シーケンス準備完了 (Target: %f m)", target_pos);    }

    void send_next_step() {
        if (current_step_index_ >= steps_.size()) {
            RCLCPP_INFO(this->get_logger(), "シーケンス完了。待機します。");
            action_in_progress_ = false;
            auto p = std_msgs::msg::Int32(); p.data = 0; phase_pub_->publish(p); // 待機フェーズ
            return;
        }

        action_in_progress_ = true;
        const auto &step = steps_.at(current_step_index_);
        
        auto phase_msg = std_msgs::msg::Int32();
        phase_msg.data = step.phase_id;
        phase_pub_->publish(phase_msg); // 現在のフェーズを通知

        if (step.type == StepType::ArmTrajectory) {
            auto opts = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
            opts.result_callback = [this](const auto &) { this->on_step_completed(); };
            arm_client_->async_send_goal(step.arm_goal, opts);
        } else {
            auto opts = rclcpp_action::Client<MoveGripper>::SendGoalOptions();
            opts.result_callback = [this](const auto &) { this->on_step_completed(); };
            gripper_client_->async_send_goal(step.gripper_goal, opts);
        }
    }

    void on_step_completed() {
        auto delay = steps_.at(current_step_index_).post_delay;
        current_step_index_++;
        timer_ = this->create_wall_timer(delay, [this]() {
            this->timer_->cancel();
            this->send_next_step();
        });
    }

    // メンバ変数
    std::string controller_action_name_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
    rclcpp_action::Client<MoveGripper>::SharedPtr gripper_client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr phase_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<SequenceStep> steps_;
    std::size_t current_step_index_;
    bool action_in_progress_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    const std::string controller_name = "scaled_joint_trajectory_controller";
    // メインループ（spin）を開始
    rclcpp::spin(std::make_shared<PickPlaceClient>(controller_name));
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}