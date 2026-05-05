#include <array>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// 必要なヘッダーを追加
#include <geometry_msgs/msg/pose_stamped.hpp>

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
    // タイムスタンプを現在時刻に設定
    goal.trajectory.header.stamp = rclcpp::Clock().now();

    // --- 修正ポイント: 許容誤差 (Tolerance) の設定 ---
    // 目標位置に対して 0.002 rad (約0.11度) の誤差を許容する
    // これにより、微小な振動や収束待ちによる Phase 93 でのフリーズを防止します
    for (const auto & name : joint_names) {
        control_msgs::msg::JointTolerance tol;
        tol.name = name;
        tol.position = 0.002;  // 許容する位置誤差
        tol.velocity = 0.01;   // 許容する速度誤差 (停止判定の緩和)
        goal.goal_tolerance.push_back(tol);
    }
    // ----------------------------------------------

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
            // "init " の後の文字列を取り出す
            std::stringstream ss(cmd.substr(5)); 
            double gripper_width = 0.0;
            std::vector<double> joints_pos(6);
            
            // 1. まずグリッパ幅を読み込む
            if (!(ss >> gripper_width)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse gripper width from: %s", cmd.c_str());
                return;
            }
            
            // 2. 次に6つの関節角度を順番に読み込む
            for(int i = 0; i < 6; ++i) {
                if (!(ss >> joints_pos[i])) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to parse joint %d from: %s", i + 1, cmd.c_str());
                    return;
                }
            }
            
            prepare_init_sequence_direct(joints_pos, gripper_width);
        }
        else if (cmd.find("run") == 0) {
            double target_pos = 0.05; 
            double target_speed = 0.1; 

            // substr(3) にして "run" の直後（スペース含む）から読み込ませるか、
            // 明示的にスペースをスキップさせます
            std::string params = cmd.substr(3); 
            std::stringstream ss(params);
            
            if (ss >> target_pos >> target_speed) {
                RCLCPP_INFO(this->get_logger(), "Parsed command: pos=%.3f, speed=%.3f", target_pos, target_speed);
            } else {
                RCLCPP_WARN(this->get_logger(), "Parse failed for: %s. Using default speed 0.1", cmd.c_str());
            }
            
            prepare_run_sequence(target_pos, target_speed);
        }
        else {
            return;
        }

        current_step_index_ = 0;
        send_next_step();
    }

    // 現在のデカルト座標を保持する構造体
    struct Pose6D {
        double x, y, z, rx, ry, rz;
    } current_pose_;

    void prepare_init_sequence_direct(const std::vector<double>& joints_pos, double g_width) {
        steps_.clear(); //
        const std::vector<std::string> joints = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        }; //
        
        // --- デバッグ用：開始の合図（一度全閉にする） ---
        MoveGripper::Goal debug_start;
        debug_start.target_position = 0.0f; // 全閉
        debug_start.target_speed = 0.5f;
        steps_.push_back(make_gripper_step(debug_start, std::chrono::seconds(1), 91)); // Phase 91

        // 1. 本来のグリッパ動作（指定幅へ開く）
        MoveGripper::Goal g_goal;
        g_goal.target_position = static_cast<float>(g_width);
        steps_.push_back(make_gripper_step(g_goal, std::chrono::seconds(1), 92)); // Phase 92

        // 2. アームの移動
        RawTrajectoryPoint p;
        std::copy(joints_pos.begin(), joints_pos.end(), p.positions.begin());
        p.time_from_start = std::chrono::seconds(4);
        steps_.push_back(make_arm_step(create_follow_joint_goal(joints, {p}), std::chrono::seconds(1), 93)); // Phase 93
    }


    void prepare_run_sequence(double target_pos, double target_speed) {
        steps_.clear();
        // 引数で受け取った target_speed を適用する
        auto make_g = [target_speed](float pos, float force) {
            MoveGripper::Goal g; 
            g.target_position = pos; 
            g.target_speed = static_cast<float>(target_speed); 
            g.target_force = force; 
            return g;
        };

        // Phase 1: 開く (ここは素早く 0.5 固定でもOK)
        steps_.push_back(make_gripper_step(make_g(0.140f, 0.1f), std::chrono::seconds(1), 1));
        
        // Phase 2: 把持 (ここを指定されたランダム速度にする)
        steps_.push_back(make_gripper_step(make_g(static_cast<float>(target_pos), 0.5f), std::chrono::seconds(3), 2));
        
        // Phase 3: 開く
        steps_.push_back(make_gripper_step(make_g(0.140f, 0.1f), std::chrono::seconds(1), 3));
    }
    void send_next_step() {
        if (current_step_index_ >= steps_.size()) {
            RCLCPP_INFO(this->get_logger(), "シーケンス完了。待機します。");
            action_in_progress_ = false;
            auto p = std_msgs::msg::Int32();
            p.data = 0; 
            phase_pub_->publish(p);
            return;
        }

        action_in_progress_ = true;
        const auto &step = steps_.at(current_step_index_);
        
        auto phase_msg = std_msgs::msg::Int32();
        phase_msg.data = step.phase_id;
        phase_pub_->publish(phase_msg); 

        if (step.type == StepType::ArmTrajectory) {
            auto opts = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
            // --- ここに挿入 ---
            opts.result_callback = [this](const auto & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    this->on_step_completed();
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Arm Action failed with code: %d", static_cast<int>(result.code));
                    this->action_in_progress_ = false;
                    auto p = std_msgs::msg::Int32();
                    p.data = 0;
                    this->phase_pub_->publish(p);
                }
            };
            // ----------------
            arm_client_->async_send_goal(step.arm_goal, opts);
        } else {
            auto opts = rclcpp_action::Client<MoveGripper>::SendGoalOptions();
            // --- グリッパ側も同様に挿入 ---
            opts.result_callback = [this](const auto & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    this->on_step_completed();
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Gripper Action failed with code: %d", static_cast<int>(result.code));
                    this->action_in_progress_ = false;
                    auto p = std_msgs::msg::Int32();
                    p.data = 0;
                    this->phase_pub_->publish(p);
                }
            };
            // ----------------
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