#include <array>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

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

enum class StepType
{
    ArmTrajectory,
    GripperMove
};

struct SequenceStep
{
    StepType type{StepType::GripperMove};
    FollowJointTrajectory::Goal arm_goal;
    MoveGripper::Goal gripper_goal;
    std::chrono::nanoseconds post_delay{std::chrono::seconds(1)};
};

struct RawTrajectoryPoint
{
    std::array<double, 6> positions{};
    std::array<double, 6> velocities{};
    std::chrono::seconds time_from_start{0};
};

SequenceStep make_arm_step(FollowJointTrajectory::Goal goal, std::chrono::nanoseconds delay)
{
    SequenceStep step;
    step.type = StepType::ArmTrajectory;
    step.arm_goal = std::move(goal);
    step.post_delay = delay;
    return step;
}

SequenceStep make_gripper_step(MoveGripper::Goal goal, std::chrono::nanoseconds delay)
{
    SequenceStep step;
    step.type = StepType::GripperMove;
    step.gripper_goal = std::move(goal);
    step.post_delay = delay;
    return step;
}

std::string follow_result_to_string(int32_t error_code)
{
    switch (error_code) {
        case FollowJointTrajectory::Result::SUCCESSFUL:
            return "SUCCESSFUL";
        case FollowJointTrajectory::Result::INVALID_GOAL:
            return "INVALID_GOAL";
        case FollowJointTrajectory::Result::INVALID_JOINTS:
            return "INVALID_JOINTS";
        case FollowJointTrajectory::Result::OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP";
        case FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED";
        case FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED";
        default:
            return "UNKNOWN";
    }
}

FollowJointTrajectory::Goal create_follow_joint_goal(
    const std::vector<std::string> &joint_names,
    const std::vector<RawTrajectoryPoint> &points)
{
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joint_names;

    goal.goal_time_tolerance.sec = 0;
    goal.goal_time_tolerance.nanosec = 500000000; // 0.5s

    goal.goal_tolerance.clear();
    goal.goal_tolerance.reserve(joint_names.size());
    for (const auto &name : joint_names) {
        control_msgs::msg::JointTolerance tolerance;
        tolerance.name = name;
        tolerance.position = 0.01;
        tolerance.velocity = 0.01;
        tolerance.acceleration = 0.0;
        goal.goal_tolerance.push_back(tolerance);
    }

    goal.trajectory.points.reserve(points.size());
    for (const auto &raw_point : points) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.assign(raw_point.positions.begin(), raw_point.positions.end());
        point.velocities.assign(raw_point.velocities.begin(), raw_point.velocities.end());
        point.accelerations.assign(raw_point.velocities.size(), 0.0);
        point.time_from_start.sec = static_cast<int32_t>(raw_point.time_from_start.count());
        point.time_from_start.nanosec = 0;
        goal.trajectory.points.push_back(point);
    }

    return goal;
}

class PickPlaceClient : public rclcpp::Node
{
public:
    PickPlaceClient(std::string controller_name,
                    std::vector<SequenceStep> steps)
    : rclcpp::Node("pick_place_client"),
      controller_action_name_(std::move(controller_name) + "/follow_joint_trajectory"),
      arm_client_(rclcpp_action::create_client<FollowJointTrajectory>(
          this, controller_action_name_)),
      gripper_client_(rclcpp_action::create_client<MoveGripper>(
          this, "/robotiq_2f_gripper_action")),
      steps_(std::move(steps)),
      current_step_index_(0),
      action_in_progress_(false)
    {
        if (steps_.empty()) {
            RCLCPP_WARN(this->get_logger(), "ステップが設定されていません。処理を終了します。");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "アームアクションサーバ %s を待機中...", controller_action_name_.c_str());
        if (!arm_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "アームアクションサーバが見つかりません。");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "グリッパアクションサーバ /robotiq_2f_gripper_action を待機中...");
        if (!gripper_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "グリッパアクションサーバが見つかりません。");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "全アクションサーバを検出しました。シーケンスを開始します。");
        schedule_next_step(std::chrono::seconds(0));
    }

private:
    void schedule_next_step(const std::chrono::nanoseconds &delay)
    {
        if (timer_) {
            timer_->cancel();
        }

        timer_ = this->create_wall_timer(
            delay,
            [this]() {
                if (timer_) {
                    timer_->cancel();
                }
                send_next_step();
            });
    }

    void send_next_step()
    {
        if (action_in_progress_) {
            RCLCPP_WARN(this->get_logger(), "前のアクションがまだ進行中です。");
            return;
        }

        if (current_step_index_ >= steps_.size()) {
            RCLCPP_INFO(this->get_logger(), "全てのステップが完了しました。");
            rclcpp::shutdown();
            return;
        }

        const auto step_index = current_step_index_;
        const auto &step = steps_.at(step_index);

        switch (step.type) {
            case StepType::ArmTrajectory:
                send_arm_goal(step_index, step.arm_goal);
                break;
            case StepType::GripperMove:
                send_gripper_goal(step_index, step.gripper_goal);
                break;
        }
    }

    void send_arm_goal(std::size_t step_index,
                       const FollowJointTrajectory::Goal &goal)
    {
        RCLCPP_INFO(this->get_logger(), "ステップ%zu: アーム軌道を送信します。", step_index + 1);
        action_in_progress_ = true;

        auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        options.goal_response_callback =
            [this, step_index](const GoalHandleFollow::SharedPtr &goal_handle) {
                if (!goal_handle) {
                    handle_failure("アーム軌道がサーバに拒否されました。");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "ステップ%zu: ゴールが受理されました。", step_index + 1);
            };

        options.feedback_callback =
            [this](GoalHandleFollow::SharedPtr,
                   const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
                if (!feedback) {
                    return;
                }
                if (!feedback->actual.positions.empty()) {
                    RCLCPP_DEBUG(this->get_logger(), "現在位置[0]=%.3f", feedback->actual.positions.front());
                }
            };

        options.result_callback =
            [this, step_index](const GoalHandleFollow::WrappedResult &result) {
                handle_arm_result(step_index, result);
            };

        arm_client_->async_send_goal(goal, options);
    }

    void handle_arm_result(std::size_t step_index,
                           const GoalHandleFollow::WrappedResult &result)
    {
        const auto &step = steps_.at(step_index);

        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            std::string code_str;
            switch (result.code) {
                case rclcpp_action::ResultCode::ABORTED:
                    code_str = "ABORTED";
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    code_str = "CANCELED";
                    break;
                default:
                    code_str = "UNKNOWN";
                    break;
            }
            handle_failure("アーム軌道が失敗しました: " + code_str);
            return;
        }

        if (!result.result) {
            handle_failure("アーム軌道の結果が取得できませんでした。");
            return;
        }

        if (result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL) {
            handle_failure("アーム軌道エラー: " +
                           follow_result_to_string(result.result->error_code));
            return;
        }

        RCLCPP_INFO(this->get_logger(), "ステップ%zu: アーム軌道が正常に完了しました。",
                    step_index + 1);
        handle_step_completion(step_index, step);
    }

    void send_gripper_goal(std::size_t step_index,
                           const MoveGripper::Goal &goal)
    {
        RCLCPP_INFO(this->get_logger(),
                    "ステップ%zu: グリッパ目標 position=%.3f speed=%.3f force=%.3f を送信します。",
                    step_index + 1, goal.target_position, goal.target_speed, goal.target_force);
        action_in_progress_ = true;

        auto options = rclcpp_action::Client<MoveGripper>::SendGoalOptions();
        options.goal_response_callback =
            [this, step_index](const GoalHandleGripper::SharedPtr &goal_handle) {
                if (!goal_handle) {
                    handle_failure("グリッパ目標がサーバに拒否されました。");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "ステップ%zu: グリッパ目標が受理されました。",
                            step_index + 1);
            };

        options.feedback_callback =
            [this](GoalHandleGripper::SharedPtr,
                   const std::shared_ptr<const MoveGripper::Feedback> feedback) {
                if (!feedback) {
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "グリッパフィードバック: %s", feedback->feedback.c_str());
            };

        options.result_callback =
            [this, step_index](const GoalHandleGripper::WrappedResult &result) {
                handle_gripper_result(step_index, result);
            };

        gripper_client_->async_send_goal(goal, options);
    }

    void handle_gripper_result(std::size_t step_index,
                               const GoalHandleGripper::WrappedResult &result)
    {
        const auto &step = steps_.at(step_index);

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                if (result.result && result.result->success) {
                    RCLCPP_INFO(this->get_logger(), "ステップ%zu: グリッパ成功 (success=true)。",
                                step_index + 1);
                    handle_step_completion(step_index, step);
                } else {
                    handle_failure("グリッパ目標は完了しましたが success=false でした。");
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                handle_failure("グリッパ目標が中断されました (ABORTED)。");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                handle_failure("グリッパ目標がキャンセルされました (CANCELED)。");
                break;
            default:
                handle_failure("グリッパ目標が不明なコードで終了しました。");
                break;
        }
    }

    void handle_step_completion(std::size_t step_index,
                                const SequenceStep &step)
    {
        action_in_progress_ = false;
        current_step_index_ = step_index + 1;

        if (current_step_index_ >= steps_.size()) {
            RCLCPP_INFO(this->get_logger(), "全てのステップが完了しました。ノードを終了します。");
            rclcpp::shutdown();
            return;
        }

        schedule_next_step(step.post_delay);
    }

    void handle_failure(const std::string &message)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        action_in_progress_ = false;
        rclcpp::shutdown();
    }

    std::string controller_action_name_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
    rclcpp_action::Client<MoveGripper>::SharedPtr gripper_client_;
    std::vector<SequenceStep> steps_;
    std::size_t current_step_index_;
    bool action_in_progress_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const std::string controller_name = "scaled_joint_trajectory_controller";
    const std::vector<std::string> joints = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    const std::vector<RawTrajectoryPoint> traj0_points = {
        {
            {1.23128, -0.982256, 0.955627, -1.57, -1.57, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            std::chrono::seconds(4)
        },
        {
            {1.23128, -0.982256, 0.955627, -1.57, -1.57, -0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            std::chrono::seconds(8)
        }
    };

    // const std::vector<RawTrajectoryPoint> traj1_points = {
    //     {
    //         {0.93128, -0.982256, 0.955627, -1.57, -1.57, -0.0},
    //         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    //         std::chrono::seconds(0)
    //     },
    //     {
    //         {0.30493, -0.982258, 0.955637, -1.57, -1.57, 0.0},
    //         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    //         std::chrono::seconds(8)
    //     }
    // };

    // const std::vector<RawTrajectoryPoint> traj2_points = {
    //     {
    //         {0.30493, -0.982258, 0.955637, -1.57, -1.57, 0.0},
    //         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    //         std::chrono::seconds(0)
    //     },
    //     {
    //         {0.93128, -1.70093, 0.902027, -1.57, -1.57, 0.0},
    //         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    //         std::chrono::seconds(8)
    //     }
    // };

    FollowJointTrajectory::Goal traj0_goal = create_follow_joint_goal(joints, traj0_points);
    // FollowJointTrajectory::Goal traj1_goal = create_follow_joint_goal(joints, traj1_points);
    // FollowJointTrajectory::Goal traj2_goal = create_follow_joint_goal(joints, traj2_points);

    const auto make_gripper_goal = [](float position) {
        MoveGripper::Goal goal;
        goal.target_position = position;
        goal.target_speed = 0.1f;
        goal.target_force = 0.1f;
        return goal;
    };

    std::vector<SequenceStep> steps;
    steps.reserve(6);
    steps.emplace_back(make_gripper_step(make_gripper_goal(0.08f), std::chrono::seconds(1)));
    steps.emplace_back(make_arm_step(std::move(traj0_goal), std::chrono::seconds(2)));
    steps.emplace_back(make_gripper_step(make_gripper_goal(0.05f), std::chrono::seconds(2)));
    steps.emplace_back(make_gripper_step(make_gripper_goal(0.08f), std::chrono::seconds(2)));
    // steps.emplace_back(make_gripper_step(make_gripper_goal(0.08f), std::chrono::seconds(1)));
    // steps.emplace_back(make_arm_step(std::move(traj0_goal), std::chrono::seconds(2)));
    // steps.emplace_back(make_gripper_step(make_gripper_goal(0.05f), std::chrono::seconds(1)));
    // steps.emplace_back(make_arm_step(std::move(traj1_goal), std::chrono::seconds(2)));
    // steps.emplace_back(make_gripper_step(make_gripper_goal(0.08f), std::chrono::seconds(1)));
    // steps.emplace_back(make_arm_step(std::move(traj2_goal), std::chrono::seconds(1)));

    rclcpp::spin(std::make_shared<PickPlaceClient>(controller_name, std::move(steps)));
    return EXIT_SUCCESS;
}
