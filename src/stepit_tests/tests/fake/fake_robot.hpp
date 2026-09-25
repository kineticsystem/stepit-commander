// Copyright 2026 Giovanni Remigi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace stepit_tests
{

/**
 * @brief A stand-in for the StepIt robot: it publishes joint states and accepts
 * trajectories, exactly like the joint_trajectory_controller does, and records
 * the goals it receives so that a test can check them.
 */
class FakeRobot
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  FakeRobot(const std::string& joint_state_topic, const std::string& action_name,
            const std::vector<std::string>& joint_names, const std::vector<double>& positions)
    : node_{ std::make_shared<rclcpp::Node>("fake_robot") }
  {
    state_.name = joint_names;
    state_.position = positions;

    // The joint_state_broadcaster of the robot publishes with the default
    // (reliable) QoS, and so must the fake robot, or the behaviors would not be
    // able to subscribe to it.
    publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, rclcpp::QoS{ 10 });
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(20), [this]() {
      state_.header.stamp = node_->now();
      publisher_->publish(state_);
    });

    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        node_, action_name,
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const FollowJointTrajectory::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle>&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const std::shared_ptr<GoalHandle>& goal_handle) { execute(goal_handle); });

    executor_.add_node(node_);
    spinner_ = std::thread{ [this]() { executor_.spin(); } };
  }

  ~FakeRobot()
  {
    executor_.cancel();
    if (spinner_.joinable())
    {
      spinner_.join();
    }
    executor_.remove_node(node_);
  }

  FakeRobot(const FakeRobot&) = delete;
  FakeRobot& operator=(const FakeRobot&) = delete;

  /// @brief The last trajectory the robot was asked to execute, if any.
  std::optional<trajectory_msgs::msg::JointTrajectory> lastTrajectory() const
  {
    const std::lock_guard<std::mutex> lock{ mutex_ };
    return last_trajectory_;
  }

  /// @brief Every trajectory the robot was asked to execute, in order.
  std::vector<trajectory_msgs::msg::JointTrajectory> trajectories() const
  {
    const std::lock_guard<std::mutex> lock{ mutex_ };
    return trajectories_;
  }

  /// @brief Make the next trajectory fail, as a controller in error would do.
  void failNextTrajectory()
  {
    fail_ = true;
  }

private:
  void execute(const std::shared_ptr<GoalHandle>& goal_handle)
  {
    {
      const std::lock_guard<std::mutex> lock{ mutex_ };
      last_trajectory_ = goal_handle->get_goal()->trajectory;
      trajectories_.push_back(*last_trajectory_);
    }

    auto result = std::make_shared<FollowJointTrajectory::Result>();
    if (fail_.exchange(false))
    {
      result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      result->error_string = "the fake robot was asked to fail";
      goal_handle->abort(result);
      return;
    }

    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  sensor_msgs::msg::JointState state_;

  mutable std::mutex mutex_;
  std::optional<trajectory_msgs::msg::JointTrajectory> last_trajectory_;
  std::vector<trajectory_msgs::msg::JointTrajectory> trajectories_;
  std::atomic_bool fail_{ false };

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
};

}  // namespace stepit_tests
