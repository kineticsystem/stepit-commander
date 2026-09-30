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

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace stepit_behaviors
{

/**
 * @brief Sends joints to positions through a position controller, and waits
 * until they are there.
 *
 * A position controller, e.g. a position_controllers/JointGroupPositionController,
 * takes one position per joint it is configured with, in order, on its
 * `~/commands` topic, and the hardware moves there on its own profile: on
 * StepIt, the trapezoid of the microcontroller, as fast as the motors allow.
 * The message names no joint, so moving some joints means sending the others
 * where they already are: `controller_joints` lists the joints of the
 * controller, in its order, and the positions of the others come from the
 * latest joint state.
 *
 * It succeeds once every moved joint is within `tolerance` of its target and
 * has stopped, as read on the joint states, and fails after `timeout`
 * seconds. The joints are not synchronised: each one runs its own profile.
 *
 * Halting it, or a timeout, deactivates the position controller, without
 * waiting for the answer: the hardware then brakes every joint the controller
 * released to rest, on its own profile, where it naturally stops. Sending the
 * joints where they are would not do: a joint moving at speed cannot stop
 * there, it would brake past it and come back. The next objective activates
 * the controller it needs, as every motion objective does first.
 */
class CommandJointPositions : public BT::StatefulActionNode
{
public:
  CommandJointPositions(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /// @brief Create the subscription and the publisher, unless already done. Returns whether both exist.
  bool connect();

  /// @brief The position of each of the given joints in the latest joint state, if all are in it.
  std::optional<std::vector<double>> positionsOf(const std::vector<std::string>& joints) const;

  /// @brief Deactivate the controller, for the hardware to bring the joints to rest.
  void stop();

  /// @brief Whether every moved joint is within tolerance of its target, and stopped.
  bool arrived() const;

  std::weak_ptr<rclcpp::Node> node_;
  rclcpp::Logger logger_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  sensor_msgs::msg::JointState::SharedPtr last_state_;

  std::vector<std::string> controller_joints_;
  std::vector<std::string> joints_;
  std::vector<double> targets_;
  double tolerance_{ 0.0 };
  double velocity_tolerance_{ 0.0 };
  double timeout_{ 0.0 };
  bool sent_{ false };
  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace stepit_behaviors
