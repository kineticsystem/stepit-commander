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
#include <string>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace stepit_behaviors
{

/**
 * @brief Reads the current position of a set of joints from /joint_states.
 *
 * It subscribes when the tree is created, and keeps the latest message. When
 * ticked before any message has arrived, it waits for one (RUNNING) up to
 * `timeout` seconds, then fails. It fails at once when one of the requested
 * joints is not part of the message: waiting would not fix that.
 */
class GetJointPositions : public BT::StatefulActionNode
{
public:
  GetJointPositions(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /// @brief Subscribe to the topic of the `topic_name` port, unless already done.
  /// @return Whether the subscription exists.
  bool subscribe();

  /// @brief Write the positions of the latest message.
  /// @return SUCCESS, FAILURE if a joint is missing, RUNNING if no message has arrived yet.
  BT::NodeStatus read();

  std::weak_ptr<rclcpp::Node> node_;
  rclcpp::Logger logger_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  sensor_msgs::msg::JointState::SharedPtr last_msg_;
  double timeout_{ 0.0 };
  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace stepit_behaviors
