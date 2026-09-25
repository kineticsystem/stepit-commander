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

#include <string>

#include <behaviortree_ros2/bt_action_node.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace stepit_behaviors
{

/**
 * @brief Sends a trajectory to a joint trajectory controller, and waits until the
 * joints have followed it.
 *
 * This is a thin behavior tree wrapper around the FollowJointTrajectory action
 * exposed by the joint_trajectory_controller of the StepIt robot. The trajectory
 * comes from another node, e.g. CubicTrajectory: this one only sends it.
 */
class FollowJointTrajectory : public BT::RosActionNode<control_msgs::action::FollowJointTrajectory>
{
public:
  FollowJointTrajectory(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;

  BT::NodeStatus onResultReceived(const WrappedResult& result) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  void onHalt() override;
};

}  // namespace stepit_behaviors
