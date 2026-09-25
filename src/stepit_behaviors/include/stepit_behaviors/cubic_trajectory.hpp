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

#include <behaviortree_cpp/action_node.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace stepit_behaviors
{

/**
 * @brief Builds a cubic trajectory to a set of absolute joint positions, reached
 * at rest after `duration` seconds.
 *
 * The trajectory is a single waypoint, with positions and zero velocities: the
 * cubic comes from the controller, which joins the joints' current state to the
 * waypoint with one. Every joint starts and stops at rest, and all of them
 * arrive together. The acceleration peaks only at the start and the end, and
 * the speed only halfway: TrapezoidalTrajectory is faster within the same
 * limits.
 *
 * This node moves nothing: FollowJointTrajectory sends the trajectory to the
 * controller.
 */
class CubicTrajectory : public BT::SyncActionNode
{
public:
  /// @brief Default duration of the motion, in seconds, when no duration is given.
  static constexpr double kDefaultDuration = 5.0;

  CubicTrajectory(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace stepit_behaviors
