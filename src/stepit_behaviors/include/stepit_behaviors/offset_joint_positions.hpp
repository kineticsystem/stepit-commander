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
#include <vector>

#include <behaviortree_cpp/action_node.h>

namespace stepit_behaviors
{

/**
 * @brief Offsets the current joint positions by the commanded displacement, and
 * writes out the absolute positions to reach.
 *
 * This node moves nothing: it only turns a relative command into the absolute
 * targets that FollowJointTrajectory then sends to the controller.
 *
 * The displacement carries its own sign, and the sign convention of the robot
 * is the one of the joint positions themselves: a negative offset decreases the
 * joint position, which on the StepIt motors means turning clockwise. Nothing
 * here is bound to rotary joints: on a prismatic joint the very same offset is
 * a distance.
 */
class OffsetJointPositions : public BT::SyncActionNode
{
public:
  OffsetJointPositions(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace stepit_behaviors
