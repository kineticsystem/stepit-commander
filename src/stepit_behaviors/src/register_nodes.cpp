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

#include "stepit_behaviors/register_nodes.hpp"

#include "stepit_behaviors/follow_joint_trajectory.hpp"
#include "stepit_behaviors/get_active_controllers.hpp"
#include "stepit_behaviors/get_joint_positions.hpp"
#include "stepit_behaviors/offset_joint_positions.hpp"
#include "stepit_behaviors/switch_controller.hpp"

namespace stepit_behaviors
{

void registerNodes(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& params)
{
  factory.registerNodeType<OffsetJointPositions>("OffsetJointPositions");
  factory.registerNodeType<GetJointPositions>("GetJointPositions", params);
  factory.registerNodeType<FollowJointTrajectory>("FollowJointTrajectory", params);
  factory.registerNodeType<GetActiveControllers>("GetActiveControllers", params);
  factory.registerNodeType<SwitchController>("SwitchController", params);
}

}  // namespace stepit_behaviors
