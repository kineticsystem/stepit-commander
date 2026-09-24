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
#include "stepit_behaviors/offset_joint_positions.hpp"

#include <algorithm>
#include <iterator>
#include <vector>

namespace stepit_behaviors
{

OffsetJointPositions::OffsetJointPositions(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList OffsetJointPositions::providedPorts()
{
  return {
    BT::InputPort<double>("offset", "signed displacement, in radians: it is negative when the motor turns clockwise"),
    BT::InputPort<std::vector<double>>("current_positions", "joint positions the motion starts from"),
    BT::OutputPort<std::vector<double>>("target_positions", "absolute joint positions to reach"),
  };
}

BT::NodeStatus OffsetJointPositions::tick()
{
  const auto offset = getInput<double>("offset");
  if (!offset)
  {
    throw BT::RuntimeError("OffsetJointPositions: ", offset.error());
  }

  const auto current_positions = getInput<std::vector<double>>("current_positions");
  if (!current_positions)
  {
    throw BT::RuntimeError("OffsetJointPositions: ", current_positions.error());
  }

  std::vector<double> targets;
  targets.reserve(current_positions.value().size());
  std::transform(current_positions.value().cbegin(), current_positions.value().cend(), std::back_inserter(targets),
                 [offset](double position) { return position + offset.value(); });

  setOutput("target_positions", targets);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace stepit_behaviors
