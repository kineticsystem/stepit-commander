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

namespace stepit_behaviors
{

/**
 * @brief Adds an offset to every element of a vector of numbers.
 *
 * The offset is either a single number, added to every element, or a list with
 * one offset per element. Pure arithmetic: the node knows nothing about joints
 * or motion. OffsetJointsBy uses it to turn the current joint positions and the
 * commanded offsets into the positions to reach.
 */
class OffsetVector : public BT::SyncActionNode
{
public:
  OffsetVector(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace stepit_behaviors
