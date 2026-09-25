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

#include <cstddef>
#include <string>
#include <vector>

#include <behaviortree_cpp/decorator_node.h>

namespace stepit_behaviors
{

/**
 * @brief Ticks its child once per value, like a for loop.
 *
 * Before each iteration it writes the value on the blackboard, through the
 * output port [value], for the child to use, e.g. as the positions of a
 * TrapezoidalTrajectory. The values are either
 *
 * - evenly spaced from [start] to [end], [count] of them, both ends included:
 *   start and end are a number, or a list with one per joint, and every value is
 *   computed from the start, so no rounding error builds up; or
 * - the list [values], one number per iteration, e.g. unevenly spaced
 *   positions of a single joint.
 *
 * The value is always a list, of one element for a single joint, as the ports
 * of the trajectories expect. Steps succeeds after the last iteration, and
 * fails as soon as its child fails. Every time it runs again, e.g. nested in the
 * child of another Steps, it starts again from the first value.
 *
 * Pure logic: the node knows nothing about joints or motion.
 */
class Steps : public BT::DecoratorNode
{
public:
  Steps(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  void halt() override;

private:
  BT::NodeStatus tick() override;

  /// @brief The values of every iteration, from the ports.
  std::vector<std::vector<double>> readValues() const;

  std::vector<std::vector<double>> values_;
  std::size_t index_ = 0;
  bool started_ = false;
};

}  // namespace stepit_behaviors
