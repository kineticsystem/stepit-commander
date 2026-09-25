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
#include "stepit_behaviors/steps.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "stepit_behaviors/ports.hpp"

namespace stepit_behaviors
{
namespace
{

/// @brief Whether the XML gives the port, even if its blackboard entry is missing.
bool isGiven(const BT::TreeNode& node, const std::string& port)
{
  const auto& ports = node.config().input_ports;
  const auto it = ports.find(port);
  return it != ports.end() && !it->second.empty();
}

std::vector<double> requireNumbers(const BT::TreeNode& node, const std::string& port)
{
  auto numbers = getNumbers(node, port);
  if (!numbers || numbers->empty())
  {
    throw BT::RuntimeError("Steps: [", port, "] must be a number, or a list of numbers");
  }
  return std::move(numbers.value());
}

/**
 * @brief An input that can be left out: Steps takes either [values] or [start],
 * [end] and [count]. The empty default tells editors that the port is optional,
 * and still counts as not given for isGiven.
 */
BT::PortsList::value_type optionalInput(const std::string& name, const std::string& description)
{
  auto port = BT::InputPort<BT::AnyTypeAllowed>(name, description);
  port.second.setDefaultValue(std::string());
  return port;
}

/// @brief The [count] port: a whole number of at least 1, as a payload gives it, e.g. 5.0.
std::size_t requireCount(const BT::TreeNode& node)
{
  const auto numbers = getNumbers(node, "count");
  if (!numbers || numbers->size() != 1)
  {
    throw BT::RuntimeError("Steps: [count] must be a number");
  }
  const double count = numbers->front();
  if (count < 1.0 || count != std::floor(count))
  {
    throw BT::RuntimeError("Steps: [count] must be a whole number of at least 1, not ", std::to_string(count));
  }
  return static_cast<std::size_t>(count);
}

}  // namespace

Steps::Steps(const std::string& name, const BT::NodeConfig& config) : BT::DecoratorNode(name, config)
{
}

BT::PortsList Steps::providedPorts()
{
  return {
    optionalInput("start", "the first value: a number, or a list with one per joint"),
    optionalInput("end", "the last value, of the same length as the first"),
    optionalInput("count", "how many values from start to end, both included"),
    optionalInput("values", "instead of start, end and count: the values, one number per iteration"),
    BT::OutputPort<std::vector<double>>("value", "the value of this iteration, always a list"),
    BT::OutputPort<int>("index", "the number of this iteration, from 0"),
  };
}

std::vector<std::vector<double>> Steps::readValues() const
{
  const bool range = isGiven(*this, "start") || isGiven(*this, "end") || isGiven(*this, "count");
  if (isGiven(*this, "values"))
  {
    if (range)
    {
      throw BT::RuntimeError("Steps: give either [values], or [start], [end] and [count], not both");
    }
    std::vector<std::vector<double>> values;
    for (const double value : requireNumbers(*this, "values"))
    {
      values.push_back({ value });
    }
    return values;
  }

  const auto start = requireNumbers(*this, "start");
  const auto end = requireNumbers(*this, "end");
  if (start.size() != end.size())
  {
    throw BT::RuntimeError("Steps: [start] has ", std::to_string(start.size()), " values but [end] has ",
                           std::to_string(end.size()), ": give one per joint in both");
  }
  const std::size_t count = requireCount(*this);

  std::vector<std::vector<double>> values;
  values.reserve(count);
  for (std::size_t i = 0; i < count; ++i)
  {
    if (i + 1 == count && count > 1)
    {
      // Exactly the end, whatever the rounding of the division.
      values.push_back(end);
      continue;
    }
    std::vector<double> value;
    value.reserve(start.size());
    for (std::size_t j = 0; j < start.size(); ++j)
    {
      value.push_back(count == 1 ? start[j] :
                                   start[j] + (end[j] - start[j]) * static_cast<double>(i) /
                                                  static_cast<double>(count - 1));
    }
    values.push_back(std::move(value));
  }
  return values;
}

BT::NodeStatus Steps::tick()
{
  if (!started_)
  {
    values_ = readValues();
    index_ = 0;
    started_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);
  while (index_ < values_.size())
  {
    const BT::NodeStatus previous = child_node_->status();
    if (previous == BT::NodeStatus::IDLE)
    {
      setOutput("value", values_[index_]);
      setOutput("index", static_cast<int>(index_));
    }

    switch (child_node_->executeTick())
    {
      case BT::NodeStatus::SUCCESS:
        ++index_;
        resetChild();
        // Like Repeat: give the flow back between iterations of an asynchronous
        // child, so that the tree can be halted between two steps.
        if (requiresWakeUp() && previous == BT::NodeStatus::IDLE && index_ < values_.size())
        {
          emitWakeUpSignal();
          return BT::NodeStatus::RUNNING;
        }
        break;
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;
      case BT::NodeStatus::FAILURE:
        started_ = false;
        resetChild();
        return BT::NodeStatus::FAILURE;
      case BT::NodeStatus::SKIPPED:
        started_ = false;
        resetChild();
        return BT::NodeStatus::SKIPPED;
      case BT::NodeStatus::IDLE:
        throw BT::LogicError("Steps: the child returned IDLE");
    }
  }

  started_ = false;
  return BT::NodeStatus::SUCCESS;
}

void Steps::halt()
{
  started_ = false;
  DecoratorNode::halt();
}

}  // namespace stepit_behaviors
