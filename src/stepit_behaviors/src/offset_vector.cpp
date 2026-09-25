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
#include "stepit_behaviors/offset_vector.hpp"

#include <string>
#include <vector>

#include "stepit_behaviors/ports.hpp"

namespace stepit_behaviors
{

OffsetVector::OffsetVector(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
{
}

BT::PortsList OffsetVector::providedPorts()
{
  return {
    BT::InputPort<std::vector<double>>("input", "the numbers to offset"),
    BT::InputPort<BT::AnyTypeAllowed>("offset", "a number added to every element, or a list with one per element"),
    BT::OutputPort<std::vector<double>>("output", "the numbers, offset"),
  };
}

BT::NodeStatus OffsetVector::tick()
{
  const auto input = getInput<std::vector<double>>("input");
  if (!input)
  {
    throw BT::RuntimeError("OffsetVector: ", input.error());
  }

  const auto offsets = getNumbers(*this, "offset");
  if (!offsets)
  {
    throw BT::RuntimeError("OffsetVector: [offset] must be a number, or a list of numbers");
  }

  const auto& values = input.value();
  if (offsets->size() != 1 && offsets->size() != values.size())
  {
    throw BT::RuntimeError("OffsetVector: ", std::to_string(offsets->size()), " offsets for ",
                           std::to_string(values.size()),
                           " elements: give one offset for every element, or one per element");
  }

  std::vector<double> output;
  output.reserve(values.size());
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    output.push_back(values[i] + (offsets->size() == 1 ? offsets->front() : (*offsets)[i]));
  }

  setOutput("output", output);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace stepit_behaviors
