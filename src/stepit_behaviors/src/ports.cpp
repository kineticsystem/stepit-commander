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

#include "stepit_behaviors/ports.hpp"

#include <utility>

namespace stepit_behaviors
{

std::vector<std::string> getNames(const BT::TreeNode& node, const std::string& port)
{
  // getInput never throws: it reports a missing entry, or a type that does not
  // match, as an error, which is exactly what we fall back on here.
  if (const auto names = node.getInput<std::vector<std::string>>(port))
  {
    return names.value();
  }
  if (const auto name = node.getInput<std::string>(port); name && !name.value().empty())
  {
    return { name.value() };
  }
  return {};
}

std::optional<std::vector<double>> getNumbers(const BT::TreeNode& node, const std::string& port)
{
  if (const auto numbers = node.getInput<std::vector<double>>(port))
  {
    return numbers.value();
  }
  if (const auto number = node.getInput<double>(port))
  {
    return std::vector<double>{ number.value() };
  }
  return std::nullopt;
}

std::vector<double> getNumbersOr(const BT::TreeNode& node, const std::string& port, double fallback)
{
  if (auto numbers = getNumbers(node, port))
  {
    return std::move(numbers.value());
  }

  const auto& ports = node.config().input_ports;
  const auto it = ports.find(port);
  bool is_set = it != ports.end() && !it->second.empty();
  if (is_set)
  {
    if (const auto key = BT::TreeNode::getRemappedKey(port, it->second))
    {
      const auto entry = node.config().blackboard->getEntry(std::string(key.value()));
      is_set = entry && !entry->value.empty();
    }
  }
  if (is_set)
  {
    throw BT::RuntimeError("[", port, "] must be a number, or a list of numbers");
  }
  return { fallback };
}

}  // namespace stepit_behaviors
