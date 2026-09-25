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

#include <optional>
#include <string>
#include <vector>

#include <behaviortree_cpp/tree_node.h>

namespace stepit_behaviors
{

/**
 * @brief Read a port that holds either a list of names or a single name.
 *
 * A command carrying one controller can then be written as
 * `{controllers: velocity_controller}` as well as
 * `{controllers: [velocity_controller]}`.
 *
 * @return The names, empty if the port is not set.
 */
std::vector<std::string> getNames(const BT::TreeNode& node, const std::string& port);

/**
 * @brief Read a port that holds either a list of numbers or a single number.
 *
 * The port has to be declared with BT::AnyTypeAllowed: a command writes
 * `{offset: -6.28}` as a double and `{offset: [-6.28, 3.14]}` as a list of
 * doubles, and a typed port would refuse one of the two when the tree is
 * created. In the XML, a literal is a list separated by `;`, e.g.
 * `offset="-6.28;3.14"`.
 *
 * @return The numbers, a single one for a single number, or nothing if the
 * port is not set or holds something else.
 */
std::optional<std::vector<double>> getNumbers(const BT::TreeNode& node, const std::string& port);

/**
 * @brief Read a port like getNumbers, with a default for when it is not set.
 *
 * The port is not set when the XML does not give it, or when it points at a
 * blackboard entry that does not exist, e.g. an optional parameter missing from
 * the payload.
 *
 * @return The numbers, or `{ fallback }` if the port is not set.
 * @throws BT::RuntimeError if the port is set to something else than numbers.
 */
std::vector<double> getNumbersOr(const BT::TreeNode& node, const std::string& port, double fallback);

}  // namespace stepit_behaviors
