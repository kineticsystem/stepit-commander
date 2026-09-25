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

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>

namespace stepit_server
{

/**
 * @brief The status of every node of a running tree, as feedback for a client
 * that shows the execution, like the StepIt Editor.
 *
 * Each feedback message is a JSON object:
 *
 *     {"tree": "<root ...>...</root>", "nodes": {"3": "RUNNING", "4": "FAILURE"}}
 *
 * - `tree`, in the first message only: the tree being executed, as written by
 *   BT::WriteTreeToXML, every subtree expanded into a `<BehaviorTree>` of its own
 *   and every node carrying its `_uid`.
 * - `nodes`: the nodes whose status changed since the previous message, by
 *   `_uid`, each with its last status: RUNNING, SUCCESS, FAILURE or SKIPPED. A
 *   node going back to IDLE, e.g. when its parent completes, keeps the status
 *   it had, so that the client can show how each node ended.
 *
 * The changes are sent at most once per period, and always after the last tick.
 */
class ExecutionStatus : public BT::StatusChangeLogger
{
public:
  using Clock = std::chrono::steady_clock;

  explicit ExecutionStatus(const BT::Tree& tree, std::chrono::milliseconds period = std::chrono::milliseconds{ 50 });

  /**
   * @brief The feedback to publish after a tick, if any.
   * @param finished Whether the tick ended the tree: the changes are then sent
   * whatever the period.
   */
  std::optional<std::string> feedback(bool finished, Clock::time_point now = Clock::now());

  void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                BT::NodeStatus status) override;

  void flush() override
  {
  }

private:
  std::string tree_xml_;
  std::chrono::milliseconds period_;
  bool tree_sent_ = false;
  Clock::time_point last_sent_;
  std::map<std::uint16_t, BT::NodeStatus> changes_;
};

}  // namespace stepit_server
