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
#include <utility>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>

#include "stepit_server/progress.hpp"

namespace stepit_server
{

/**
 * @brief The status of every node of a running tree, as feedback for a client
 * that shows the execution, like the StepIt Editor.
 *
 * Each feedback message is a JSON object:
 *
 *     {"tree": "<root ...>...</root>", "nodes": {"3": "RUNNING", "4": "FAILURE"},
 *      "progress": {"3": {"done": 2, "total": 11}}}
 *
 * - `tree`, in the first message only: the tree being executed, as written by
 *   BT::WriteTreeToXML, every subtree expanded into a `<BehaviorTree>` of its own
 *   and every node carrying its `_uid`.
 * - `nodes`: the nodes whose status changed since the previous message, by
 *   `_uid`, each with its last status: RUNNING, SUCCESS, FAILURE, SKIPPED, or
 *   HALTED for a node stopped while running, e.g. by a reactive parent. A node
 *   going back to IDLE after it ended, when its parent resets it, keeps the
 *   status it had, so that the client can show how each node ended.
 * - `progress`, only when one changed: the running nodes that implement
 *   ProgressReporter and whose progress changed since the previous message,
 *   by `_uid`. A node reports none before it runs, nor once it ended.
 *
 * The changes, of status or of progress, are sent at most once per period, and
 * always after the last tick. The first message also carries `run`, the number
 * of the run, which tells its snapshots, below, from those of other runs.
 *
 * snapshot() is the whole run in one message, for a client that comes while it
 * runs, or after:
 *
 *     {"run": 7, "objective": "Main", "tree": "<root ...>...</root>",
 *      "nodes": {"1": "RUNNING", "2": "SUCCESS"}, "progress": {"3": {"done": 2, "total": 11}},
 *      "running": true}
 *
 * `nodes` holds every node that has run, with its last status, `progress` every
 * running reporter. A run that ended has `running` false, and `status`, how the
 * tree ended, SUCCESS or FAILURE, `cancelled`, and `message`, e.g. why it was
 * preempted.
 */
class ExecutionStatus : public BT::StatusChangeLogger
{
public:
  using Clock = std::chrono::steady_clock;

  /// @brief How often the changes are sent, at most.
  static constexpr std::chrono::milliseconds kDefaultPeriod{ 50 };

  /// @brief How a run ended, for its last snapshot.
  struct Ending
  {
    BT::NodeStatus status;
    bool cancelled;
    std::string message;
  };

  /**
   * @param run The number of the run, which the server counts: in the first
   * feedback message and in every snapshot.
   */
  explicit ExecutionStatus(const BT::Tree& tree, std::chrono::milliseconds period = kDefaultPeriod,
                           std::uint64_t run = 0);

  /**
   * @brief The feedback to publish after a tick, if any.
   * @param finished Whether the tick ended the tree: the changes are then sent
   * whatever the period.
   */
  std::optional<std::string> feedback(bool finished, Clock::time_point now = Clock::now());

  /**
   * @brief The whole run, see above: every node with its last status, and the
   * progress of the running reporters.
   * @param ending How the run ended, or none while it runs.
   */
  std::string snapshot(const std::optional<Ending>& ending = std::nullopt) const;

  void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                BT::NodeStatus status) override;

  void flush() override
  {
  }

private:
  std::uint64_t run_;
  std::string objective_;
  std::string tree_xml_;
  std::chrono::milliseconds period_;
  bool tree_sent_ = false;
  Clock::time_point last_sent_;
  /// @brief The last status of the nodes that changed, as sent: see above.
  std::map<std::uint16_t, std::string> changes_;
  /// @brief The last status of every node that has run, for the snapshots.
  std::map<std::uint16_t, std::string> statuses_;
  /// @brief The nodes of the tree that report their progress.
  std::vector<std::pair<const BT::TreeNode*, const ProgressReporter*>> reporters_;
  /// @brief The last progress sent of each running reporter.
  std::map<std::uint16_t, std::pair<double, double>> progress_sent_;
};

}  // namespace stepit_server
