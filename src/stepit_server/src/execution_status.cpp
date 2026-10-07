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

#include "stepit_server/execution_status.hpp"

#include <string>
#include <utility>

#include <behaviortree_cpp/contrib/json.hpp>
#include <behaviortree_cpp/xml_parsing.h>

namespace stepit_server
{

ExecutionStatus::ExecutionStatus(const BT::Tree& tree, std::chrono::milliseconds period, std::uint64_t run)
  : BT::StatusChangeLogger(tree.rootNode())
  , run_(run)
  , objective_(tree.subtrees.empty() ? "" : tree.subtrees.front()->tree_ID)
  , tree_xml_(BT::WriteTreeToXML(tree, true, false))
  , period_(period)
{
  for (const auto& subtree : tree.subtrees)
  {
    for (const auto& node : subtree->nodes)
    {
      if (const auto* reporter = dynamic_cast<const ProgressReporter*>(node.get()))
      {
        reporters_.emplace_back(node.get(), reporter);
      }
    }
  }
}

void ExecutionStatus::callback(BT::Duration, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status)
{
  if (status != BT::NodeStatus::IDLE)
  {
    changes_[node.UID()] = statuses_[node.UID()] = BT::toStr(status);
  }
  else if (prev_status == BT::NodeStatus::RUNNING)
  {
    // A node that ends goes through SUCCESS or FAILURE: straight back to IDLE,
    // it was halted.
    changes_[node.UID()] = statuses_[node.UID()] = "HALTED";
  }
}

std::string ExecutionStatus::snapshot(const std::optional<Ending>& ending) const
{
  nlohmann::json message;
  message["run"] = run_;
  message["objective"] = objective_;
  message["tree"] = tree_xml_;
  auto& nodes = message["nodes"] = nlohmann::json::object();
  for (const auto& [uid, status] : statuses_)
  {
    nodes[std::to_string(uid)] = status;
  }
  auto& progress = message["progress"] = nlohmann::json::object();
  if (!ending)
  {
    for (const auto& [node, reporter] : reporters_)
    {
      const auto current = node->status() == BT::NodeStatus::RUNNING ? reporter->progress() : std::nullopt;
      if (current)
      {
        progress[std::to_string(node->UID())] = { { "done", current->done }, { "total", current->total } };
      }
    }
  }
  message["running"] = !ending.has_value();
  if (ending)
  {
    message["status"] = BT::toStr(ending->status);
    message["cancelled"] = ending->cancelled;
    message["message"] = ending->message;
  }
  return message.dump();
}

std::optional<std::string> ExecutionStatus::feedback(bool finished, Clock::time_point now)
{
  if (tree_sent_ && !finished && now - last_sent_ < period_)
  {
    return std::nullopt;
  }

  // The progress of the running reporters, where it changed since it was sent.
  nlohmann::json progress = nlohmann::json::object();
  for (const auto& [node, reporter] : reporters_)
  {
    const auto uid = node->UID();
    const auto current = node->status() == BT::NodeStatus::RUNNING ? reporter->progress() : std::nullopt;
    if (!current)
    {
      progress_sent_.erase(uid);
      continue;
    }
    const std::pair<double, double> value{ current->done, current->total };
    const auto sent = progress_sent_.find(uid);
    if (sent == progress_sent_.end() || sent->second != value)
    {
      progress[std::to_string(uid)] = { { "done", value.first }, { "total", value.second } };
      progress_sent_[uid] = value;
    }
  }

  if (tree_sent_ && changes_.empty() && progress.empty())
  {
    return std::nullopt;
  }

  nlohmann::json message;
  if (!tree_sent_)
  {
    message["run"] = run_;
    message["tree"] = tree_xml_;
    tree_sent_ = true;
  }
  auto& nodes = message["nodes"] = nlohmann::json::object();
  for (const auto& [uid, status] : changes_)
  {
    nodes[std::to_string(uid)] = status;
  }
  if (!progress.empty())
  {
    message["progress"] = std::move(progress);
  }
  changes_.clear();
  last_sent_ = now;
  return message.dump();
}

}  // namespace stepit_server
