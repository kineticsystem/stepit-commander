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

#include <behaviortree_cpp/contrib/json.hpp>
#include <behaviortree_cpp/xml_parsing.h>

namespace stepit_server
{

ExecutionStatus::ExecutionStatus(const BT::Tree& tree, std::chrono::milliseconds period)
  : BT::StatusChangeLogger(tree.rootNode()), tree_xml_(BT::WriteTreeToXML(tree, true, false)), period_(period)
{
}

void ExecutionStatus::callback(BT::Duration, const BT::TreeNode& node, BT::NodeStatus prev_status,
                               BT::NodeStatus status)
{
  if (status != BT::NodeStatus::IDLE)
  {
    changes_[node.UID()] = BT::toStr(status);
  }
  else if (prev_status == BT::NodeStatus::RUNNING)
  {
    // A node that ends goes through SUCCESS or FAILURE: straight back to IDLE,
    // it was halted.
    changes_[node.UID()] = "HALTED";
  }
}

std::optional<std::string> ExecutionStatus::feedback(bool finished, Clock::time_point now)
{
  if (tree_sent_ && (changes_.empty() || (!finished && now - last_sent_ < period_)))
  {
    return std::nullopt;
  }

  nlohmann::json message;
  if (!tree_sent_)
  {
    message["tree"] = tree_xml_;
    tree_sent_ = true;
  }
  auto& nodes = message["nodes"] = nlohmann::json::object();
  for (const auto& [uid, status] : changes_)
  {
    nodes[std::to_string(uid)] = status;
  }
  changes_.clear();
  last_sent_ = now;
  return message.dump();
}

}  // namespace stepit_server
