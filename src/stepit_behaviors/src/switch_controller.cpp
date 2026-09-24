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

#include "stepit_behaviors/switch_controller.hpp"

#include <algorithm>
#include <cctype>

#include "stepit_behaviors/ports.hpp"

namespace stepit_behaviors
{
namespace
{
using Srv = controller_manager_msgs::srv::SwitchController;

std::string lowercase(const std::string& text)
{
  std::string out;
  out.reserve(text.size());
  std::transform(text.cbegin(), text.cend(), std::back_inserter(out),
                 [](char c) { return static_cast<char>(std::tolower(static_cast<unsigned char>(c))); });
  return out;
}

std::string join(const std::vector<std::string>& names)
{
  std::string out;
  for (const auto& name : names)
  {
    if (!out.empty())
    {
      out += ", ";
    }
    out += name;
  }
  return out.empty() ? "none" : out;
}
}  // namespace

std::optional<std::int32_t> parseStrictness(const std::string& text)
{
  const auto word = lowercase(text);
  if (word == "best_effort")
  {
    return Srv::Request::BEST_EFFORT;
  }
  if (word == "strict")
  {
    return Srv::Request::STRICT;
  }
  if (word == "auto")
  {
    return Srv::Request::AUTO;
  }
  if (word == "force_auto")
  {
    return Srv::Request::FORCE_AUTO;
  }
  return std::nullopt;
}

SwitchController::SwitchController(const std::string& name, const BT::NodeConfig& config,
                                   const BT::RosNodeParams& params)
  : BT::RosServiceNode<controller_manager_msgs::srv::SwitchController>(name, config, params)
{
}

BT::PortsList SwitchController::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::vector<std::string>>("activate", "controllers to activate"),
      BT::InputPort<std::vector<std::string>>("deactivate", "controllers to deactivate"),
      BT::InputPort<std::string>("strictness", "best_effort", "best_effort, strict, auto or force_auto"),
  });
}

bool SwitchController::setRequest(Request::SharedPtr& request)
{
  const auto activate = getNames(*this, "activate");
  const auto deactivate = getNames(*this, "deactivate");

  if (activate.empty() && deactivate.empty())
  {
    throw BT::RuntimeError("SwitchController: no controller to activate or deactivate");
  }

  const auto strictness_text = getInput<std::string>("strictness").value_or("best_effort");
  const auto strictness = parseStrictness(strictness_text);
  if (!strictness)
  {
    throw BT::RuntimeError("SwitchController: unknown strictness '", strictness_text,
                           "': expected best_effort, strict, auto or force_auto");
  }

  request->activate_controllers = activate;
  request->deactivate_controllers = deactivate;
  request->strictness = strictness.value();
  request->activate_asap = false;
  // Zero means the controller manager waits as long as it takes.
  request->timeout = rclcpp::Duration::from_seconds(0.0);

  RCLCPP_INFO(logger(), "%s: activating [%s], deactivating [%s]", name().c_str(), join(activate).c_str(),
              join(deactivate).c_str());

  return true;
}

BT::NodeStatus SwitchController::onResponseReceived(const Response::SharedPtr& response)
{
  if (!response->ok)
  {
    RCLCPP_ERROR(logger(), "%s: the controllers were not switched: %s", name().c_str(), response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(logger(), "%s: %s", name().c_str(), response->message.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SwitchController::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace stepit_behaviors
