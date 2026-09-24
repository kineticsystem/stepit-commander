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

#include "stepit_behaviors/get_active_controllers.hpp"

#include <algorithm>

#include "stepit_behaviors/ports.hpp"

namespace stepit_behaviors
{

GetActiveControllers::GetActiveControllers(const std::string& name, const BT::NodeConfig& config,
                                           const BT::RosNodeParams& params)
  : BT::RosServiceNode<controller_manager_msgs::srv::ListControllers>(name, config, params)
{
}

BT::PortsList GetActiveControllers::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::vector<std::string>>("exclude", "controllers to leave out of the result"),
      BT::OutputPort<std::vector<std::string>>("active_controllers", "active controllers that own a command interface"),
  });
}

bool GetActiveControllers::setRequest(Request::SharedPtr& /* request */)
{
  // The service takes no argument.
  return true;
}

BT::NodeStatus GetActiveControllers::onResponseReceived(const Response::SharedPtr& response)
{
  const auto excluded = getNames(*this, "exclude");

  std::vector<std::string> controllers;
  for (const auto& controller : response->controller)
  {
    if (controller.state != "active" || controller.claimed_interfaces.empty())
    {
      continue;
    }
    if (std::find(excluded.cbegin(), excluded.cend(), controller.name) != excluded.cend())
    {
      continue;
    }
    controllers.push_back(controller.name);
  }

  RCLCPP_INFO(logger(), "%s: %zu controller(s) are driving the robot", name().c_str(), controllers.size());

  setOutput("active_controllers", controllers);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetActiveControllers::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace stepit_behaviors
