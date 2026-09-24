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

#include <algorithm>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>

namespace stepit_tests
{

/**
 * @brief A stand-in for the controller manager of the robot.
 *
 * It answers list_controllers and switch_controller, keeps the state of each
 * controller, and records the last switch it was asked for.
 */
class FakeControllerManager
{
public:
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using SwitchController = controller_manager_msgs::srv::SwitchController;

  /// @brief A controller, as the controller manager sees it.
  struct Controller
  {
    std::string name;
    /// @brief "active" or "inactive".
    std::string state;
    /// @brief False for a broadcaster, which only reads the state of the robot.
    bool owns_command_interfaces;
  };

  explicit FakeControllerManager(std::vector<Controller> controllers)
    : node_{ std::make_shared<rclcpp::Node>("fake_controller_manager") }, controllers_{ std::move(controllers) }
  {
    list_service_ = node_->create_service<ListControllers>("/controller_manager/list_controllers",
                                                           [this](const ListControllers::Request::SharedPtr&,
                                                                  const ListControllers::Response::SharedPtr response) {
                                                             list(response);
                                                           });

    switch_service_ = node_->create_service<SwitchController>(
        "/controller_manager/switch_controller",
        [this](const SwitchController::Request::SharedPtr request,
               const SwitchController::Response::SharedPtr response) { switchControllers(request, response); });

    executor_.add_node(node_);
    spinner_ = std::thread{ [this]() { executor_.spin(); } };
  }

  ~FakeControllerManager()
  {
    executor_.cancel();
    if (spinner_.joinable())
    {
      spinner_.join();
    }
    executor_.remove_node(node_);
  }

  FakeControllerManager(const FakeControllerManager&) = delete;
  FakeControllerManager& operator=(const FakeControllerManager&) = delete;

  /// @brief The last switch the controller manager was asked for, if any.
  std::optional<SwitchController::Request> lastSwitch() const
  {
    const std::lock_guard<std::mutex> lock{ mutex_ };
    return last_switch_;
  }

  /// @brief The state of a controller: "active", "inactive" or "" if unknown.
  std::string stateOf(const std::string& name) const
  {
    const std::lock_guard<std::mutex> lock{ mutex_ };
    const auto it =
        std::find_if(controllers_.cbegin(), controllers_.cend(), [&name](const auto& c) { return c.name == name; });
    return it == controllers_.cend() ? "" : it->state;
  }

private:
  void list(const ListControllers::Response::SharedPtr& response)
  {
    const std::lock_guard<std::mutex> lock{ mutex_ };
    for (const auto& controller : controllers_)
    {
      controller_manager_msgs::msg::ControllerState state;
      state.name = controller.name;
      state.state = controller.state;
      if (controller.owns_command_interfaces && controller.state == "active")
      {
        state.claimed_interfaces = { controller.name + "/joint1/position" };
      }
      response->controller.push_back(state);
    }
  }

  void switchControllers(const SwitchController::Request::SharedPtr& request,
                         const SwitchController::Response::SharedPtr& response)
  {
    const std::lock_guard<std::mutex> lock{ mutex_ };
    last_switch_ = *request;

    const auto find = [this](const std::string& name) {
      return std::find_if(controllers_.begin(), controllers_.end(), [&name](const auto& c) { return c.name == name; });
    };

    // An unknown controller is an error, whatever the strictness is.
    for (const auto& names : { request->activate_controllers, request->deactivate_controllers })
    {
      for (const auto& name : names)
      {
        if (find(name) == controllers_.end())
        {
          response->ok = false;
          response->message = "controller '" + name + "' is not loaded";
          return;
        }
      }
    }

    for (const auto& name : request->deactivate_controllers)
    {
      find(name)->state = "inactive";
    }
    for (const auto& name : request->activate_controllers)
    {
      find(name)->state = "active";
    }

    response->ok = true;
    response->message = "Successfully switched controllers!";
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<ListControllers>::SharedPtr list_service_;
  rclcpp::Service<SwitchController>::SharedPtr switch_service_;

  mutable std::mutex mutex_;
  std::vector<Controller> controllers_;
  std::optional<SwitchController::Request> last_switch_;

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
};

}  // namespace stepit_tests
