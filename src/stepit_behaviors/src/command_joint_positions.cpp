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

#include "stepit_behaviors/command_joint_positions.hpp"

#include <algorithm>
#include <cmath>

#include "stepit_behaviors/ports.hpp"

namespace stepit_behaviors
{

CommandJointPositions::CommandJointPositions(const std::string& name, const BT::NodeConfig& config,
                                             const BT::RosNodeParams& params)
  : BT::StatefulActionNode(name, config), node_(params.nh), logger_(rclcpp::get_logger("CommandJointPositions"))
{
  if (const auto node = node_.lock())
  {
    logger_ = node->get_logger();
  }
  // Connect now, so that a joint state has usually arrived, and the controller
  // has discovered the publisher, by the first tick. A topic name read from the
  // blackboard is only known when ticked.
  connect();
}

BT::PortsList CommandJointPositions::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic_name", "/position_controller/commands",
                               "the commands topic of the position controller"),
    BT::InputPort<std::string>("controller_name", "position_controller",
                               "the position controller, which a halt deactivates to stop the robot"),
    BT::InputPort<std::string>("switch_service", "/controller_manager/switch_controller",
                               "the switch_controller service of the controller manager"),
    BT::InputPort<std::vector<std::string>>("controller_joints",
                                            "every joint of the position controller, in the order of its commands"),
    BT::InputPort<std::vector<std::string>>("joint_names", "joints to move"),
    BT::InputPort<BT::AnyTypeAllowed>("positions",
                                      "target positions, in radians: one for every joint, or one per joint"),
    BT::InputPort<std::string>("joint_states_topic", "/joint_states", "topic the joint states are published on"),
    BT::InputPort<double>("tolerance", 0.01, "how close to its target a joint must be, in radians"),
    BT::InputPort<double>("velocity_tolerance", 0.01, "how slow a joint must be to count as stopped, in rad/s"),
    BT::InputPort<double>("timeout", 60.0, "how long the joints may take to get there, in seconds"),
  };
}

bool CommandJointPositions::connect()
{
  if (subscription_ && publisher_ && switch_client_)
  {
    return true;
  }
  const auto topic_name = getInput<std::string>("topic_name");
  const auto joint_states_topic = getInput<std::string>("joint_states_topic");
  const auto switch_service = getInput<std::string>("switch_service");
  if (!topic_name || !joint_states_topic || !switch_service)
  {
    return false;
  }
  auto node = node_.lock();
  if (!node)
  {
    throw BT::RuntimeError("CommandJointPositions: the ROS node went out of scope");
  }

  // A callback group of our own, spun only when ticked, as the nodes of
  // BehaviorTree.ROS2 do: the tree is ticked from a single thread.
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_, node->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic.value(), rclcpp::QoS{ 1 },
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { last_state_ = msg; }, options);
  publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name.value(), rclcpp::QoS{ 10 });
  switch_client_ = node->create_client<controller_manager_msgs::srv::SwitchController>(
      switch_service.value(), rclcpp::ServicesQoS(), callback_group_);
  return true;
}

BT::NodeStatus CommandJointPositions::onStart()
{
  if (!connect())
  {
    throw BT::RuntimeError(
        "CommandJointPositions: [topic_name], [joint_states_topic] and [switch_service] must be set");
  }
  controller_joints_ = getNames(*this, "controller_joints");
  joints_ = getNames(*this, "joint_names");
  const auto positions = getNumbers(*this, "positions");
  const auto tolerance = getInput<double>("tolerance");
  const auto velocity_tolerance = getInput<double>("velocity_tolerance");
  const auto timeout = getInput<double>("timeout");
  if (controller_joints_.empty())
  {
    throw BT::RuntimeError("CommandJointPositions: [controller_joints] must name the joints of the controller");
  }
  if (joints_.empty())
  {
    throw BT::RuntimeError("CommandJointPositions: [joint_names] must name at least one joint");
  }
  if (!positions)
  {
    throw BT::RuntimeError("CommandJointPositions: [positions] must be a number, or a list of numbers");
  }
  if (!tolerance || !velocity_tolerance || !timeout)
  {
    throw BT::RuntimeError("CommandJointPositions: [tolerance], [velocity_tolerance] and [timeout] must be numbers");
  }
  for (const auto& joint : joints_)
  {
    if (std::find(controller_joints_.cbegin(), controller_joints_.cend(), joint) == controller_joints_.cend())
    {
      throw BT::RuntimeError("CommandJointPositions: joint '", joint, "' is not one of [controller_joints]");
    }
  }
  targets_ = positions.value();
  if (targets_.size() == 1 && joints_.size() > 1)
  {
    targets_.assign(joints_.size(), targets_.front());
  }
  if (targets_.size() != joints_.size())
  {
    throw BT::RuntimeError("CommandJointPositions: [positions] has ", std::to_string(targets_.size()), " values for ",
                           std::to_string(joints_.size()), " joints");
  }

  tolerance_ = tolerance.value();
  velocity_tolerance_ = velocity_tolerance.value();
  timeout_ = timeout.value();
  sent_ = false;
  deadline_ = std::chrono::steady_clock::now() +
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout_));
  return onRunning();
}

BT::NodeStatus CommandJointPositions::onRunning()
{
  executor_.spin_some();

  if (!sent_)
  {
    // The positions of the joints left alone come from the joint state, and a
    // command sent before the controller subscribes is lost: wait for both.
    const auto current = last_state_ ? positionsOf(controller_joints_) : std::nullopt;
    if (last_state_ && !current)
    {
      return BT::NodeStatus::FAILURE;
    }
    if (current && publisher_->get_subscription_count() > 0)
    {
      std_msgs::msg::Float64MultiArray command;
      command.data = current.value();
      for (std::size_t i = 0; i < joints_.size(); ++i)
      {
        const auto it = std::find(controller_joints_.cbegin(), controller_joints_.cend(), joints_[i]);
        command.data[static_cast<std::size_t>(std::distance(controller_joints_.cbegin(), it))] = targets_[i];
      }
      publisher_->publish(command);
      sent_ = true;
    }
  }
  else if (arrived())
  {
    return BT::NodeStatus::SUCCESS;
  }

  if (std::chrono::steady_clock::now() >= deadline_)
  {
    if (!sent_)
    {
      RCLCPP_ERROR(logger_, "%s: no joint state on %s, or no position controller on %s, within %.1f s", name().c_str(),
                   subscription_->get_topic_name(), publisher_->get_topic_name(), timeout_);
    }
    else
    {
      RCLCPP_ERROR(logger_, "%s: the joints did not reach their targets within %.1f s", name().c_str(), timeout_);
      stop();
    }
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void CommandJointPositions::onHalted()
{
  if (sent_)
  {
    stop();
  }
}

std::optional<std::vector<double>> CommandJointPositions::positionsOf(const std::vector<std::string>& joints) const
{
  std::vector<double> positions;
  positions.reserve(joints.size());
  for (const auto& joint : joints)
  {
    const auto it = std::find(last_state_->name.cbegin(), last_state_->name.cend(), joint);
    const auto index = static_cast<std::size_t>(std::distance(last_state_->name.cbegin(), it));
    if (it == last_state_->name.cend() || index >= last_state_->position.size())
    {
      RCLCPP_ERROR(logger_, "%s: no position published for joint '%s' on %s", name().c_str(), joint.c_str(),
                   subscription_->get_topic_name());
      return std::nullopt;
    }
    positions.push_back(last_state_->position[index]);
  }
  return positions;
}

void CommandJointPositions::stop()
{
  const auto controller = getInput<std::string>("controller_name");
  if (!controller)
  {
    RCLCPP_ERROR(logger_, "%s: cannot stop the robot: %s", name().c_str(), controller.error().c_str());
    return;
  }
  if (!switch_client_->service_is_ready())
  {
    RCLCPP_ERROR(logger_, "%s: cannot stop the robot: %s is not available", name().c_str(),
                 switch_client_->get_service_name());
    return;
  }
  // Released, the joints are brought to rest by the hardware. The answer is
  // not awaited: a halt must return at once.
  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->deactivate_controllers = { controller.value() };
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  switch_client_->async_send_request(request);
  RCLCPP_INFO(logger_, "%s: deactivating %s to stop the robot", name().c_str(), controller.value().c_str());
}

bool CommandJointPositions::arrived() const
{
  if (!last_state_)
  {
    return false;
  }
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    const auto it = std::find(last_state_->name.cbegin(), last_state_->name.cend(), joints_[i]);
    const auto index = static_cast<std::size_t>(std::distance(last_state_->name.cbegin(), it));
    if (it == last_state_->name.cend() || index >= last_state_->position.size())
    {
      return false;
    }
    if (std::abs(last_state_->position[index] - targets_[i]) > tolerance_)
    {
      return false;
    }
    // A joint state without velocities cannot tell a moving joint from a
    // stopped one: then being within tolerance is enough.
    if (index < last_state_->velocity.size() && std::abs(last_state_->velocity[index]) > velocity_tolerance_)
    {
      return false;
    }
  }
  return true;
}

}  // namespace stepit_behaviors
