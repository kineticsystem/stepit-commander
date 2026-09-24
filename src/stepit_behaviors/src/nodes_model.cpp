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

#include "stepit_behaviors/nodes_model.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>

#include "stepit_behaviors/register_nodes.hpp"

namespace stepit_behaviors
{

namespace
{
constexpr auto kHeader = R"(<?xml version="1.0" encoding="UTF-8"?>
<!--
  The behaviors of stepit_behaviors, for editors that cannot load the plugin.

  Generated: do not edit. After changing a behavior, regenerate it from the
  workspace root with

    ros2 run stepit_behaviors write_nodes_model src/stepit_objectives/objectives/stepit_behaviors.xml
-->
)";
}  // namespace

std::string nodesModel()
{
  BT::BehaviorTreeFactory factory;
  // Registering only reads the ports of each behavior: no ROS node is needed.
  registerNodes(factory, BT::RosNodeParams());
  return kHeader + BT::writeTreeNodesModelXML(factory, false);
}

}  // namespace stepit_behaviors
