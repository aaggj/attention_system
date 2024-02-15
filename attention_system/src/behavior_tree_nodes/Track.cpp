// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "attention_system/behavior_tree_nodes/Track.hpp"
#include "attention_system_msgs/msg/attention_points.hpp"

#include "rclcpp/rclcpp.hpp"

namespace attention_system
{

using namespace std::chrono_literals;

Track::Track(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
:  BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  points_pub_ = node->create_publisher<attention_system_msgs::msg::AttentionPoints>(
    "attention/attention_points", 100);
}

void
Track::halt()
{
}

BT::NodeStatus
Track::tick()
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");

  if (pose.has_value()) {
    auto points = std::make_unique<attention_system_msgs::msg::AttentionPoints>();
    points->instance_id = "track";
    points->lifeness = rclcpp::Duration(0.5s);
    points->time_in_point = rclcpp::Duration(1s);
    geometry_msgs::msg::PointStamped point;
    point.header = pose.value().header;
    point.point = pose.value().pose.position;
    // REMOVE AFTER THE ROBOCUP
    point.point.z = point.point.z - 0.2;
    points->attention_points.push_back(point);

    points_pub_->publish(std::move(points));
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace plansys2_bt_example

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<attention_system::Track>("Track");
}
