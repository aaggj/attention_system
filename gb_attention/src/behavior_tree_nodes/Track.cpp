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

#include "gb_attention/behavior_tree_nodes/Track.hpp"
#include "gb_attention_msgs/msg/attention_points.hpp"

#include "rclcpp/rclcpp.hpp"

namespace gb_attention
{

using namespace std::chrono_literals;

Track::Track(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
:  BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  points_pub_ = node->create_publisher<gb_attention_msgs::msg::AttentionPoints>(
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

  auto point = getInput<geometry_msgs::msg::PointStamped>("point");

  if (point.has_value()) {
    auto points = std::make_unique<gb_attention_msgs::msg::AttentionPoints>();
    points->instance_id = "track";
    points->lifeness = rclcpp::Duration(0.5s);
    points->time_in_point = rclcpp::Duration(1s);
    points->attention_points.push_back(point.value());

    points_pub_->publish(std::move(points));
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace plansys2_bt_example

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_attention::Track>("Track");
}
