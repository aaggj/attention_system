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

#include "gb_attention/behavior_tree_nodes/Scan.hpp"
#include "gb_attention_msgs/msg/attention_points.hpp"

#include "rclcpp/rclcpp.hpp"

namespace gb_attention
{

using namespace std::chrono_literals;

Scan::Scan(
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
Scan::halt()
{
}

BT::NodeStatus
Scan::tick()
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  auto mode = getInput<std::string>("mode");

  auto points = std::make_unique<gb_attention_msgs::msg::AttentionPoints>();
  points->lifeness = rclcpp::Duration(0.5s);
  points->time_in_point = rclcpp::Duration(3s);
        
  geometry_msgs::msg::PointStamped point;
  point.header.frame_id = "base_footprint";
  point.header.stamp = node->now();
  point.point.z = 0;

  if (mode.has_value()) {
    if (mode.value() == "floor_near") {
        points->instance_id = "floor_near";
        point.point.x = 1.0;
        point.point.y = 0.0;
        points->attention_points.push_back(point);
        point.point.y = 1.0;
        points->attention_points.push_back(point);
        point.point.y = -1.0;
        points->attention_points.push_back(point);
        points_pub_->publish(std::move(points));
    }
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace plansys2_bt_example

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_attention::Scan>("Scan");
}
