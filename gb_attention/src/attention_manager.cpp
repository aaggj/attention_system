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

#include <tf2_ros/static_transform_broadcaster.h>

#include <string>

#include "lifecycle_msgs/msg/transition.hpp"

#include "gb_attention/AttentionClientNode.hpp"
#include "gb_attention/OptimizedAttentionServerNode.hpp"
#include "gb_attention/RoundRobinAttentionServerNode.hpp"
#include "gb_attention/SimpleAttentionServerNode.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto manager_node = rclcpp::Node::make_shared("attention_manager");
  manager_node->declare_parameter("clients");
  manager_node->declare_parameter("robot_name");
  manager_node->declare_parameter("strategy");

  std::string strategy;
  manager_node->get_parameter_or("strategy", strategy, strategy);

  RCLCPP_INFO_STREAM(manager_node->get_logger(), "Attention stratey: " << strategy);
  gb_attention::AttentionServerNode::SharedPtr attention_server;
  if (strategy == "optimized") {
     attention_server = std::make_shared<gb_attention::OptimizedAttentionServerNode>();
  } else if (strategy == "roundrobin") {
     attention_server = std::make_shared<gb_attention::RoundRobinAttentionServerNode>();
  } else if (strategy == "simple") {
    attention_server = std::make_shared<gb_attention::SimpleAttentionServerNode>();
  } else {
    RCLCPP_ERROR_STREAM(manager_node->get_logger(), "Strategy not set");
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor exe;

  exe.add_node(manager_node);
  exe.add_node(attention_server->get_node_base_interface());

  std::vector<std::string> clients;
  manager_node->get_parameter_or("clients", clients, clients);

  std::string robot_name;
  manager_node->get_parameter_or("robot_name", robot_name, robot_name);

  if (robot_name == "") {
    RCLCPP_ERROR_STREAM(manager_node->get_logger(), "robot_name not set");
    return 1;
  }

  tf2_ros::StaticTransformBroadcaster tf_broadcaster(manager_node);
  geometry_msgs::msg::TransformStamped msg;
  msg.header.frame_id = "base_footprint";
  msg.header.stamp = manager_node->now();
  msg.child_frame_id = robot_name;
  msg.transform.rotation.w = 1.0;
  tf_broadcaster.sendTransform(msg);

  RCLCPP_INFO_STREAM(manager_node->get_logger(), "Number of attention clients:" << clients.size());
  std::list<gb_attention::AttentionClientNode::SharedPtr> attention_clienst_nodes;
  for (const auto & client : clients) {
    RCLCPP_INFO_STREAM(manager_node->get_logger(), "Starting [" << client <<  "] client");
    auto node = std::make_shared<gb_attention::AttentionClientNode>(client);
    attention_clienst_nodes.push_back(node);
    exe.add_node(node->get_node_base_interface());
  }

  for (auto & client_node : attention_clienst_nodes) {
    client_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  }
  attention_server->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  {
    rclcpp::Rate rate(10);
    auto start = manager_node->now();
    while ((manager_node->now() - start).seconds() < 0.5) {
      exe.spin_some();
      rate.sleep();
    }
  }

  for (auto & client_node : attention_clienst_nodes) {
    client_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  }
  attention_server->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  exe.spin();

  return 0;
 }
