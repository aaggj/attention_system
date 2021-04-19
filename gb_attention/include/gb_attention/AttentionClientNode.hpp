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

#ifndef GB_ATTENTION_ATTENTIONCLIENT_H
#define GB_ATTENTION_ATTENTIONCLIENT_H

#include <tf2_ros/transform_listener.h>

#include <string>
#include <list>
#include <set>
#include <map>
#include <vector>

#include "ros2_knowledge_graph/GraphNode.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "gb_attention_msgs/msg/attention_points.hpp"
#include "gb_attention_msgs/srv/remove_attention_stimuli.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace gb_attention
{

class AttentionClientNode : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
	explicit AttentionClientNode(const std::string & name = "attention_client");

	void update();

	virtual std::list<geometry_msgs::msg::PointStamped> get_attention_points(const ros2_knowledge_graph::Edge & edge);

protected:
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

	void init_attention_points();
	std::vector<std::string> tokenize(const std::string & text);
	void publish_markers(const std::list<geometry_msgs::msg::PointStamped> & points);

	std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;

  std::string working_frame_;
  rclcpp_lifecycle::LifecyclePublisher<gb_attention_msgs::msg::AttentionPoints>::SharedPtr attention_points_pub_;
	rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
	rclcpp::Client<gb_attention_msgs::srv::RemoveAttentionStimuli>::SharedPtr remove_instance_service_;
	std::string class_;

	std::set<std::string> instances_sent_;

	std::map<std::string, std::list<geometry_msgs::msg::PointStamped>> attention_points_;

	int marker_counter_id_;

  rclcpp::TimerBase::SharedPtr timer_;

  tf2::BufferCore tfBuffer_;
  tf2_ros::TransformListener tf_listener_;
};

};  // namespace gb_attention

#endif  // GB_ATTENTION_ATTENTIONCLIENT_H
