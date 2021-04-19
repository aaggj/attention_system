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


#include <string>
#include <list>
#include <vector>
#include <set>

#include "gb_attention_msgs/msg/attention_points.hpp"
#include "gb_attention_msgs/srv/remove_attention_stimuli.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "gb_attention/AttentionClientNode.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace gb_attention
{

AttentionClientNode::AttentionClientNode(const std::string & name)
: CascadeLifecycleNode(name),
	marker_counter_id_(0),
  tfBuffer_(),
  tf_listener_(tfBuffer_)
{
	attention_points_pub_ = create_publisher<gb_attention_msgs::msg::AttentionPoints>(
    "/attention/attention_points", 100);

	remove_instance_service_ = create_client<gb_attention_msgs::srv::RemoveAttentionStimuli>(
		"/attention/remove_instances");

	markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/attention_markers", 100);

  declare_parameter("instances");
  declare_parameter("class_id");
  declare_parameter("default");
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
AttentionClientNode::on_configure(const rclcpp_lifecycle::State & state)
{
  get_parameter("class_id", class_);

  graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(class_ + "_attention_client");
  graph_->start();

	init_attention_points();

  return CascadeLifecycleNode::on_configure(state);
}

CallbackReturnT
AttentionClientNode::on_activate(const rclcpp_lifecycle::State & state)
{
  timer_ = create_wall_timer(200ms, std::bind(&AttentionClientNode::update, this));

  attention_points_pub_->on_activate();
  markers_pub_->on_activate();

  return CascadeLifecycleNode::on_activate(state);
}

CallbackReturnT
AttentionClientNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  timer_ = nullptr;

  return CascadeLifecycleNode::on_deactivate(state);
}

std::vector<std::string> AttentionClientNode::tokenize(const std::string & text)
{
  std::vector<std::string> ret;
  size_t start = 0, end = 0;

  while (end != std::string::npos) {
    end = text.find(",", start);
    ret.push_back(text.substr(start, (end == std::string::npos) ? std::string::npos : end - start));
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }
  return ret;
}

void
AttentionClientNode::init_attention_points()
{
	std::vector<std::string> instances;
  get_parameter_or("instances", instances, {});

	for (const auto & instance : instances) {
		std::list<geometry_msgs::msg::PointStamped> instance_coords;

		std::vector<std::string> coords;
    if (!has_parameter(instance)) {
      declare_parameter(instance);
    }

		get_parameter_or(instance, coords, {});

		for (const auto & coord : coords)
    {
			geometry_msgs::msg::PointStamped p;
			std::vector<std::string> coord_str = tokenize(coord);

      if (coord_str.size() != 3) {
        RCLCPP_ERROR_STREAM(get_logger(), "Instance [" << instance << "]: incorrect coordinates from ["
          << coord << "](" << coord_str.size() << ")");
        continue;
      }

			p.point.x = std::stod(coord_str[0]);
			p.point.y = std::stod(coord_str[1]);
			p.point.z = std::stod(coord_str[2]);

			instance_coords.push_back(p);
		}

		attention_points_[instance] = instance_coords;
	}
}

void
AttentionClientNode::publish_markers(const std::list<geometry_msgs::msg::PointStamped> & points)
{
	if (markers_pub_->get_subscription_count() == 0)
		return;

	visualization_msgs::msg::MarkerArray msg;

	for (auto point : points)
  {
		visualization_msgs::msg::Marker att_marker;

		att_marker.header.frame_id = point.header.frame_id;
		att_marker.header.stamp = point.header.stamp;
		att_marker.ns = get_name();
		att_marker.id = marker_counter_id_++;
		att_marker.type = visualization_msgs::msg::Marker::SPHERE;
		att_marker.action = visualization_msgs::msg::Marker::ADD;
		att_marker.pose.position.x = point.point.x;
		att_marker.pose.position.y = point.point.y;
		att_marker.pose.position.z = point.point.z;
		att_marker.pose.orientation.x = 0.0;
		att_marker.pose.orientation.y = 0.0;
		att_marker.pose.orientation.z = 0.0;
		att_marker.pose.orientation.w = 1.0;
		att_marker.scale.x = 0.1;
		att_marker.scale.y = 0.1;
		att_marker.scale.z = 0.1;
		att_marker.color.b = 0;
		att_marker.color.g = 255.0;
		att_marker.color.r = 0.0;
		att_marker.color.a = 0.6;
		att_marker.lifetime = rclcpp::Duration(1.0s);

		msg.markers.push_back(att_marker);
	}

	markers_pub_->publish(msg);
}

void
AttentionClientNode::update()
{
	const auto & candidates = graph_->get_edges_by_data("want_see", "symbolic");
	std::set<std::string> instances_aux = instances_sent_;

  std::cerr << "candidates: " << candidates.size() << std::endl;
	static bool inited = false;

	for (const auto & edge : candidates) {
		bool exist_tf = true;
    try {
      tfBuffer_.lookupTransform(edge.source, edge.target, tf2::TimePointZero);
    } catch (std::exception & e) {
      exist_tf = false;
    }

		if (graph_->get_node(edge.source).value().type == "robot" &&
				graph_->get_node(edge.target).value().type == class_ &&
				exist_tf)
	  {
      std::cerr << "getting points to: " << edge.target << std::endl;
			gb_attention_msgs::msg::AttentionPoints msg;
			msg.class_id = class_;
			msg.instance_id = edge.target;

			std::list<geometry_msgs::msg::PointStamped> points = get_attention_points(edge);

			publish_markers(points);

			for (auto point : points) {
				msg.attention_points.push_back(point);
      }

			attention_points_pub_->publish(msg);
			instances_sent_.insert(edge.target);

			auto it = instances_aux.find(edge.target);
			if (it != instances_aux.end()) {
				instances_aux.erase(it);
      }
		}
	}

	for (const auto & instance : instances_aux) {
		auto request = std::make_shared<gb_attention_msgs::srv::RemoveAttentionStimuli::Request>();
		request->class_id = class_;
		request->instance_id = instance;

    auto result = remove_instance_service_->async_send_request(request);
    
    result.wait_for(3s);

    if (!result.valid()) {
      RCLCPP_ERROR(get_logger(), "/attention/remove_instances");
    }

		auto it = instances_sent_.find(instance);
		if (it != instances_sent_.end()) {
			instances_sent_.erase(it);
    }
	}
}

std::list<geometry_msgs::msg::PointStamped>
AttentionClientNode::get_attention_points(const ros2_knowledge_graph::Edge & edge)
{
  std::list<geometry_msgs::msg::PointStamped> ret;

	if (attention_points_.find (edge.target) != attention_points_.end()) {
		ret = attention_points_[edge.target];
	} else {
		ret = attention_points_["default"];
	}

	for (auto & point : ret) {
		point.header.stamp = now();
		point.header.frame_id = edge.target;
	}

	return ret;
}

};  // namespace gb_attention
