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

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "gb_attention/AttentionServerNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace gb_attention
{

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


AttentionServerNode::AttentionServerNode(const std::string & name)
: CascadeLifecycleNode(name),
  tfBuffer_(nullptr),
	tf_listener_(nullptr),
	current_yaw_(0.0),
	current_pitch_(0.0),
	time_in_point_(1.5),
	time_head_travel_(1.0)
{
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
AttentionServerNode::on_configure(const rclcpp_lifecycle::State & state)
{
	joint_cmd_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/head_controller/command", 100);
	attention_points_sub_ = create_subscription<gb_attention_msgs::msg::AttentionPoints>(
    "/attention/attention_points", 100, std::bind(&AttentionServerNode::attention_point_callback,
    this, _1));

	joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1,
		std::bind(&AttentionServerNode::joint_state_callback, this, _1));

	remove_instance_service_ = create_service<gb_attention_msgs::srv::RemoveAttentionStimuli>(
    "/attention/remove_instances", std::bind(&AttentionServerNode::remove_stimuli_callback,
    this, _1, _2));

	markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/attention_markers", 100);

	time_in_pos_ = now();

  tfBuffer_ = std::make_shared<tf2::BufferCore>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, shared_from_this(), false);

	init_join_state();
  return CascadeLifecycleNode::on_configure(state);
}

CallbackReturnT
AttentionServerNode::on_activate(const rclcpp_lifecycle::State & state)
{
  timer_ = create_wall_timer(100ms, std::bind(&AttentionServerNode::update, this));

  joint_cmd_pub_->on_activate();
  markers_pub_->on_activate();

  return CascadeLifecycleNode::on_activate(state);
}

CallbackReturnT
AttentionServerNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  timer_ = nullptr;

  return CascadeLifecycleNode::on_deactivate(state);
}

void
AttentionServerNode::attention_point_callback(
  const gb_attention_msgs::msg::AttentionPoints::ConstSharedPtr msg)
{
	int point_counter = 0;
	for (const auto & msg_point : msg->attention_points) {
		std::string point_id = msg->class_id + "." + msg->instance_id + "." + std::to_string(point_counter++);

		bool found = false;
		std::list<AttentionPoint>::iterator it = attention_points_.begin();
		while (it != attention_points_.end() && !found) {
			found = it->point_id == point_id;
			if (!found) it++;
		}

		if (found) {
			fromMsg(msg_point, it->point);
		} else {
			AttentionPoint att_point;
			att_point.point_id = point_id;

			if (attention_points_.empty()) {
				att_point.epoch = 0;
      } else {
				att_point.epoch = attention_points_.begin()->epoch;
      }

			fromMsg(msg_point, att_point.point);

      att_point.last_time_in_fovea = now();
			attention_points_.push_back(att_point);
		}
	}
}

void
AttentionServerNode::update_points()
{
	for (auto& point : attention_points_)
  {
		geometry_msgs::msg::TransformStamped p2torso_msg;
		tf2::Transform point2torso;
		tf2::Transform torso2head1;
		tf2::Transform head12head;

		std::string error;
		if (tfBuffer_->canTransform(point.point.frame_id_, "torso_lift_link",
			 tf2::TimePointZero, &error)) {
			p2torso_msg = tfBuffer_->lookupTransform(point.point.frame_id_, "torso_lift_link", tf2::TimePointZero);
    } else {
			RCLCPP_ERROR(get_logger(), "Can't transform %s", error.c_str());
			continue;
		}
		tf2::Stamped<tf2::Transform> aux;
		tf2::convert(p2torso_msg, aux);

		point2torso = aux;

		torso2head1.setOrigin(tf2::Vector3(0.182, 0.0, 0.0));
		torso2head1.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
		head12head.setOrigin(tf2::Vector3(0.005, 0.0, 0.098));
		head12head.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

		tf2::Vector3 point_head_1 = (point2torso * torso2head1).inverse() * point.point;
		tf2::Vector3 point_head_2 = (point2torso * torso2head1 * head12head).inverse() * point.point;

		point.yaw = atan2(point_head_1.y(), point_head_1.x());
		point.pitch = atan2(point_head_1.z(), point_head_1.x());
	}
}

void
AttentionServerNode::remove_stimuli_callback(
    const std::shared_ptr<gb_attention_msgs::srv::RemoveAttentionStimuli::Request> req,
	  std::shared_ptr<gb_attention_msgs::srv::RemoveAttentionStimuli::Response> res)
{
	remove_points(req->class_id, req->instance_id);
}

void
AttentionServerNode::joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  for (int i = 0; i < msg->name.size(); i++)
  {
		if (msg->name[i] == "head_1_joint")
			current_yaw_ = msg->position[i];
		if (msg->name[i] == "head_2_joint")
			current_pitch_ = msg->position[i];
	}
}


void
AttentionServerNode::remove_points(const std::string & class_id, const std::string & instance_id)
{
	std::string point_id = class_id + "." + instance_id;

	auto it = attention_points_.begin();
	while (it != attention_points_.end()) {
		if (it->point_id.rfind(point_id, 0) == 0) { // If it->point_id starts with point_id
			it = attention_points_.erase(it);
    } else {
			++it;
    }
	}
}

void
AttentionServerNode::init_join_state()
{
  //joint_cmd_.header.stamp = now();
  joint_cmd_.joint_names.resize(2);
  joint_cmd_.points.resize(1);

  joint_cmd_.joint_names[0] ="head_1_joint";
  joint_cmd_.joint_names[1] ="head_2_joint";

  joint_cmd_.points[0].positions.resize(2);
  joint_cmd_.points[0].velocities.resize(2);
  joint_cmd_.points[0].accelerations.resize(2);
  joint_cmd_.points[0].time_from_start = rclcpp::Duration(time_head_travel_);

  joint_cmd_.points[0].positions[0] = 0.0;
  joint_cmd_.points[0].positions[1] = 0.0;
  joint_cmd_.points[0].velocities[0] = NECK_SPEED;
  joint_cmd_.points[0].velocities[1] = NECK_SPEED;
  joint_cmd_.points[0].accelerations[0] = NECK_SPEED;
  joint_cmd_.points[0].accelerations[1] = NECK_SPEED;
}


void
AttentionServerNode::publish_markers()
{
	if (markers_pub_->get_subscription_count() == 0) {
		return;
  }

	tf2::Stamped<tf2::Vector3> end_point = attention_points_.begin()->point;


	visualization_msgs::msg::MarkerArray msg;
	visualization_msgs::msg::Marker att_marker;

	att_marker.header.frame_id = "xtion_link";
	att_marker.header.stamp = now();
	att_marker.ns = get_name();
	att_marker.id = 0;
	att_marker.type = visualization_msgs::msg::Marker::ARROW;
	att_marker.action = visualization_msgs::msg::Marker::ADD;
	att_marker.scale.x = 0.01;
	att_marker.scale.y = 0.1;
	att_marker.scale.z = 0.1;
	att_marker.color.b = 0;
	att_marker.color.g = 0;
	att_marker.color.r = 255;
	att_marker.color.a = 1.0;
	att_marker.lifetime = rclcpp::Duration(5.0s);

	geometry_msgs::msg::Point start, end;
	start.x = 0.0;
	start.y = 0.0;
	start.z = 0.0;

	geometry_msgs::msg::TransformStamped tf_msg;
	std::string error;
	if (tfBuffer_->canTransform(end_point.frame_id_, "xtion_link",
		tf2::TimePointZero, &error))
  {
		tf_msg = tfBuffer_->lookupTransform(end_point.frame_id_, "xtion_link", tf2::TimePointZero);
	} else {
		RCLCPP_ERROR(get_logger(), "Can't transform %s", error.c_str());
	}

	tf2::Transform tf;
	tf2::Stamped<tf2::Transform> aux;
	tf2::convert(tf_msg, aux);
	tf = aux;

	tf2::Vector3 point_end = tf.inverse() *  end_point;

	end.x = point_end.x();
	end.y = point_end.y();
	end.z = point_end.z();

	att_marker.points.push_back(start);
	att_marker.points.push_back(end);


	msg.markers.push_back(att_marker);

	markers_pub_->publish(msg);
}

void
AttentionServerNode::print()
{
	RCLCPP_INFO(get_logger(), "===============================");
	for (auto points : attention_points_)
  {
		RCLCPP_INFO(get_logger(), "[%d] [%s]", points.epoch, points.point_id.c_str());
		RCLCPP_INFO(get_logger(), "\t[%s] (%lf, %lf, %lf) [%lf, %lf]", points.point.frame_id_.c_str(),
			points.point.x(), points.point.y(), points.point.z(), points.yaw, points.pitch);
	}
}

void
AttentionServerNode::update_time_in_fovea() {
  //std::cerr << "=================" << std::endl;
  for (auto & point : attention_points_) {
		geometry_msgs::msg::TransformStamped tf_msg;
	  std::string error;
	  if (tfBuffer_->canTransform(point.point.frame_id_, "xtion_link", tf2::TimePointZero, &error)) {
		  tf_msg = tfBuffer_->lookupTransform(point.point.frame_id_, "xtion_link", tf2::TimePointZero);
	  } else {
		  RCLCPP_ERROR(get_logger(), "Can't transform %s", error.c_str());
	  }

	  tf2::Transform tf;
	  tf2::Stamped<tf2::Transform> aux;
	  tf2::convert(tf_msg, aux);
	  tf = aux;

	  tf2::Vector3 point_camera = tf.inverse() *  point.point;

    double angle_y = atan2(point_camera.z(), point_camera.x());
    double angle_x = atan2(point_camera.y(), point_camera.x());

    if (fabs(angle_x) < FOVEA_YAW && fabs(angle_y) < FOVEA_PITCH) {
      auto elapsed = (now() - point.last_time_in_fovea).seconds();
      if (elapsed > 1.0) {
        std::cerr << "Time since fovea [" << point.point_id << "] = " << elapsed << " secs \t[" << angle_x << ", " << angle_y << "]" << std::endl;
      }
      point.last_time_in_fovea = now();
    }
	}

}


inline
void fromMsg(const geometry_msgs::msg::PointStamped & in, tf2::Stamped<tf2::Vector3> & out)
{
  out.stamp_ = tf2_ros::fromMsg(in.header.stamp);
  out.frame_id_ = in.header.frame_id;
  out.setX(in.point.x);
  out.setY(in.point.y);
  out.setZ(in.point.z);
}


};  // namespace gb_attention
