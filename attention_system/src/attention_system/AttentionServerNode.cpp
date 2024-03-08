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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
// #include "attention_system_msgs/msg/pan_tilt_command.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "attention_system/AttentionServerNode.hpp"


#include "rclcpp/rclcpp.hpp"

namespace attention_system
{

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


AttentionServerNode::AttentionServerNode(const std::string & name)
: CascadeLifecycleNode(name),
  tfBuffer_(nullptr),
  tf_listener_(nullptr),
  current_yaw_(0.0),
  current_pitch_(0.0)
{
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
AttentionServerNode::on_configure(const rclcpp_lifecycle::State & state)
{
  joint_cmd_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/head_controller/joint_trajectory", 100);
  // comm_pub_ = create_publisher<attention_system_msgs::msg::PanTiltCommand>(
  //   "/command", 100);

  action_client_ = rclcpp_action::create_client
    <control_msgs::action::FollowJointTrajectory>(
    this,
    "/head_controller/follow_joint_trajectory");

 if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Action server not available after waiting 10s");
    }    

  attention_points_sub_ = create_subscription<attention_system_msgs::msg::AttentionPoints>(
    "attention/attention_points", 100, std::bind(
      &AttentionServerNode::attention_point_callback,
      this, _1));

  // command_sub_ = create_subscription<attention_system_msgs::msg::PanTiltCommand>(
  //   "command", 100, std::bind(&AttentionServerNode::command_callback, this, _1));

  // joint_state_sub_ = create_subscription<
  //   control_msgs::msg::JointTrajectoryControllerState>(
  //     "joint_state", rclcpp::SensorDataQoS(),
  //       std::bind(&AttentionServerNode::joint_state_callback, this, _1));

  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/attention_markers", 100);

  time_in_pos_ = now();

  tfBuffer_ = std::make_shared<tf2::BufferCore>();
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, 
      shared_from_this(), false);

  // server_timeout_ = 1s;
  last_pitch_ = 0.0;
  last_yaw_ = 0.0;

  init_join_state();
  RCLCPP_INFO(get_logger(), "AttentionServerNode configured");
  // node_ = rclcpp::Node::make_shared("action_client");
  // node_ = get_node_base_interface();
  return CascadeLifecycleNode::on_configure(state);
}

CallbackReturnT
AttentionServerNode::on_activate(const rclcpp_lifecycle::State & state)
{
  timer_ = create_wall_timer(500ms, std::bind(&AttentionServerNode::update, this));

  joint_cmd_pub_->on_activate();
  markers_pub_->on_activate();
  // comm_pub_->on_activate();

  RCLCPP_INFO(get_logger(), "AttentionServerNode activated");
  return CascadeLifecycleNode::on_activate(state);
}

CallbackReturnT
AttentionServerNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  trajectory_msgs::msg::JointTrajectory command_msg;
  command_msg.header.stamp = now();
  // command_msg.joint_names = last_state_->joint_names;
  command_msg.points.resize(1);
  command_msg.points[0].positions.resize(2);
  command_msg.points[0].velocities.resize(2);
  command_msg.points[0].accelerations.resize(2);
  command_msg.points[0].positions[0] = 0.0;
  command_msg.points[0].positions[1] = 0.0;
  command_msg.points[0].velocities[0] = 0.1;
  command_msg.points[0].velocities[1] = 0.1;
  command_msg.points[0].accelerations[0] = 0.1;
  command_msg.points[0].accelerations[1] = 0.1;
  command_msg.points[0].time_from_start = rclcpp::Duration(1s);

  joint_cmd_pub_->publish(command_msg);

  joint_cmd_pub_->on_deactivate();
  timer_ = nullptr;
  RCLCPP_INFO(get_logger(), "AttentionServerNode deactivated");

  return CascadeLifecycleNode::on_deactivate(state);
}

void
AttentionServerNode::attention_point_callback(
  const attention_system_msgs::msg::AttentionPoints::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "----------AttentionPoint received via callback-------------");
  int point_counter = 0;
  for (const auto & msg_point : msg->attention_points) {
    std::string point_id = msg->instance_id + "." + std::to_string(point_counter++);

    bool found = false;
    std::list<AttentionPoint>::iterator it = attention_points_.begin();
    while (it != attention_points_.end() && !found) {
      found = it->point_id == point_id;
      if (!found) {it++;}
    }

    if (found) {
      fromMsg(msg_point, it->point);
      it->lifeness = msg->lifeness;
      it->time_in_point = msg->time_in_point;
      it->last_update_ts = now();

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
      att_point.lifeness = msg->lifeness;
      att_point.time_in_point = msg->time_in_point;
      att_point.last_update_ts = now();
      attention_points_.push_back(att_point);
    }
  }
}

void
AttentionServerNode::update_points()
{
  for (auto & point : attention_points_) {
    geometry_msgs::msg::TransformStamped p2torso_msg;
    tf2::Transform point2torso;
    tf2::Transform torso2head1;
    tf2::Transform head12head;

    std::string error;
    if (tfBuffer_->canTransform(
        point.point.frame_id_, "torso_lift_link",
        tf2::TimePointZero, &error))
    {
      p2torso_msg = tfBuffer_->lookupTransform(
        point.point.frame_id_, "torso_lift_link",
        tf2::TimePointZero);
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

// void
// AttentionServerNode::joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
// {
//   for (int i = 0; i < msg->name.size(); i++)
//   {
//              if (msg->name[i] == "head_1_joint")
//                      current_yaw_ = msg->position[i];
//              if (msg->name[i] == "head_2_joint")
//                      current_pitch_ = msg->position[i];
//      }
// }
// void
// AttentionServerNode::joint_state_callback(
//   control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg)
// {
//   last_state_ = std::move(msg);
// }

void
AttentionServerNode::init_join_state()
{
//   joint_cmd_.header.stamp = now();
  joint_cmd_.joint_names.resize(2);
  joint_cmd_.points.resize(1);

  joint_cmd_.joint_names[0] = "head_1_joint";
  joint_cmd_.joint_names[1] = "head_2_joint";

  joint_cmd_.points[0].positions.resize(2);
  joint_cmd_.points[0].velocities.resize(2);
  joint_cmd_.points[0].accelerations.resize(2);
  joint_cmd_.points[0].time_from_start = rclcpp::Duration(1, 0);  // 1 sec

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
    RCLCPP_INFO(get_logger(), "No subscribers to markers");
    return;
  }

  tf2::Stamped<tf2::Vector3> end_point = attention_points_.begin()->point;


  visualization_msgs::msg::MarkerArray msg;
  visualization_msgs::msg::Marker att_marker;

  att_marker.header.frame_id = "head_front_camera_link";
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
  if (tfBuffer_->canTransform(
      end_point.frame_id_, "head_front_camera_link",
      tf2::TimePointZero, &error))
  {
    tf_msg = tfBuffer_->lookupTransform(
      end_point.frame_id_, "head_front_camera_link",
      tf2::TimePointZero);
  } else {
    RCLCPP_ERROR(get_logger(), "Can't transform %s", error.c_str());
  }

  tf2::Transform tf;
  tf2::Stamped<tf2::Transform> aux;
  tf2::convert(tf_msg, aux);
  tf = aux;

  tf2::Vector3 point_end = tf.inverse() * end_point;

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
  for (auto points : attention_points_) {
    RCLCPP_INFO(get_logger(), "[%d] [%s]", points.epoch, points.point_id.c_str());
    RCLCPP_INFO(
      get_logger(), "\t[%s] (%lf, %lf, %lf) [%lf, %lf]", points.point.frame_id_.c_str(),
      points.point.x(), points.point.y(), points.point.z(), points.yaw, points.pitch);
  }
}

void
AttentionServerNode::update_time_in_fovea()
{
  for (auto & point : attention_points_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    std::string error;
    if (tfBuffer_->canTransform(
        point.point.frame_id_, "head_front_camera_link", tf2::TimePointZero,
        &error))
    {
      tf_msg = tfBuffer_->lookupTransform(
        point.point.frame_id_, "head_front_camera_link",
        tf2::TimePointZero);
    } else {
      RCLCPP_ERROR(get_logger(), "Can't transform %s", error.c_str());
    }

    tf2::Transform tf;
    tf2::Stamped<tf2::Transform> aux;
    tf2::convert(tf_msg, aux);
    tf = aux;

    tf2::Vector3 point_camera = tf.inverse() * point.point;

    double angle_y = atan2(point_camera.z(), point_camera.x());
    double angle_x = atan2(point_camera.y(), point_camera.x());

    if (fabs(angle_x) < FOVEA_YAW && fabs(angle_y) < FOVEA_PITCH) {
      auto elapsed = (now() - point.last_time_in_fovea).seconds();
      point.last_time_in_fovea = now();
    }
  }
}

void
AttentionServerNode::remove_expired_points()
{
  auto it = attention_points_.begin();
  while (it != attention_points_.end()) {
    const AttentionPoint & point = *it;

    if ((now() - point.last_update_ts) > point.lifeness) {
      it = attention_points_.erase(it);
    } else {
      ++it;
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


}   // namespace attention_system
