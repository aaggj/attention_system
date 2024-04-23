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

#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"

#include "attention_system/AttentionServerNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace attention_system
{

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


AttentionServerNode::AttentionServerNode(const rclcpp::NodeOptions & options)
: CascadeLifecycleNode("attention_server", options)
{
  declare_parameter("period", 0.050);
  declare_parameter("max_yaw", max_yaw_);
  declare_parameter("max_pitch", max_pitch_);
  declare_parameter("max_vel_yaw", max_vel_yaw_);
  declare_parameter("max_vel_pitch", max_vel_pitch_);
  declare_parameter("rotation_threshold", 0.436332);  // 25 degrees
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
AttentionServerNode::on_configure(const rclcpp_lifecycle::State & state)
{
  double period;
  get_parameter("period", period);
  period_ = rclcpp::Duration::from_seconds(period);

  get_parameter("max_yaw", max_yaw_);
  get_parameter("max_pitch", max_pitch_);
  get_parameter("max_vel_yaw", max_vel_yaw_);
  get_parameter("max_vel_pitch", max_vel_pitch_);
  get_parameter("rotation_threshold", rotation_threshold_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  joint_cmd_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/head_controller/joint_trajectory", 100);
  joint_sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    "/head_controller/state", rclcpp::SensorDataQoS().reliable(),
    std::bind(&AttentionServerNode::joint_state_callback, this, _1));

  attention_command_sub_ = create_subscription<attention_system_msgs::msg::AttentionCommand>(
    "attention/attention_command", 100, std::bind(
      &AttentionServerNode::attention_command_callback,
      this, _1));

  return CascadeLifecycleNode::on_configure(state);
}

CallbackReturnT
AttentionServerNode::on_activate(const rclcpp_lifecycle::State & state)
{
  auto joint_trajectory_msg = get_command_to_angles(0.0, 0.0, 1s);
  joint_cmd_pub_->publish(joint_trajectory_msg);

  timer_ = create_wall_timer(
    period_.to_chrono<std::chrono::nanoseconds>(), std::bind(&AttentionServerNode::update, this));
  joint_cmd_pub_->on_activate();

  return CascadeLifecycleNode::on_activate(state);
}

CallbackReturnT
AttentionServerNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  auto joint_trajectory_msg = get_command_to_angles(0.0, 0.0, 1s);
  joint_cmd_pub_->publish(joint_trajectory_msg);
  joint_cmd_pub_->on_deactivate();

  timer_ = nullptr;

  return CascadeLifecycleNode::on_deactivate(state);
}

void
AttentionServerNode::attention_command_callback(
  attention_system_msgs::msg::AttentionCommand::SharedPtr msg)
{
  attention_frame_ = msg->frame_id_to_track;
}

std::tuple<double, double, bool>
AttentionServerNode::get_py_from_frame(const std::string & frame)
{
  geometry_msgs::msg::TransformStamped p2torso_msg;
  tf2::Transform point2torso;
  tf2::Transform torso2head1;
  tf2::Transform head12head;

  std::string error;
  if (tf_buffer_->canTransform(frame, "torso_lift_link", tf2::TimePointZero, &error)) {
    p2torso_msg = tf_buffer_->lookupTransform(frame, "torso_lift_link", tf2::TimePointZero);
  } else {
    RCLCPP_ERROR(get_logger(), "Can't transform %s", error.c_str());
    return {0.0, 0.0, false};
  }

  tf2::Stamped<tf2::Transform> aux;
  tf2::convert(p2torso_msg, aux);

  point2torso = aux;

  torso2head1.setOrigin(tf2::Vector3(0.182, 0.0, 0.0));
  torso2head1.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  head12head.setOrigin(tf2::Vector3(0.005, 0.0, 0.098));
  head12head.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  tf2::Transform point_head_1 = (point2torso * torso2head1).inverse();
  tf2::Transform point_head_2 = (point2torso * torso2head1 * head12head).inverse();

  double yaw = atan2(point_head_1.getOrigin().y(), point_head_1.getOrigin().x());
  double pitch = atan2(point_head_1.getOrigin().z(), point_head_1.getOrigin().x());

  yaw = std::clamp(yaw, -max_yaw_, max_yaw_);
  pitch = std::clamp(pitch, -max_pitch_, max_pitch_);

  return {pitch, yaw, true};
}

void
AttentionServerNode::joint_state_callback(
  control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg)
{
  current_yaw_ = msg->actual.positions[0];
  current_pitch_ = msg->actual.positions[1];
}

trajectory_msgs::msg::JointTrajectory
AttentionServerNode::get_command_to_angles(
  double yaw, double pitch, const rclcpp::Duration time2pos)
{
  double limit_yaw = max_vel_yaw_ * time2pos.seconds();
  double limit_pitch = max_vel_pitch_ * time2pos.seconds();

  double delta_yaw = std::clamp(yaw - current_yaw_, -limit_yaw, limit_yaw);
  double delta_pitch = std::clamp(pitch - current_pitch_, -limit_pitch, limit_pitch);

  trajectory_msgs::msg::JointTrajectory command_msg;

  command_msg.joint_names = std::vector<std::string>{"head_1_joint", "head_2_joint"};
  command_msg.points.resize(1);
  command_msg.points[0].positions.resize(2);
  command_msg.points[0].velocities.resize(2);
  command_msg.points[0].accelerations.resize(2);
  command_msg.points[0].positions[0] =
    (std::abs(delta_yaw) < rotation_threshold_) ? yaw : (current_yaw_);
  command_msg.points[0].positions[1] =
    (std::abs(delta_pitch) < rotation_threshold_) ? pitch : (current_pitch_);
  command_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.00);

  return command_msg;
}

void
AttentionServerNode::update()
{
  if (attention_frame_ == "") {return;}

  auto [pitch, yaw, success] = get_py_from_frame(attention_frame_);

  if (success) {
    auto joint_trajectory_msg = get_command_to_angles(yaw, pitch, period_);
    joint_cmd_pub_->publish(joint_trajectory_msg);
  }
}


}   // namespace attention_system
