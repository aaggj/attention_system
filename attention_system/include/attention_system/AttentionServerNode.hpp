// Copyright 2024 Intelligent Robotics Lab
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

#ifndef ATTENTION_SYSTEM__ATTENTIONSERVERNODE_HPP_
#define ATTENTION_SYSTEM__ATTENTIONSERVERNODE_HPP_

#include <tuple>
#include <memory>
#include <list>
#include <string>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/transform_datatypes.h"

#include "attention_system_msgs/msg/attention_command.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"


namespace attention_system
{

using namespace std::chrono_literals;  // NOLINT

class AttentionServerNode : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  explicit AttentionServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  virtual void update();

protected:
  void attention_command_callback(
    attention_system_msgs::msg::AttentionCommand::SharedPtr msg);
  void joint_state_callback(control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg);

  std::tuple<double, double, bool> get_py_from_frame(const std::string & frame);
  trajectory_msgs::msg::JointTrajectory get_command_to_angles(
    double yaw, double pitch, const rclcpp::Duration time2pos);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    joint_cmd_pub_;
  rclcpp::Subscription<attention_system_msgs::msg::AttentionCommand>::SharedPtr
    attention_command_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_sub_;

  std::string attention_frame_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Duration period_ {50ms};
  double current_yaw_ {0.0}, current_pitch_{0.0};
  double max_yaw_ {1.3};
  double max_pitch_ {0.185};
  double max_vel_yaw_ {0.5};  // rad/sec
  double max_vel_pitch_ {0.5};  // rad/sec
  double rotation_threshold_ {0.436332};  // 25 degrees
};

}   // namespace attention_system

#endif  // ATTENTION_SYSTEM__ATTENTIONSERVERNODE_HPP_
