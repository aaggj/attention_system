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

#ifndef ATTENTION_SYSTEM_ATTENTIONSERVER_H
#define ATTENTION_SYSTEM_ATTENTIONSERVER_H

#include <list>
#include <string>

#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>

#include "attention_system_msgs/msg/attention_points.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "attention_system/PIDController.hpp"
// #include "attention_system_msgs/msg/pan_tilt_command.hpp"


namespace attention_system
{

struct AttentionPoint
{
  std::string point_id;
  tf2::Stamped<tf2::Vector3> point;
  float yaw;
  float pitch;
  int epoch;
  rclcpp::Time last_time_in_fovea;
  rclcpp::Time last_update_ts;
  rclcpp::Duration lifeness {0, 0};
  rclcpp::Duration time_in_point {0, 0};
};

inline
void fromMsg(const geometry_msgs::msg::PointStamped & in, tf2::Stamped<tf2::Vector3> & out);

class AttentionServerNode : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  explicit AttentionServerNode(const std::string & name = "attention_server");
  void init();

  virtual void update() = 0;
  void print();

protected:
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  static constexpr double H_FOV = 58.0 * M_PI / 180.0;
  static constexpr double V_FOV = 45.0 * M_PI / 180.0;
  static constexpr double FOVEA_YAW = H_FOV / 2.0;
  static constexpr double FOVEA_PITCH = V_FOV / 2.0;
  static constexpr double NECK_SPEED = 0.1;
  static constexpr float MAX_YAW = 1.3;
  static constexpr float MAX_PITCH = 0.185;
  static constexpr float THRESOLD_PITCH = 0.5;
  static constexpr float THRESOLD_YAW = 0.5;

  virtual void update_points();
  void update_time_in_fovea();
  void remove_expired_points();
  void attention_point_callback(
    const attention_system_msgs::msg::AttentionPoints::ConstSharedPtr msg);
  // void joint_state_callback(control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg);
  // void command_callback(attention_system_msgs::msg::PanTiltCommand::UniquePtr msg);

  void init_join_state();
  void publish_markers();

  std::shared_ptr<tf2::BufferCore> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_lifecycle::LifecyclePublisher<
    trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  // rclcpp_lifecycle::LifecyclePublisher<
  //   attention_system_msgs::msg::PanTiltCommand>::SharedPtr comm_pub_;

  std::shared_ptr<rclcpp_action::Client<
      control_msgs::action::FollowJointTrajectory>> action_client_;
  bool goal_result_available_{false};
  bool goal_sent_{false};
  rclcpp_action::ClientGoalHandle<
    control_msgs::action::FollowJointTrajectory>::WrappedResult result_;
  rclcpp_action::ClientGoalHandle<
    control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle_;

  rclcpp::Subscription<attention_system_msgs::msg::AttentionPoints>::SharedPtr attention_points_sub_;
  // rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
  //   joint_state_sub_;
  // rclcpp::Subscription<attention_system_msgs::msg::PanTiltCommand>::SharedPtr command_sub_;

  std::list<AttentionPoint> attention_points_;

  rclcpp::Time time_in_pos_, last_command_ts_;

  trajectory_msgs::msg::JointTrajectory joint_cmd_;

  sensor_msgs::msg::JointState joint_state_;

  float current_yaw_, current_pitch_, goal_yaw_, goal_pitch_, last_yaw_,
    last_pitch_;



  rclcpp::TimerBase::SharedPtr timer_;
  // attention_system_msgs::msg::PanTiltCommand::UniquePtr last_command_;
  // control_msgs::msg::JointTrajectoryControllerState::UniquePtr last_state_;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;


  std::chrono::milliseconds server_timeout_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}   // namespace attention_system

#endif  // ATTENTION_SYSTEM_ATTENTIONSERVER_H
