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

#ifndef GB_ATTENTION_ATTENTIONSERVER_H
#define GB_ATTENTION_ATTENTIONSERVER_H

#include <list>
#include <string>

#include <tf2_ros/transform_listener.h>

#include "gb_attention_msgs/msg/attention_points.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace gb_attention
{

struct AttentionPoint
{
	std::string point_id;
	geometry_msgs::msg::PointStamped point;
	float yaw;
	float pitch;
	int epoch;
  rclcpp::Time last_time_in_fovea;
  rclcpp::Time last_update_ts;
  rclcpp::Duration lifeness {0,0};
  rclcpp::Duration time_in_point {0,0};
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
  static constexpr float MAX_YAW = M_PI / 2.0;
  static constexpr float MAX_PITCH = M_PI / 3.0;

	virtual void update_points();
  void update_time_in_fovea();
  void remove_expired_points();
	void attention_point_callback(const gb_attention_msgs::msg::AttentionPoints::ConstSharedPtr msg);
	void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);

	void init_join_state();
	void publish_markers();
  
	std::shared_ptr<tf2::BufferCore> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

	rclcpp::Subscription<gb_attention_msgs::msg::AttentionPoints>::SharedPtr attention_points_sub_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

	std::list<AttentionPoint> attention_points_;

	rclcpp::Time time_in_pos_;

	trajectory_msgs::msg::JointTrajectory joint_cmd_;
	sensor_msgs::msg::JointState joint_state_;

	float current_yaw_;
	float current_pitch_;
	float goal_yaw_;
	float goal_pitch_;

  rclcpp::TimerBase::SharedPtr timer_;
};

};  // namespace gb_attention

#endif  // GB_ATTENTION_ATTENTIONSERVER_H
