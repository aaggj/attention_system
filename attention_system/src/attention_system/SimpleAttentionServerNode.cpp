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

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "visualization_msgs/msg/marker_array.hpp"

#include "attention_system/SimpleAttentionServerNode.hpp"

#include "rclcpp/rclcpp.hpp"


namespace attention_system
{
using namespace std::chrono_literals;

#define DIST_TOL	0.02

#define MOV_DURATION	1s

using namespace std::chrono_literals;

SimpleAttentionServerNode::SimpleAttentionServerNode()
: duration_{1, 0}
{
	direction_yaw_ = 1.0f;
	direction_pitch_ = 1.0f;

  vertical_ = true;
  change_ = true;

	ts_sent_ = now();
}

void
SimpleAttentionServerNode::update()
{
  RCLCPP_INFO(get_logger(), "update");
  remove_expired_points();
  RCLCPP_INFO(get_logger(), "remove_expired_points");

  if (attention_points_.empty()) {
    RCLCPP_INFO(get_logger(), "###############No attention points, Returning#################");
    return;
  }

  update_time_in_fovea();
  RCLCPP_INFO(get_logger(), "update_time_in_fovea");

	update_points();
  RCLCPP_INFO(get_logger(), "update_points");
	update_limits();
  RCLCPP_INFO(get_logger(), "update_limits");

  // attention_system_msgs::msg::PanTiltCommand command;
  // command.pan = 0.0;
  // command.tilt = 0.0;
  // comm_pub_->publish(command);
  // RCLCPP_INFO(get_logger(), "comm_pub_->publish(command)");

  // trajectory_msgs::msg::JointTrajectory command_msg;
  //   command_msg.header.stamp = now();
  //   command_msg.joint_names = last_state_->joint_names;
  //   command_msg.points.resize(1);
  //   command_msg.points[0].positions.resize(2);
  //   command_msg.points[0].velocities.resize(2);
  //   command_msg.points[0].accelerations.resize(2);
  //   command_msg.points[0].time_from_start = rclcpp::Duration(200ms);
        
  //   if (last_command_ == nullptr || (now() - last_command_ts_) > 100ms) {
  //     command_msg.points[0].positions[0] = 0.0;
  //     command_msg.points[0].positions[1] = 0.0;
  //     command_msg.points[0].velocities[0] = 0.1;
  //     command_msg.points[0].velocities[1] = 0.1;
  //     command_msg.points[0].accelerations[0] = 0.1;
  //     command_msg.points[0].accelerations[1] = 0.1;
  //     command_msg.points[0].time_from_start = rclcpp::Duration(1s);
  //   } else {
  //     double control_pan = pan_pid_.get_output(last_command_->pan);
  //     double control_tilt = tilt_pid_.get_output(last_command_->tilt);
  
  //     command_msg.points[0].positions[0] = goal_yaw_ - control_pan;
  //     command_msg.points[0].positions[1] = goal_pitch_ - control_tilt;
  
  //     command_msg.points[0].velocities[0] = 0.5;
  //     command_msg.points[0].velocities[1] = 0.5;
  //     command_msg.points[0].accelerations[0] = 0.5;
  //     command_msg.points[0].accelerations[1] = 0.5;
  //   }
  
  //   joint_cmd_pub_->publish(command_msg);

  // joint_cmd.joint_names.resize(2);
  // joint_cmd.joint_names[0] ="head_1_joint";
	// joint_cmd.joint_names[1] ="head_2_joint";

	// joint_cmd.points.resize(1);
  bool init = true;
  double control_pan, control_tilt;
  if (init) {
  control_pan = pan_pid_.get_output(0.0);
  RCLCPP_INFO(get_logger(), "pan_pid_.get_output");
  control_tilt = tilt_pid_.get_output(0.0);
  RCLCPP_INFO(get_logger(), "tilt_pid_.get_output");
  init = false;
  }
  

  if (change_ && vertical_) {
    change_ = false;
    direction_pitch_ = -direction_pitch_;
		//  Vertical movement
		joint_cmd_.points[0].positions.resize(2);
  	joint_cmd_.points[0].velocities.resize(2);
  	joint_cmd_.points[0].accelerations.resize(2);
  	joint_cmd_.points[0].time_from_start = rclcpp::Duration(200ms);

		if (current_pitch_ > max_pitch_) direction_pitch_ = -1.0;
		if (current_pitch_ < min_pitch_) direction_pitch_ = 1.0;

    if (fabs(goal_pitch_ - min_pitch_) < 0.1) {
      goal_pitch_ = max_pitch_;
    } else {
      goal_pitch_ = min_pitch_;
    }

		joint_cmd_.points[0].positions[0] = current_yaw_;
    if (init)
		joint_cmd_.points[0].positions[1] = goal_pitch_ - control_tilt;
    else
    joint_cmd_.points[0].positions[1] = goal_pitch_;

    std::cerr << "---------> New goal pitch = " << goal_pitch_ << std::endl;
		joint_cmd_.points[0].velocities[0] = NECK_SPEED;
		joint_cmd_.points[0].velocities[1] = 0.0;
		joint_cmd_.points[0].accelerations[0] = NECK_SPEED;
		joint_cmd_.points[0].accelerations[1] = 0.0;

    ts_sent_ = now();
  }

  if (change_ && !vertical_) {
    change_ = false;
    
		//  Vertical movement
		joint_cmd_.points[0].positions.resize(2);
  	joint_cmd_.points[0].velocities.resize(2);
  	joint_cmd_.points[0].accelerations.resize(2);
  	joint_cmd_.points[0].time_from_start = rclcpp::Duration(MOV_DURATION);

		if (current_yaw_ > max_yaw_) direction_yaw_ = -1.0;
		if (current_yaw_ < min_yaw_) direction_yaw_ = 1.0;

		goal_yaw_ = current_yaw_ + direction_yaw_ * FOVEA_YAW;
    
    if(init)
    joint_cmd_.points[0].positions[0] = goal_yaw_ - control_pan;
    else
    joint_cmd_.points[0].positions[0] = goal_yaw_;
		joint_cmd_.points[0].positions[1] = current_pitch_;

    std::cerr << "---------> New goal yaw = " << goal_yaw_ << std::endl;
		joint_cmd_.points[0].velocities[0] = NECK_SPEED;
		joint_cmd_.points[0].velocities[1] = 0.0;
		joint_cmd_.points[0].accelerations[0] = NECK_SPEED;
		joint_cmd_.points[0].accelerations[1] = 0.0;
    ts_sent_ = now();
  }
 
  auto elapsed = (now() - ts_sent_);

  if (vertical_ && ((fabs(goal_pitch_ - current_pitch_) < DIST_TOL) || elapsed > rclcpp::Duration(2 * MOV_DURATION))) {
    change_ = true;
    vertical_ = !vertical_;

    std::cerr << "Change to horizontal" << std::endl;
  } else if (!vertical_ && ((fabs(goal_yaw_ - current_yaw_) < DIST_TOL) || elapsed > rclcpp::Duration(2 * MOV_DURATION))) {

    if (direction_yaw_ > 0 && fabs(goal_yaw_ - max_yaw_) < DIST_TOL) {
      direction_yaw_ = -direction_yaw_;
    }
    if (direction_yaw_ < 0 && fabs(goal_yaw_ - min_yaw_) < DIST_TOL) {
      direction_yaw_ = -direction_yaw_;
    }

    vertical_ = !vertical_;
    change_ = true;
    std::cerr << "Change to vertical" << std::endl;
  }


	joint_cmd_pub_->publish(joint_cmd_);
	

	publish_markers();
}

void
SimpleAttentionServerNode::update_limits()
{
	max_yaw_ = -std::numeric_limits<float>::max();
	max_pitch_ = -std::numeric_limits<float>::max();
	min_yaw_ = std::numeric_limits<float>::max();
	min_pitch_ = std::numeric_limits<float>::max();

	for (const auto & point : attention_points_) {
		max_yaw_ = std::max<float>(max_yaw_, point.yaw);
		max_pitch_ = std::max<float>(max_pitch_, point.pitch);
		min_yaw_ = std::min<float>(min_yaw_, point.yaw);
		min_pitch_ = std::min<float>(min_pitch_, point.pitch);
	}
  
  // max_yaw_ = std::max(-MAX_YAW, std::min(MAX_YAW, max_yaw_));
  // min_yaw_ = std::max(-MAX_YAW, std::min(MAX_YAW, min_yaw_));
  // max_pitch_ = std::max(-MAX_YAW, std::min(MAX_YAW, max_pitch_));
  // min_pitch_ = std::max(-MAX_YAW, std::min(MAX_YAW, min_pitch_));
}


};  // namespace attention_system
