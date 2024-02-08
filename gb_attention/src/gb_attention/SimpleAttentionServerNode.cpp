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

#include "gb_attention/SimpleAttentionServerNode.hpp"

#include "rclcpp/rclcpp.hpp"


namespace gb_attention
{

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
  remove_expired_points();

  if (attention_points_.empty()) {
    return;
  }

  update_time_in_fovea();

	update_points();
	update_limits();

	trajectory_msgs::msg::JointTrajectory joint_cmd;

  joint_cmd.joint_names.resize(2);
  joint_cmd.joint_names[0] ="head_1_joint";
	joint_cmd.joint_names[1] ="head_2_joint";

	joint_cmd.points.resize(1);

  if (change_ && vertical_) {
    change_ = false;
    direction_pitch_ = -direction_pitch_;
		//  Vertical movement
		joint_cmd.points[0].positions.resize(2);
  	joint_cmd.points[0].velocities.resize(2);
  	joint_cmd.points[0].accelerations.resize(2);
  	joint_cmd.points[0].time_from_start = rclcpp::Duration(MOV_DURATION);

		if (current_pitch_ > max_pitch_) direction_pitch_ = -1.0;
		if (current_pitch_ < min_pitch_) direction_pitch_ = 1.0;

    if (fabs(goal_pitch_ - min_pitch_) < 0.1) {
      goal_pitch_ = max_pitch_;
    } else {
      goal_pitch_ = min_pitch_;
    }

		joint_cmd.points[0].positions[0] = current_yaw_;
		joint_cmd.points[0].positions[1] = goal_pitch_;

    //std::cerr << "---------> New goal pitch = " << goal_pitch_ << std::endl;
		joint_cmd.points[0].velocities[0] = NECK_SPEED;
		joint_cmd.points[0].velocities[1] = 0.0;
		joint_cmd.points[0].accelerations[0] = NECK_SPEED;
		joint_cmd.points[0].accelerations[1] = 0.0;

    ts_sent_ = now();
  }

  if (change_ && !vertical_) {
    change_ = false;
    
		//  Vertical movement
		joint_cmd.points[0].positions.resize(2);
  	joint_cmd.points[0].velocities.resize(2);
  	joint_cmd.points[0].accelerations.resize(2);
  	joint_cmd.points[0].time_from_start = rclcpp::Duration(MOV_DURATION);

		if (current_yaw_ > max_yaw_) direction_yaw_ = -1.0;
		if (current_yaw_ < min_yaw_) direction_yaw_ = 1.0;

		goal_yaw_ = current_yaw_ + direction_yaw_ * FOVEA_YAW;
    
    joint_cmd.points[0].positions[0] = goal_yaw_;
		joint_cmd.points[0].positions[1] = current_pitch_;

    //std::cerr << "---------> New goal yaw = " << goal_yaw_ << std::endl;
		joint_cmd.points[0].velocities[0] = NECK_SPEED;
		joint_cmd.points[0].velocities[1] = 0.0;
		joint_cmd.points[0].accelerations[0] = NECK_SPEED;
		joint_cmd.points[0].accelerations[1] = 0.0;
    ts_sent_ = now();
  }
 
//  if (vertical_) {
//    std::cerr << "Vertical test : " << goal_pitch_ << " - " << current_pitch_ << " = " <<
//      fabs(goal_pitch_ - current_pitch_) << std::endl;
//  } else {
//    std::cerr << "Horizontal test : " << goal_yaw_ << " - " << current_yaw_ << " = " <<
//      fabs(goal_yaw_ - current_yaw_) << std::endl;
//  }
//
  auto elapsed = (now() - ts_sent_);

  if (vertical_ && ((fabs(goal_pitch_ - current_pitch_) < DIST_TOL) || elapsed > rclcpp::Duration(2 * MOV_DURATION))) {
    change_ = true;
    vertical_ = !vertical_;

    //std::cerr << "Change to horizontal" << std::endl;
  } else if (!vertical_ && ((fabs(goal_yaw_ - current_yaw_) < DIST_TOL) || elapsed > rclcpp::Duration(2 * MOV_DURATION))) {

    if (direction_yaw_ > 0 && fabs(goal_yaw_ - max_yaw_) < DIST_TOL) {
      direction_yaw_ = -direction_yaw_;
    }
    if (direction_yaw_ < 0 && fabs(goal_yaw_ - min_yaw_) < DIST_TOL) {
      direction_yaw_ = -direction_yaw_;
    }

    vertical_ = !vertical_;
    change_ = true;
    //std::cerr << "Change to vertical" << std::endl;
  }


	joint_cmd_pub_->publish(joint_cmd);
	

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


};  // namespace gb_attention
