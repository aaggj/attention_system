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

#include "attention_system/RoundRobinAttentionServerNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace attention_system
{

using namespace std::chrono_literals;

RoundRobinAttentionServerNode::RoundRobinAttentionServerNode()
{
}

void
RoundRobinAttentionServerNode::update_points()
{
	AttentionServerNode::update_points();

	attention_points_.sort(AttentionPointCompareRoundRobin());
}


void
RoundRobinAttentionServerNode::update()
{
  remove_expired_points();

  if (attention_points_.empty()) {
    return;
  }

  update_time_in_fovea();

	if ((now() - time_in_pos_) > (attention_points_.begin()->time_in_point + rclcpp::Duration(1, 0)))
  {
		attention_points_.begin()->epoch++;

		update_points();
		publish_markers();

		goal_yaw_ = std::max(-MAX_YAW, std::min(MAX_YAW, attention_points_.begin()->yaw));
		goal_pitch_ = std::max(-MAX_PITCH, std::min(MAX_PITCH, attention_points_.begin()->pitch));

		joint_cmd_.points[0].positions[0] = goal_yaw_;
		joint_cmd_.points[0].positions[1] = goal_pitch_;

		// joint_cmd_.header.stamp = now() + rclcpp::Duration(1s);
		joint_cmd_pub_->publish(joint_cmd_);
	}

	if (fabs(current_yaw_ - goal_yaw_) > FOVEA_YAW ||
			fabs(current_pitch_ - goal_pitch_) > FOVEA_PITCH)
  {
		time_in_pos_ = now();
  }
}


};  // namespace attention_system
