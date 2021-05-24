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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "visualization_msgs/msg/marker_array.hpp"

#include "gb_attention/OptimizedAttentionServerNode.hpp"

#include "rclcpp/rclcpp.hpp"


namespace gb_attention
{

using namespace std::chrono_literals;

OptimizedAttentionServerNode::OptimizedAttentionServerNode()
{
	ts_sent_ = now() - rclcpp::Duration(5.0);
	time_in_pos_ = now();
}

void
OptimizedAttentionServerNode::update_points()
{
	AttentionServerNode::update_points();

	attention_points_.sort(AttentionPointCompareOptimized(current_yaw_, current_pitch_));
}


void
OptimizedAttentionServerNode::update()
{
  remove_expired_points();

  if (attention_points_.empty()) {
    return;
  }

  update_time_in_fovea();


	for (auto & point : attention_points_) {
		geometry_msgs::msg::TransformStamped p2torso_msg;
		tf2::Transform point2torso;
		tf2::Transform torso2head1;
		tf2::Transform head12head;

		std::string error;
		if (tfBuffer_->canTransform(point.point.frame_id_, "torso_lift_link",
			tf2::TimePointZero, &error))
    {
			p2torso_msg = tfBuffer_->lookupTransform(point.point.frame_id_, "torso_lift_link", tf2::TimePointZero);
    } else {
			RCLCPP_ERROR(get_logger(), "Can't transform %s", error.c_str());
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

	if ((now() - ts_sent_).seconds() > 5.0 ||
		(now() - time_in_pos_) > (rclcpp::Duration(1.0) + attention_points_.begin()->time_in_point))
  {
		if ((now() - ts_sent_).seconds() > 10) {
			RCLCPP_WARN(get_logger(), "Timeout in attention point. Skipping");
    }

		attention_points_.begin()->epoch++;
		update_points();
		publish_markers();

		goal_yaw_ = std::max(-MAX_YAW, std::min(MAX_YAW, attention_points_.begin()->yaw));
		goal_pitch_ = std::max(-MAX_PITCH, std::min(MAX_PITCH, attention_points_.begin()->pitch));

		joint_cmd_.points[0].positions[0] = goal_yaw_;
		joint_cmd_.points[0].positions[1] = goal_pitch_;

		// joint_cmd_.header.stamp = now() + rclcpp::Duration(1s) ;  // Disabled in sim
		ts_sent_ = now();
		joint_cmd_pub_->publish(joint_cmd_);
	}

	if (fabs(current_yaw_ - goal_yaw_) > FOVEA_YAW ||
			fabs(current_pitch_ - goal_pitch_) > FOVEA_PITCH)
  {
		time_in_pos_ = now();
  }
}


};  // namespace gb_attention
