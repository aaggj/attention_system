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
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "attention_system/OptimizedAttentionServerNode.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/rclcpp_action.hpp"


namespace attention_system
{

using namespace std::chrono_literals;

OptimizedAttentionServerNode::OptimizedAttentionServerNode()
{
  ts_sent_ = now() - rclcpp::Duration(5, 0);
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
//   if(last_state_ == nullptr) {return;}
  remove_expired_points();

  if (attention_points_.empty()) {
    RCLCPP_INFO(get_logger(), "###############No attention points#################");
    return;
  }

  update_time_in_fovea();


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
    (now() - time_in_pos_) > (rclcpp::Duration(1, 0) + attention_points_.begin()->time_in_point))
  {
    RCLCPP_INFO(get_logger(), "**********************************");
    if ((now() - ts_sent_).seconds() > 10) {
      RCLCPP_WARN(get_logger(), "Timeout in attention point. Skipping");
      RCLCPP_INFO(get_logger(), "Current time: %f", (now() - ts_sent_).seconds());
      RCLCPP_INFO(get_logger(), "Now time: %f", now().seconds());
      RCLCPP_INFO(get_logger(), "ts_sent_ time: %f", ts_sent_.seconds());

    }

    attention_points_.begin()->epoch++;

    double x = attention_points_.begin()->point[0];
    double y = attention_points_.begin()->point[1];
    double z = attention_points_.begin()->point[2];

    RCLCPP_INFO(this->get_logger(), "Punto x: %f, y: %f, z: %f", x, y, z);
    RCLCPP_INFO(
      get_logger(), "1st point id : %s",
      attention_points_.begin()->point.frame_id_.c_str());

    update_points();
    RCLCPP_INFO(get_logger(), "Points updated");
    publish_markers();

    RCLCPP_INFO(get_logger(), "Markers published");

    double control_pan, control_tilt;
    control_pan = pan_pid_.get_output(0.0);
    control_tilt = tilt_pid_.get_output(0.0);

    goal_yaw_ = std::max(
      -MAX_YAW, std::min(
        MAX_YAW,
        attention_points_.begin()->yaw));
    goal_pitch_ = std::max(
      -MAX_PITCH, std::min(
        MAX_PITCH,
        attention_points_.begin()->pitch));

    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();

    goal_msg.trajectory.joint_names.resize(2);
    goal_msg.trajectory.joint_names.push_back("head_1_joint");
    goal_msg.trajectory.joint_names.push_back("head_2_joint");
    goal_msg.trajectory.points.resize(2);
    goal_msg.trajectory.points[0].positions.resize(2);

    goal_msg.trajectory.points[0].positions[0] = goal_yaw_;
    goal_msg.trajectory.points[0].positions[1] = goal_pitch_;

    goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration(1s);

    // joint_cmd_.points[0].positions[0] = goal_yaw_ - control_pan;
    // joint_cmd_.points[0].positions[1] = goal_pitch_ - control_tilt;

    // joint_cmd_.header.stamp = now() + rclcpp::Duration(1s) ;  // Disabled in sim
    ts_sent_ = now();

    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Action server not available after waiting, exiting update");
      return;
    }

    goal_result_available_ = false;
    auto send_goal_options = rclcpp_action::Client<
      control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

    send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<
          control_msgs::action::FollowJointTrajectory>::WrappedResult & result) {
        if (this->goal_handle_->get_goal_id() == result.goal_id) {
          goal_result_available_ = true;
          result_ = result;
          RCLCPP_INFO(this->get_logger(), "Goal result received");
        }
      };


    auto future_goal_handle = action_client_->async_send_goal(
      goal_msg,
      send_goal_options);

    // RCLCPP_ERROR(this->get_logger(), "Sending goal");

    if (rclcpp::spin_until_future_complete(node_, future_goal_handle) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "send goal call failed :(, exiting update");
      return;
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Goal was rejected by server, exiting update");
      return;
    }
    // joint_cmd_pub_->publish(joint_cmd_);
    RCLCPP_INFO(
      get_logger(), "joint_cmd: %f, %f", joint_cmd_.points[0].positions[0],
      joint_cmd_.points[0].positions[1]);
  }

  if (fabs(current_yaw_ - goal_yaw_) > FOVEA_YAW ||
    fabs(current_pitch_ - goal_pitch_) > FOVEA_PITCH)
  {
    time_in_pos_ = now();
  }
  RCLCPP_INFO(get_logger(), "OptimizedAttentionServerNode::update() end");
}


}   // namespace attention_system
