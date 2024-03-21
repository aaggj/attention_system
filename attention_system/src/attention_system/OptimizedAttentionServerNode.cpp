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
  callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_executor_.add_callback_group(
    callback_group_, this->get_node_base_interface());
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

  RCLCPP_INFO(get_logger(), "Markers published");

  goal_yaw_ = std::max(
    -MAX_YAW, std::min(
      MAX_YAW,
      attention_points_.begin()->yaw));
  goal_pitch_ = std::max(
    -MAX_PITCH, std::min(
      MAX_PITCH,
      attention_points_.begin()->pitch));

  auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();

  goal_msg.trajectory.joint_names = std::vector<std::string>{"head_1_joint", "head_2_joint"};

  trajectory_msgs::msg::JointTrajectoryPoint point;

  std::cout << "goal_yaw_: " << goal_yaw_ << std::endl;
  std::cout << "goal_pitch_: " << goal_pitch_ << std::endl;
  goal_yaw_ = std::clamp(goal_yaw_, -MAX_YAW, MAX_YAW);
  point.positions = std::vector<double>{goal_yaw_, goal_pitch_};
  point.time_from_start = rclcpp::Duration::from_seconds(1.0);

  goal_msg.trajectory.points.push_back(point);

  ts_sent_ = now();

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

  auto dif_yaw = fabs(goal_yaw_ - last_yaw_);
  auto dif_pitch = fabs(goal_pitch_ - last_pitch_);

  if (!goal_sent_ || (dif_yaw < THRESOLD_YAW || dif_pitch < THRESOLD_PITCH)) {

    auto future_goal_handle = action_client_->async_send_goal(
      goal_msg);
    goal_sent_ = true;
    RCLCPP_INFO(get_logger(), "Sending goal");


    callback_group_executor_.spin_some();
    RCLCPP_INFO(get_logger(), "Goal sent, waiting for result");

  } else {
    RCLCPP_INFO(
      get_logger(),
      "Goal not sent thresold exceeded or goal already sent");
    RCLCPP_INFO(
      get_logger(),
      "dif_yaw: %f, dif_pitch: %f, goal_sent: %d", dif_yaw, dif_pitch,
      goal_sent_);
  }
  // }

  if (fabs(current_yaw_ - goal_yaw_) > FOVEA_YAW ||
    fabs(current_pitch_ - goal_pitch_) > FOVEA_PITCH)
  {
    time_in_pos_ = now();
  }
  last_pitch_ = goal_pitch_;
  last_yaw_ = goal_yaw_;

  RCLCPP_INFO(get_logger(), "OptimizedAttentionServerNode::update() end");
}


}   // namespace attention_system
