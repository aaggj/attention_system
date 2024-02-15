// Copyright 2019 Intelligent Robotics Lab
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

#ifndef ATTENTION_SYSTEM__BEHAVIOR_TREE_NODES__TRACK_HPP_
#define ATTENTION_SYSTEM__BEHAVIOR_TREE_NODES__TRACK_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "attention_system_msgs/msg/attention_points.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace attention_system
{

class Track : public BT::ActionNodeBase
{
public:
  explicit Track(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose")
    };
  }

private:
  rclcpp::Publisher<attention_system_msgs::msg::AttentionPoints>::SharedPtr points_pub_;
};

}  // namespace attention_system

#endif  // ATTENTION_SYSTEM__BEHAVIOR_TREE_NODES__TRACK_HPP_
