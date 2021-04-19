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

#include "lifecycle_msgs/msg/transition.hpp"

#include "gb_attention/OptimizedAttentionServerNode.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto attention_server_node = std::make_shared<gb_attention::OptimizedAttentionServerNode>();

  attention_server_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin_some(attention_server_node->get_node_base_interface());
  attention_server_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::spin(attention_server_node->get_node_base_interface());

  return 0;
 }
