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

#ifndef ATTENTION_SYSTEM_SIMPLEATTENTIONSERVER_H
#define ATTENTION_SYSTEM_SIMPLEATTENTIONSERVER_H

#include <list>
#include <string>

#include "attention_system/AttentionServerNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace attention_system
{

class SimpleAttentionServerNode : public AttentionServerNode
{
public:
  SimpleAttentionServerNode();

  void update();

protected:
  void update_limits();

  float max_yaw_, min_yaw_;
  float max_pitch_, min_pitch_;

  float direction_yaw_, direction_pitch_;

  rclcpp::Time ts_sent_;
  rclcpp::Duration duration_;
  bool vertical_;
  bool change_;
};

}   // namespace attention_system

#endif  // ATTENTION_SYSTEM_SIMPLEATTENTIONSERVER_H
