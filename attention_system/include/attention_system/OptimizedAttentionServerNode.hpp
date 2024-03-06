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

#ifndef ATTENTION_SYSTEM_OPTIMIZEDATTENTIONSERVER_H
#define ATTENTION_SYSTEM_OPTIMIZEDATTENTIONSERVER_H

#include <list>
#include <string>

#include "attention_system/AttentionServerNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace attention_system
{

class AttentionPointCompareOptimized
{
public:
  AttentionPointCompareOptimized(float ref_yaw, float ref_pitch)
  : ref_yaw_(ref_yaw), ref_pitch_(ref_pitch) {}

  bool operator()(const AttentionPoint & a, const AttentionPoint & b)
  {
    if (a.epoch < b.epoch) {
      return true;
    } else if (b.epoch < a.epoch) {
      return false;
    } else {
      return (fabs(a.yaw - ref_yaw_) + fabs(a.pitch - ref_pitch_)) <
             (fabs(b.yaw - ref_yaw_) + fabs(b.pitch - ref_pitch_));
    }
  }

  float ref_yaw_;
  float ref_pitch_;
};


class OptimizedAttentionServerNode : public AttentionServerNode
{
public:
  OptimizedAttentionServerNode();

  void update();

protected:
  rclcpp::Time ts_sent_;

  void update_points();
};

}   // namespace attention_system

#endif  // ATTENTION_SYSTEM_OPTIMIZEDATTENTIONSERVER_H
