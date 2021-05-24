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

#ifndef GB_ATTENTION_ROUNDROBINATTENTIONSERVER_H
#define GB_ATTENTION_ROUNDROBINATTENTIONSERVER_H

#include <list>
#include <string>

#include "gb_attention/AttentionServerNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace gb_attention
{

class AttentionPointCompareRoundRobin
{
public:
	AttentionPointCompareRoundRobin() {}

	bool operator()(const AttentionPoint& a, const AttentionPoint& b)
  {
		return (a.epoch < b.epoch);
	}
};


class RoundRobinAttentionServerNode: public AttentionServerNode
{
public:
	RoundRobinAttentionServerNode();

	void update();

protected:
	void update_points();
};

};  // namespace gb_attention

#endif  // GB_ATTENTION_ROUNDROBINATTENTIONSERVER_H
