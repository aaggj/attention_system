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

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "attention_system/OptimizedAttentionServerNode.hpp"
#include "attention_system/RoundRobinAttentionServerNode.hpp"
#include "attention_system/SimpleAttentionServerNode.hpp"

#include "attention_system_msgs/msg/attention_points.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "tf2_ros/static_transform_broadcaster.h"

#include "gtest/gtest.h"

class OptimizedAttentionServerNodeTest : public attention_system::OptimizedAttentionServerNode
{
public:
  const std::list<attention_system::AttentionPoint> & get_attention_points()
  {
    return attention_points_;
  }

  void attention_point_callback(const attention_system_msgs::msg::AttentionPoints::ConstSharedPtr msg)
  {
    
  }

};


TEST(TerminalTestCase, test_scan_floor_near_node)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto server_node = std::make_shared<OptimizedAttentionServerNodeTest>();

  // Init frames, needed by attention
  tf2_ros::StaticTransformBroadcaster tf_br(test_node);
  geometry_msgs::msg::TransformStamped bf2bl_msg;
  bf2bl_msg.header.frame_id = "base_link";
  bf2bl_msg.header.stamp = test_node->now();
  bf2bl_msg.child_frame_id = "base_footprint";
  bf2bl_msg.transform.rotation.w = 1.0;
  tf_br.sendTransform(bf2bl_msg);
  bf2bl_msg.header.frame_id = "base_footprint";
  bf2bl_msg.child_frame_id = "torso_lift_link";
  bf2bl_msg.transform.translation.z = 1.5;
  tf_br.sendTransform(bf2bl_msg);
  bf2bl_msg.header.frame_id = "torso_lift_link";
  bf2bl_msg.child_frame_id = "xtion_link";
  bf2bl_msg.transform.translation.z = 0.5;
  tf_br.sendTransform(bf2bl_msg);
 
  std::vector<attention_system_msgs::msg::AttentionPoints> attentions_points_sent;
  auto attention_points_sub = test_node->create_subscription<attention_system_msgs::msg::AttentionPoints>(
    "attention/attention_points", 100,
    [&attentions_points_sent] (const attention_system_msgs::msg::AttentionPoints::ConstSharedPtr msg) {
      attentions_points_sent.push_back(*msg);
    });

  std::vector<trajectory_msgs::msg::JointTrajectory> joints_commands_sent;
  auto joints_commands_sub = test_node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/head_controller/command", 100,
    [&joints_commands_sent] (const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
      joints_commands_sent.push_back(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(test_node);
  exe.add_node(server_node->get_node_base_interface());
  
  server_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  exe.spin_some();
  server_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  exe.spin_some();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;
  factory.registerFromPlugin(loader.getOSName("attention_system_scan_bt_node"));

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", test_node);

  const std::string bt_xml = 
    R"(<root main_tree_to_execute = "MainTree" >
          <BehaviorTree ID="MainTree">
            <Scan  name="scan" mode="floor_near"/>
          </BehaviorTree>
       </root>)";

  BT::Tree tree = factory.createTreeFromText(bt_xml, blackboard);
  
  rclcpp::Rate rate(10);
  rclcpp::Time start = test_node->now();
  while (rclcpp::ok() && (test_node->now() - start).seconds() < 5.0) {
    tree.rootNode()->executeTick();
    rate.sleep();
    exe.spin_some();
    
    ASSERT_EQ(attentions_points_sent.size(), 1u);
    ASSERT_EQ(attentions_points_sent[0].attention_points.size(), 3u);
    ASSERT_EQ(attentions_points_sent[0].instance_id, "floor_near");
    attentions_points_sent.clear();
    
    ASSERT_EQ(server_node->get_attention_points().size(), 3u);
  }
  ASSERT_FALSE(joints_commands_sent.empty());

  start = test_node->now();
  while (rclcpp::ok() && (test_node->now() - start).seconds() < 1.0) {
    rate.sleep();
    exe.spin_some();
  }
  joints_commands_sent.clear();
  ASSERT_EQ(server_node->get_attention_points().size(), 0u);

  start = test_node->now();
  while (rclcpp::ok() && (test_node->now() - start).seconds() < 1.0) {
    rate.sleep();
    exe.spin_some();
  }
  ASSERT_TRUE(joints_commands_sent.empty());
}


TEST(TerminalTestCase, test_track_node)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto server_node = std::make_shared<OptimizedAttentionServerNodeTest>();

  // Init frames, needed by attention
  tf2_ros::StaticTransformBroadcaster tf_br(test_node);
  geometry_msgs::msg::TransformStamped bf2bl_msg;
  bf2bl_msg.header.frame_id = "base_link";
  bf2bl_msg.header.stamp = test_node->now();
  bf2bl_msg.child_frame_id = "base_footprint";
  bf2bl_msg.transform.rotation.w = 1.0;
  tf_br.sendTransform(bf2bl_msg);
  bf2bl_msg.header.frame_id = "base_footprint";
  bf2bl_msg.child_frame_id = "torso_lift_link";
  bf2bl_msg.transform.translation.z = 1.5;
  tf_br.sendTransform(bf2bl_msg);
  bf2bl_msg.header.frame_id = "torso_lift_link";
  bf2bl_msg.child_frame_id = "xtion_link";
  bf2bl_msg.transform.translation.z = 0.5;
  tf_br.sendTransform(bf2bl_msg);
 
  std::vector<attention_system_msgs::msg::AttentionPoints> attentions_points_sent;
  auto attention_points_sub = test_node->create_subscription<attention_system_msgs::msg::AttentionPoints>(
    "attention/attention_points", 100,
    [&attentions_points_sent] (const attention_system_msgs::msg::AttentionPoints::ConstSharedPtr msg) {
      attentions_points_sent.push_back(*msg);
    });

  std::vector<trajectory_msgs::msg::JointTrajectory> joints_commands_sent;
  auto joints_commands_sub = test_node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/head_controller/command", 100,
    [&joints_commands_sent] (const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
      joints_commands_sent.push_back(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(test_node);
  exe.add_node(server_node->get_node_base_interface());
  
  server_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  exe.spin_some();
  server_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  exe.spin_some();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;
  factory.registerFromPlugin(loader.getOSName("attention_system_track_bt_node"));

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", test_node);

  geometry_msgs::msg::PointStamped point;
  point.header.frame_id = "base_footprint";
  point.point.x = 0.5;

  const std::string bt_xml = 
    R"(<root main_tree_to_execute = "MainTree" >
          <BehaviorTree ID="MainTree">
            <Track  name="track" mode="${point}"/>
          </BehaviorTree>
       </root>)";

  BT::Tree tree = factory.createTreeFromText(bt_xml, blackboard);
  
  rclcpp::Rate rate(10);
  rclcpp::Time start = test_node->now();
  while (rclcpp::ok() && (test_node->now() - start).seconds() < 10.0) {
    tree.rootNode()->executeTick();
    rate.sleep();
    exe.spin_some();
    
    point.point.x += 0.01;
    point.header.stamp = test_node->now();
    blackboard->set("point", point);
  }
}

/*
TEST(TerminalTestCase, test_client_simple)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto graph = std::make_shared<ros2_knowledge_graph::GraphNode>("Grapth_test");
  graph->start();

  auto attention_client = std::make_shared<AttentionClientNodeTest>(
    "table_attention_client");

  attention_client->set_parameter({"class_id", "table"});
  attention_client->set_parameter({"instances", std::vector<std::string>{}});
  attention_client->set_parameter({"default", std::vector<std::string>{
      "0.5, 0.15, 0.75",
      "0.5, -0.15, 0.75",
      "0.0, 0.15, 0.75",
      "0.0, -0.15, 0.75",
      "-0.5, 0.15, 0.75",
      "-0.5, -0.15, 0.75",
      "0.4, 0.6, 1.00",
      "0.4, -0.6, 1.00",
      "-0.4, 0.6, 1.00",
      "-0.4, -0.6, 1.00"
    }});

  std::vector<attention_system_msgs::msg::AttentionPoints> attentions_points_sent;
  auto attention_points_sub = test_node->create_subscription<attention_system_msgs::msg::AttentionPoints>(
    "/attention/attention_points", 100,
    [&attentions_points_sent] (const attention_system_msgs::msg::AttentionPoints::ConstSharedPtr msg) {
      attentions_points_sent.push_back(*msg);
    });

  std::vector<attention_system_msgs::srv::RemoveAttentionStimuli_Request> remove_attention_points_reqs;
  auto remove_point_srv = test_node->create_service<attention_system_msgs::srv::RemoveAttentionStimuli>(
    "/attention/remove_instances", [&remove_attention_points_reqs] 
      (const std::shared_ptr<attention_system_msgs::srv::RemoveAttentionStimuli::Request> req,
	    std::shared_ptr<attention_system_msgs::srv::RemoveAttentionStimuli::Response> res) {
        remove_attention_points_reqs.push_back(*req);
      });

  std::string pkgpath = ament_index_cpp::get_package_share_directory("attention_system");

  rclcpp::executors::SingleThreadedExecutor exe;

  exe.add_node(attention_client->get_node_base_interface());
  exe.add_node(test_node);

  std::thread t([&]() {
      exe.spin();
    });

  attention_client->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(attention_client->get_attention_points().size(), 1);
  ASSERT_EQ(attention_client->get_attention_points().begin()->first, "default");

  const auto & points = attention_client->get_attention_points().begin()->second;
  ASSERT_EQ(points.front().point.x, 0.5);
  ASSERT_EQ(points.front().point.y, 0.15);
  ASSERT_EQ(points.front().point.z, 0.75);
  ASSERT_EQ(points.back().point.x, -0.4);
  ASSERT_EQ(points.back().point.y, -0.6);
  ASSERT_EQ(points.back().point.z, 1.0);

  attention_client->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }
  
  ASSERT_TRUE(attentions_points_sent.empty());

  ASSERT_EQ(attention_client->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  graph->add_node({"r2d2", "robot"});
  graph->add_node({"table1", "table"});
  graph->add_edge({"sees", "symbolic", "r2d2", "table1"});
  graph->add_edge({"1.0:0.0:0.0:0.0:0.0:0.0", "tf", "r2d2", "table1"});
  graph->add_edge({"want_see", "symbolic", "r2d2", "table1"});

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while (attentions_points_sent.empty()) {
      rate.sleep();
    }

    ASSERT_LT((test_node->now() - start).seconds(), 2.0);
  }

  ASSERT_EQ(attentions_points_sent.size(), 1u);
  ASSERT_EQ(attentions_points_sent[0].class_id, "table");
  ASSERT_EQ(attentions_points_sent[0].instance_id, "table1");
  ASSERT_EQ(attentions_points_sent[0].attention_points.size(), 10u);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].header.frame_id, "table1");
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.x, 0.5);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.y, 0.15);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.z, 0.75);
  ASSERT_EQ(attentions_points_sent[0].attention_points[1].header.frame_id, "table1");
  ASSERT_EQ(attentions_points_sent[0].attention_points[1].point.x, 0.5);
  ASSERT_EQ(attentions_points_sent[0].attention_points[1].point.y, -0.15);
  ASSERT_EQ(attentions_points_sent[0].attention_points[1].point.z, 0.75);
  ASSERT_EQ(attentions_points_sent[0].attention_points[9].header.frame_id, "table1");
  ASSERT_EQ(attentions_points_sent[0].attention_points[9].point.x, -0.4);
  ASSERT_EQ(attentions_points_sent[0].attention_points[9].point.y, -0.6);
  ASSERT_EQ(attentions_points_sent[0].attention_points[9].point.z, 1.0);

  attentions_points_sent.clear();
  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while (attentions_points_sent.empty()) {
      rate.sleep();
    }

    ASSERT_LT((test_node->now() - start).seconds(), 2.0);
  }

  ASSERT_EQ(attentions_points_sent.size(), 1u);
  ASSERT_EQ(attentions_points_sent[0].class_id, "table");
  ASSERT_EQ(attentions_points_sent[0].instance_id, "table1");
  ASSERT_EQ(attentions_points_sent[0].attention_points.size(), 10u);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].header.frame_id, "table1");
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.x, 0.5);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.y, 0.15);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.z, 0.75);
  ASSERT_EQ(attentions_points_sent[0].attention_points[1].header.frame_id, "table1");
  ASSERT_EQ(attentions_points_sent[0].attention_points[1].point.x, 0.5);
  ASSERT_EQ(attentions_points_sent[0].attention_points[1].point.y, -0.15);
  ASSERT_EQ(attentions_points_sent[0].attention_points[1].point.z, 0.75);
  ASSERT_EQ(attentions_points_sent[0].attention_points[9].header.frame_id, "table1");
  ASSERT_EQ(attentions_points_sent[0].attention_points[9].point.x, -0.4);
  ASSERT_EQ(attentions_points_sent[0].attention_points[9].point.y, -0.6);
  ASSERT_EQ(attentions_points_sent[0].attention_points[9].point.z, 1.0);
  
  ASSERT_TRUE(remove_attention_points_reqs.empty());
  graph->remove_edge({"want_see", "symbolic", "r2d2", "table1"});

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 2.0) {
      rate.sleep();
    }
  }
  attentions_points_sent.clear();

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 2.0) {
      rate.sleep();
    }
  }

  ASSERT_EQ(attentions_points_sent.size(), 0u);
  ASSERT_EQ(remove_attention_points_reqs.size(), 1u);
  ASSERT_EQ(remove_attention_points_reqs[0].class_id, "table");
  ASSERT_EQ(remove_attention_points_reqs[0].instance_id, "table1");

  exe.cancel();
  t.join();
}

TEST(TerminalTestCase, test_client_options)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto graph = std::make_shared<ros2_knowledge_graph::GraphNode>("Grapth_test");
  graph->start();

  auto attention_client = std::make_shared<AttentionClientNodeTest>(
    "table_attention_client");

  attention_client->set_parameter({"class_id", "table"});
  attention_client->set_parameter({"instances", std::vector<std::string>{"table_special"}});
  attention_client->set_parameter({"default", std::vector<std::string>{
      "1, 1, 1"
    }});
  attention_client->declare_parameter("table_special");
  attention_client->set_parameter({"table_special", std::vector<std::string>{
      "2, 2, 2"
    }});

  std::vector<attention_system_msgs::msg::AttentionPoints> attentions_points_sent;
  auto attention_points_sub = test_node->create_subscription<attention_system_msgs::msg::AttentionPoints>(
    "/attention/attention_points", 100,
    [&attentions_points_sent] (const attention_system_msgs::msg::AttentionPoints::ConstSharedPtr msg) {
      attentions_points_sent.push_back(*msg);
    });

  std::vector<attention_system_msgs::srv::RemoveAttentionStimuli_Request> remove_attention_points_reqs;
  auto remove_point_srv = test_node->create_service<attention_system_msgs::srv::RemoveAttentionStimuli>(
    "/attention/remove_instances", [&remove_attention_points_reqs] 
      (const std::shared_ptr<attention_system_msgs::srv::RemoveAttentionStimuli::Request> req,
	    std::shared_ptr<attention_system_msgs::srv::RemoveAttentionStimuli::Response> res) {
        remove_attention_points_reqs.push_back(*req);
      });

  std::string pkgpath = ament_index_cpp::get_package_share_directory("attention_system");

  rclcpp::executors::SingleThreadedExecutor exe;

  exe.add_node(attention_client->get_node_base_interface());
  exe.add_node(test_node);

  std::thread t([&]() {
      exe.spin();
    });

  attention_client->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(attention_client->get_attention_points().size(), 2);
  ASSERT_EQ(attention_client->get_attention_points().begin()->first, "default");
  ASSERT_EQ((++attention_client->get_attention_points().begin())->first, "table_special");

  {
    const auto & points = attention_client->get_attention_points().begin()->second;
    ASSERT_EQ(points.size(), 1u);
    ASSERT_EQ(points.front().point.x, 1);
    ASSERT_EQ(points.front().point.y, 1);
    ASSERT_EQ(points.front().point.z, 1);
  }

  {
    const auto & points = (++attention_client->get_attention_points().begin())->second;
    ASSERT_EQ(points.size(), 1u);
    ASSERT_EQ(points.front().point.x, 2);
    ASSERT_EQ(points.front().point.y, 2);
    ASSERT_EQ(points.front().point.z, 2);
  }

  attention_client->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }
  
  ASSERT_TRUE(attentions_points_sent.empty());

  ASSERT_EQ(attention_client->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  graph->add_node({"r2d2", "robot"});
  graph->add_node({"table1", "table"});
  graph->add_node({"table_special", "table"});
  graph->add_edge({"sees", "symbolic", "r2d2", "table1"});
  graph->add_edge({"sees", "symbolic", "r2d2", "table_special"});
  graph->add_edge({"1.0:0.0:0.0:0.0:0.0:0.0", "tf", "r2d2", "table1"});
  graph->add_edge({"2.0:0.0:0.0:0.0:0.0:0.0", "tf", "r2d2", "table_special"});
  graph->add_edge({"want_see", "symbolic", "r2d2", "table1"});

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while (attentions_points_sent.empty()) {
      rate.sleep();
    }

    ASSERT_LT((test_node->now() - start).seconds(), 2.0);
  }

  ASSERT_EQ(attentions_points_sent.size(), 1u);
  ASSERT_EQ(attentions_points_sent[0].class_id, "table");
  ASSERT_EQ(attentions_points_sent[0].instance_id, "table1");
  ASSERT_EQ(attentions_points_sent[0].attention_points.size(), 1u);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].header.frame_id, "table1");
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.x, 1);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.y, 1);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.z, 1);

  graph->remove_edge({"want_see", "symbolic", "r2d2", "table1"});
  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 2.0) {
      rate.sleep();
    }
  }
  graph->add_edge({"want_see", "symbolic", "r2d2", "table_special"});
  attentions_points_sent.clear();

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while (attentions_points_sent.empty()) {
      rate.sleep();
    }

    ASSERT_LT((test_node->now() - start).seconds(), 2.0);
  }

  ASSERT_EQ(attentions_points_sent.size(), 1u);
  ASSERT_EQ(attentions_points_sent[0].class_id, "table");
  ASSERT_EQ(attentions_points_sent[0].instance_id, "table_special");
  ASSERT_EQ(attentions_points_sent[0].attention_points.size(), 1u);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].header.frame_id, "table_special");
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.x, 2);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.y, 2);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.z, 2);

  graph->add_edge({"want_see", "symbolic", "r2d2", "table1"});
 
  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 2.0) {
      rate.sleep();
    }
  }

  attentions_points_sent.clear();

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while (attentions_points_sent.size() < 2) {
      rate.sleep();
    }

    ASSERT_LT((test_node->now() - start).seconds(), 2.0);
  }

  ASSERT_EQ(attentions_points_sent.size(), 2u);
  ASSERT_EQ(attentions_points_sent[0].class_id, "table");
  ASSERT_EQ(attentions_points_sent[0].instance_id, "table1");
  ASSERT_EQ(attentions_points_sent[0].attention_points.size(), 1u);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].header.frame_id, "table1");
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.x, 1);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.y, 1);
  ASSERT_EQ(attentions_points_sent[0].attention_points[0].point.z, 1);
  ASSERT_EQ(attentions_points_sent[1].class_id, "table");
  ASSERT_EQ(attentions_points_sent[1].instance_id, "table_special");
  ASSERT_EQ(attentions_points_sent[1].attention_points.size(), 1u);
  ASSERT_EQ(attentions_points_sent[1].attention_points[0].header.frame_id, "table_special");
  ASSERT_EQ(attentions_points_sent[1].attention_points[0].point.x, 2);
  ASSERT_EQ(attentions_points_sent[1].attention_points[0].point.y, 2);
  ASSERT_EQ(attentions_points_sent[1].attention_points[0].point.z, 2);


  exe.cancel();
  t.join();
}

TEST(TerminalTestCase, test_server)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto graph = std::make_shared<ros2_knowledge_graph::GraphNode>("Grapth_test");
  graph->start();

  auto attention_client = std::make_shared<AttentionClientNodeTest>(
    "table_attention_client");
  auto attention_server = std::make_shared<OptimizedAttentionServerNodeTest>();

  attention_client->set_parameter({"class_id", "table"});
  attention_client->set_parameter({"instances", std::vector<std::string>{}});
  attention_client->set_parameter({"default", std::vector<std::string>{
      "1, 0, 0",
      "0, 1, 1"
    }});
  attention_client->declare_parameter("table_special");
  attention_client->set_parameter({"table_special", std::vector<std::string>{
      "0, -1, 1"
    }});
  std::vector<attention_system_msgs::msg::AttentionPoints> attentions_points_sent;
  auto attention_points_sub = test_node->create_subscription<attention_system_msgs::msg::AttentionPoints>(
    "/attention/attention_points", 100,
    [&attentions_points_sent] (const attention_system_msgs::msg::AttentionPoints::ConstSharedPtr msg) {
      attentions_points_sent.push_back(*msg);
    });

  std::vector<trajectory_msgs::msg::JointTrajectory> neck_commands;
  auto neck_commands_sub = test_node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/head_controller/command", 100,
    [&neck_commands] (const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
      neck_commands.push_back(*msg);
    });

  std::vector<attention_system_msgs::srv::RemoveAttentionStimuli_Request> remove_attention_points_reqs;
  auto remove_point_srv = test_node->create_service<attention_system_msgs::srv::RemoveAttentionStimuli>(
    "/attention/remove_instances", [&remove_attention_points_reqs] 
      (const std::shared_ptr<attention_system_msgs::srv::RemoveAttentionStimuli::Request> req,
	    std::shared_ptr<attention_system_msgs::srv::RemoveAttentionStimuli::Response> res) {
        remove_attention_points_reqs.push_back(*req);
      });

  std::string pkgpath = ament_index_cpp::get_package_share_directory("attention_system");

  rclcpp::executors::SingleThreadedExecutor exe;

  exe.add_node(attention_client->get_node_base_interface());
  exe.add_node(attention_server->get_node_base_interface());
  exe.add_node(test_node);

  std::thread t([&]() {
      exe.spin();
    });

  attention_client->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  attention_server->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(attention_client->get_attention_points().size(), 1);
  ASSERT_EQ(attention_client->get_attention_points().begin()->first, "default");

  {
    const auto & points = attention_client->get_attention_points().begin()->second;
    ASSERT_EQ(points.front().point.x, 1.0);
    ASSERT_EQ(points.front().point.y, 0.0);
    ASSERT_EQ(points.front().point.z, 0.0);
    ASSERT_EQ(points.back().point.x, 0.0);
    ASSERT_EQ(points.back().point.y, 1.0);
    ASSERT_EQ(points.back().point.z, 1.0);
  }

  attention_client->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  attention_server->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }
  
  ASSERT_TRUE(attentions_points_sent.empty());

  ASSERT_EQ(attention_client->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  graph->add_node({"r2d2", "robot"});
  graph->add_node({"torso_lift_link", "link"});
  graph->add_node({"xtion_link", "link"});
  graph->add_node({"table1", "table"});

  graph->add_edge({"0.0:0.0:1.0:0.0:0.0:0.0", "tf_static", "r2d2", "torso_lift_link"});
  graph->add_edge({"0.2:0.0:0.0:0.0:0.0:0.0", "tf_static", "torso_lift_link", "xtion_link"});


  graph->add_edge({"sees", "symbolic", "r2d2", "table1"});
  graph->add_edge({"1.0:0.0:0.0:0.0:0.0:0.0", "tf", "r2d2", "table1"});
  graph->add_edge({"want_see", "symbolic", "r2d2", "table1"});

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 2.0) {
      rate.sleep();
    }
  }

  const auto & points = attention_server->get_attention_points();
  ASSERT_EQ(points.size(), 2u);
  ASSERT_EQ(points.front().point.x(), 0.0);
  ASSERT_EQ(points.front().point.y(), 1.0);
  ASSERT_EQ(points.front().point.z(), 1.0);
  ASSERT_NEAR(points.front().yaw, 0.885176, 0.01);
  ASSERT_EQ(points.front().pitch, 0.0);
  ASSERT_EQ(points.back().point.x(), 1.0);
  ASSERT_EQ(points.back().point.y(), 0.0);
  ASSERT_EQ(points.back().point.z(), 0.0);
  ASSERT_EQ(points.back().yaw, 0.0);
  ASSERT_NEAR(points.back().pitch, -0.502885, 0.01);

  ASSERT_FALSE(neck_commands.empty());
  ASSERT_EQ(neck_commands.size(), 1u);
  ASSERT_EQ(neck_commands[0].joint_names.size(), 2u);
  ASSERT_EQ(neck_commands[0].points.size(), 1u);
  ASSERT_EQ(neck_commands[0].points[0].positions.size(), 2u);

  ASSERT_EQ(neck_commands[0].joint_names[0], "head_1_joint");
  ASSERT_EQ(neck_commands[0].joint_names[1], "head_2_joint");
  ASSERT_EQ(neck_commands[0].points[0].positions[0], points.front().yaw);
  ASSERT_EQ(neck_commands[0].points[0].positions[1], points.front().pitch);

  exe.cancel();
  t.join();
}
*/

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
