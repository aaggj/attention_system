cmake_minimum_required(VERSION 3.5)
project(attention_system)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(attention_system_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

set(dependencies
  rclcpp
  rclcpp_cascade_lifecycle
  rclcpp_action
  lifecycle_msgs
  std_msgs
  geometry_msgs
  attention_system_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  control_msgs
  visualization_msgs
  behaviortree_cpp_v3
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/attention_system/AttentionServerNode.cpp
  src/attention_system/OptimizedAttentionServerNode.cpp
  src/attention_system/RoundRobinAttentionServerNode.cpp
  src/attention_system/SimpleAttentionServerNode.cpp
  src/attention_system/PIDController.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(attention_server src/attention_server.cpp)
ament_target_dependencies(attention_server ${dependencies})
target_link_libraries(attention_server ${PROJECT_NAME})

add_library(attention_system_scan_bt_node SHARED src/behavior_tree_nodes/Scan.cpp)
list(APPEND plugin_libs attention_system_scan_bt_node)

add_library(attention_system_track_bt_node SHARED src/behavior_tree_nodes/Track.cpp)
list(APPEND plugin_libs attention_system_track_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  attention_server
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  add_subdirectory(tests)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()