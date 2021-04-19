cmake_minimum_required(VERSION 3.5)
project(gb_attention)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gb_attention_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_knowledge_graph REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ros2_knowledge_graph_plugins REQUIRED)

set(dependencies
  rclcpp
  rclcpp_cascade_lifecycle
  lifecycle_msgs
  std_msgs
  geometry_msgs
  gb_attention_msgs
  sensor_msgs
  ros2_knowledge_graph
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  visualization_msgs
  ros2_knowledge_graph_plugins
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/gb_attention/AttentionClientNode.cpp
  src/gb_attention/AttentionServerNode.cpp
  src/gb_attention/OptimizedAttentionServerNode.cpp
  src/gb_attention/RoundRobinAttentionServerNode.cpp
  src/gb_attention/SimpleAttentionServerNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(attention_server src/attention_server.cpp)
ament_target_dependencies(attention_server ${dependencies})
target_link_libraries(attention_server ${PROJECT_NAME})

add_executable(attention_client src/attention_client.cpp)
ament_target_dependencies(attention_client ${dependencies})
target_link_libraries(attention_client ${PROJECT_NAME})

add_executable(attention_manager src/attention_manager.cpp)
ament_target_dependencies(attention_manager ${dependencies})
target_link_libraries(attention_manager ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  attention_server
  attention_client
  attention_manager
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