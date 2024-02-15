ament_add_gtest(test_attention test_attention.cpp)
ament_target_dependencies(test_attention ${dependencies})
target_link_libraries(test_attention ${PROJECT_NAME})