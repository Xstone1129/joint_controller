cmake_minimum_required(VERSION 3.8)
project(robot_lower_gateway)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_control_msg REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(robot_lower_gateway_utils SHARED
  src/feedback_policy.cpp
  src/joint_state_mapping.cpp)
target_compile_features(robot_lower_gateway_utils PUBLIC cxx_std_17)
target_include_directories(robot_lower_gateway_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_lower_gateway_utils robot_control_msg sensor_msgs)

add_library(robot_lower_backend_erobot SHARED src/erobot_backend.cpp)
target_compile_features(robot_lower_backend_erobot PUBLIC cxx_std_17)
target_include_directories(robot_lower_backend_erobot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_lower_backend_erobot
  geometry_msgs
  pluginlib
  rclcpp
  robot_control_msg
  sensor_msgs)
target_link_libraries(robot_lower_backend_erobot robot_lower_gateway_utils)

pluginlib_export_plugin_description_file(robot_lower_gateway plugins.xml)

add_executable(robot_lower_gateway_node src/robot_lower_gateway_node.cpp)
target_compile_features(robot_lower_gateway_node PUBLIC cxx_std_17)
target_include_directories(robot_lower_gateway_node PRIVATE include)
ament_target_dependencies(robot_lower_gateway_node
  diagnostic_msgs
  geometry_msgs
  pluginlib
  rclcpp
  robot_control_msg
  sensor_msgs)
target_link_libraries(robot_lower_gateway_node robot_lower_gateway_utils)

install(TARGETS
  robot_lower_gateway_utils
  robot_lower_backend_erobot
  robot_lower_gateway_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config docs launch systemd DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/run_gateway.sh
  scripts/verify_gateway_proxy.sh
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs pluginlib rclcpp robot_control_msg)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_joint_state_mapping test/test_joint_state_mapping.cpp)
  target_link_libraries(test_joint_state_mapping robot_lower_gateway_utils)
  ament_target_dependencies(test_joint_state_mapping sensor_msgs)

  ament_add_gtest(test_feedback_policy test/test_feedback_policy.cpp)
  target_link_libraries(test_feedback_policy robot_lower_gateway_utils)
  ament_target_dependencies(test_feedback_policy robot_control_msg)

  ament_add_gtest(test_native_service_forwarder test/test_native_service_forwarder.cpp)
  target_include_directories(test_native_service_forwarder PRIVATE include)
  ament_target_dependencies(test_native_service_forwarder rclcpp robot_control_msg)
endif()

ament_package()
