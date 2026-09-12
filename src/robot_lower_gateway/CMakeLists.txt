cmake_minimum_required(VERSION 3.8)
project(robot_lower_gateway)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_control_msg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(robot_lower_gateway_utils SHARED
  src/feedback_policy.cpp
  src/joint_state_mapping.cpp
  src/lift_status_parser.cpp)
target_compile_features(robot_lower_gateway_utils PUBLIC cxx_std_17)
target_include_directories(robot_lower_gateway_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_lower_gateway_utils robot_control_msg sensor_msgs)
target_link_libraries(robot_lower_gateway_utils JsonCpp::JsonCpp)

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

add_library(robot_lower_gateway_heavy_v1 SHARED src/heavy_gateway_v1.cpp)
target_compile_features(robot_lower_gateway_heavy_v1 PUBLIC cxx_std_17)
target_include_directories(robot_lower_gateway_heavy_v1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_lower_gateway_heavy_v1
  rclcpp
  robot_control_msg
  sensor_msgs
  std_msgs
  std_srvs)

add_executable(robot_lower_gateway_node src/robot_lower_gateway_node.cpp)
target_compile_features(robot_lower_gateway_node PUBLIC cxx_std_17)
target_include_directories(robot_lower_gateway_node PRIVATE include)
ament_target_dependencies(robot_lower_gateway_node
  diagnostic_msgs
  geometry_msgs
  pluginlib
  rclcpp
  robot_control_msg
  sensor_msgs
  std_msgs
  std_srvs)
target_link_libraries(robot_lower_gateway_node
  robot_lower_gateway_heavy_v1
  robot_lower_gateway_utils)

install(TARGETS
  robot_lower_gateway_utils
  robot_lower_gateway_heavy_v1
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
  scripts/wait_control_network.sh
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

  ament_add_gtest(test_gateway_residency test/test_gateway_residency.cpp)
  target_include_directories(test_gateway_residency PRIVATE include)
  ament_target_dependencies(test_gateway_residency
    rclcpp
    robot_control_msg)
  target_compile_definitions(test_gateway_residency PRIVATE
    GATEWAY_NODE_EXECUTABLE="$<TARGET_FILE:robot_lower_gateway_node>"
    GATEWAY_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/erobot_v1.yaml")

  ament_add_gtest(test_lift_zero_proxy test/test_lift_zero_proxy.cpp)
  ament_target_dependencies(test_lift_zero_proxy diagnostic_msgs rclcpp std_srvs)
  target_compile_definitions(test_lift_zero_proxy PRIVATE
    GATEWAY_NODE_EXECUTABLE="$<TARGET_FILE:robot_lower_gateway_node>"
    GATEWAY_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/erobot_v1.yaml")
  set_tests_properties(
    test_lift_zero_proxy
    PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=/tmp/robot_lower_gateway_ros_logs;ROS_DOMAIN_ID=225;ROS_LOCALHOST_ONLY=1"
    TIMEOUT 60)

  ament_add_gtest(test_lift_status_parser test/test_lift_status_parser.cpp)
  target_link_libraries(test_lift_status_parser robot_lower_gateway_utils)

  ament_add_gtest(test_lift_status_publication test/test_lift_status_publication.cpp)
  ament_target_dependencies(test_lift_status_publication
    diagnostic_msgs
    rclcpp
    robot_control_msg
    sensor_msgs
    std_msgs)
  target_compile_definitions(test_lift_status_publication PRIVATE
    GATEWAY_NODE_EXECUTABLE="$<TARGET_FILE:robot_lower_gateway_node>"
    GATEWAY_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/erobot_v1.yaml")

  ament_add_gtest(test_heavy_gateway_v1 test/test_heavy_gateway_v1.cpp)
  target_include_directories(test_heavy_gateway_v1 PRIVATE include)
  target_link_libraries(test_heavy_gateway_v1 robot_lower_gateway_heavy_v1)
  ament_target_dependencies(test_heavy_gateway_v1
    rclcpp
    robot_control_msg
    sensor_msgs
    std_msgs)
  set_tests_properties(
    test_heavy_gateway_v1
    PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=/tmp/robot_lower_gateway_ros_logs;ROS_DOMAIN_ID=221;ROS_LOCALHOST_ONLY=1"
    TIMEOUT 75)
endif()

ament_package()
