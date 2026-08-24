cmake_minimum_required(VERSION 3.8)
project(robot_control_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(LIFT_STATUS_INTERFACE_SHA256
  "6ca1acd5609994451f156f4dcc7ccb745291922bef12cbecdb7ecf5c1a89a51b")
file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/msg/LiftStatus.msg" LIFT_STATUS_SOURCE_SHA256)
if(NOT LIFT_STATUS_SOURCE_SHA256 STREQUAL LIFT_STATUS_INTERFACE_SHA256)
  message(FATAL_ERROR
    "LiftStatus.msg ABI changed: expected ${LIFT_STATUS_INTERFACE_SHA256}, "
    "got ${LIFT_STATUS_SOURCE_SHA256}")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Robotarmservomsg.msg"
  "msg/Robotteleopctrl.msg"
  "msg/Robotteleopstate.msg"
  "msg/Robotarmmovel.msg"
  "msg/Robotarmmovelpath.msg"
  "msg/Robotarmjoint.msg"
  "msg/ArmMotionStatus.msg"
  "msg/ArmPowerStatus.msg"
  "msg/ArmControlModeStatus.msg"
  "msg/EndEffectorPose.msg"
  "msg/CartesianExecutionStatus.msg"
  "msg/PlannerInput.msg"
  "msg/PlannerDecision.msg"
  "msg/WorkspaceControl.msg"
  "msg/WorkspaceStatus.msg"
  "msg/JointBatchExecutionStatus.msg"
  "msg/LiftStatus.msg"
  "msg/HeavyUpperBodyGatewayCommandV1.msg"
  "msg/HeavyUpperBodyGatewayFeedbackV1.msg"
  "msg/HeavyUpperBodyGatewayStatusV1.msg"
  "srv/SetRobotPower.srv"
  "srv/SetArmControlMode.srv"
  "srv/CartesianIncrementControl.srv"
  "srv/CartesianAbsoluteControl.srv"
  "srv/CartesianPathAbsoluteControl.srv"
  "srv/CartesianPathIncrementControl.srv"
  "srv/JointAbsoluteControl.srv"
  "srv/SelectedJointControl.srv"
  "srv/JointBatchControl.srv"
  "srv/ActuatorControl.srv"
  "srv/GetHeavyGatewayCapabilitiesV1.srv"
  "srv/AcquireHeavyExecutionLeaseV1.srv"
  "srv/ReleaseHeavyExecutionLeaseV1.srv"
  "srv/SetHeavyLiftBrakeV1.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

find_package(rosidl_default_runtime REQUIRED)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(robot_power_service src/robot_power_service.cpp)
target_include_directories(robot_power_service PRIVATE include)
ament_target_dependencies(robot_power_service
  rclcpp
  std_msgs
  rosidl_default_runtime
)
target_link_libraries(robot_power_service "${cpp_typesupport_target}")

add_executable(arm_control_mode_service src/arm_control_mode_service.cpp)
target_include_directories(arm_control_mode_service PRIVATE include)
ament_target_dependencies(arm_control_mode_service
  rclcpp
  rosidl_default_runtime
)
target_link_libraries(arm_control_mode_service "${cpp_typesupport_target}")

install(TARGETS
  robot_power_service
  arm_control_mode_service
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  add_test(
    NAME lift_status_interface_hash
    COMMAND ${CMAKE_COMMAND}
      -DSOURCE_MSG=${CMAKE_CURRENT_SOURCE_DIR}/msg/LiftStatus.msg
      -DINSTALL_MSG=${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/msg/LiftStatus.msg
      -DEXPECTED_HASH=${LIFT_STATUS_INTERFACE_SHA256}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/check_lift_status_hash.cmake)
endif()

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
