cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_control_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty)
find_library(
  RUCKIG_LIB
  NAMES ruckig
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/lib/ruckig
  NO_DEFAULT_PATH
)

add_executable(joint_absolute_control_srv
  src/joint_absolute_control_srv.cpp
)

add_executable(joint_batch_control_srv
  src/joint_batch_control_srv.cpp
)

add_executable(cartesian_increment_control_srv
  src/cartesian_increment_control_srv.cpp
)

add_executable(cartesian_absolute_control_srv
  src/cartesian_absolute_control_srv.cpp
)

add_executable(cartesian_single_control_srv
  src/cartesian_single_control_srv.cpp
)

add_executable(cartesian_path_absolute_control_srv
  src/cartesian_path_absolute_control_srv.cpp
)

add_executable(cartesian_path_increment_control_srv
  src/cartesian_path_increment_control_srv.cpp
)

add_executable(cartesian_moveL_path
  src/arm_ik_moveL_path.cpp
)

add_executable(planner_input_publisher_ubuntu
  src/planner_input_publisher_ubuntu.cpp
)

# The supervisor is split by theme; the file list must stay in sync with the
# implementation notes in include/robot_control/workspace_supervisor_ubuntu.hpp.
add_executable(workspace_supervisor_ubuntu
  src/workspace_supervisor_ubuntu.cpp
  src/workspace_supervisor_systemd.cpp
  src/workspace_supervisor_probe.cpp
  src/workspace_supervisor_readiness.cpp
)

target_include_directories(workspace_supervisor_ubuntu PRIVATE include)

foreach(target_name
    joint_absolute_control_srv
    joint_batch_control_srv
    cartesian_increment_control_srv
    cartesian_absolute_control_srv
    cartesian_single_control_srv
    cartesian_path_absolute_control_srv
    cartesian_path_increment_control_srv
    cartesian_moveL_path)
  target_include_directories(${target_name} PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../robot_control_msg/include
    ${EIGEN3_INCLUDE_DIRS}
  )
endforeach()

target_link_libraries(joint_absolute_control_srv
  ${pinocchio_LIBRARIES}
)
ament_target_dependencies(joint_absolute_control_srv
  rclcpp
  robot_control_msg
  sensor_msgs
)

target_link_libraries(joint_batch_control_srv
  ${pinocchio_LIBRARIES}
)
ament_target_dependencies(joint_batch_control_srv
  rclcpp
  robot_control_msg
  sensor_msgs
)

target_link_libraries(cartesian_increment_control_srv
  ${pinocchio_LIBRARIES}
)
ament_target_dependencies(cartesian_increment_control_srv
  rclcpp
  robot_control_msg
  sensor_msgs
)

target_link_libraries(cartesian_absolute_control_srv
  ${pinocchio_LIBRARIES}
)
ament_target_dependencies(cartesian_absolute_control_srv
  rclcpp
  robot_control_msg
  sensor_msgs
)

target_link_libraries(cartesian_single_control_srv
  ${pinocchio_LIBRARIES}
)
ament_target_dependencies(cartesian_single_control_srv
  rclcpp
  robot_control_msg
  sensor_msgs
)

target_link_libraries(cartesian_path_absolute_control_srv
  ${pinocchio_LIBRARIES}
)
ament_target_dependencies(cartesian_path_absolute_control_srv
  rclcpp
  robot_control_msg
  geometry_msgs
  sensor_msgs
)

target_link_libraries(cartesian_path_increment_control_srv
  ${pinocchio_LIBRARIES}
)
ament_target_dependencies(cartesian_path_increment_control_srv
  rclcpp
  robot_control_msg
  geometry_msgs
  sensor_msgs
)

target_link_libraries(cartesian_moveL_path
  ${pinocchio_LIBRARIES}
  ${RUCKIG_LIB}
)
ament_target_dependencies(cartesian_moveL_path
  rclcpp
  robot_control_msg
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

ament_target_dependencies(planner_input_publisher_ubuntu
  rclcpp
  robot_control_msg
)

ament_target_dependencies(workspace_supervisor_ubuntu
  rclcpp
  robot_control_msg
  std_msgs
)

install(TARGETS
  joint_absolute_control_srv
  joint_batch_control_srv
  cartesian_increment_control_srv
  cartesian_absolute_control_srv
  cartesian_single_control_srv
  cartesian_path_absolute_control_srv
  cartesian_path_increment_control_srv
  cartesian_moveL_path
  planner_input_publisher_ubuntu
  workspace_supervisor_ubuntu
  DESTINATION lib/${PROJECT_NAME}
)

if(RUCKIG_LIB)
  get_filename_component(RUCKIG_ABS_PATH ${RUCKIG_LIB} ABSOLUTE)

  install(FILES ${RUCKIG_ABS_PATH}
    DESTINATION lib/${PROJECT_NAME}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  )
endif()

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_workspace_supervisor_policy
    test/test_workspace_supervisor_policy.cpp
  )
  if(TARGET test_workspace_supervisor_policy)
    target_include_directories(test_workspace_supervisor_policy PRIVATE include)
    ament_target_dependencies(test_workspace_supervisor_policy robot_control_msg)
  endif()

  ament_add_gtest(test_joint_completion_policy
    test/test_joint_completion_policy.cpp
  )
  if(TARGET test_joint_completion_policy)
    target_include_directories(test_joint_completion_policy PRIVATE include)
  endif()
endif()

ament_package()
