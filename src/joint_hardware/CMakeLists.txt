cmake_minimum_required(VERSION 3.8)
project(joint_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  hardware_interface
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  realtime_tools
  robot_control_msg
  std_msgs
  std_srvs
  trajectory_msgs
  urdf
)

find_package(ament_cmake REQUIRED)
foreach(DEPENDENCY IN LISTS THIS_PACKAGE_INCLUDE_DEPENDS)
  find_package(${DEPENDENCY} REQUIRED)
endforeach()
find_package(Threads REQUIRED)

# EtherLab/IgH is an optional open-source master interface.  The package still
# builds with the Mock and Unavailable backends when the development headers or
# library are absent.  A vendor SDK is not required by this adapter.
option(JOINT_HARDWARE_ENABLE_ETHERLAB "Build the EtherLab/IgH backend" ON)
if(JOINT_HARDWARE_ENABLE_ETHERLAB)
  find_package(EtherCAT QUIET CONFIG
    PATHS /usr/local/etherlab/lib/cmake/ethercat
  )
endif()

add_library(${PROJECT_NAME} SHARED
  src/joint_hardware.cpp
  src/hardware_state_bus.cpp
  src/protocol/waist_can.cpp
  src/transport/usb_bridge_hub.cpp
  src/lift/ethercat_backend.cpp
  src/lift/ethercat_sdk_adapter.cpp
  src/lift/etherlab_backend.cpp
  src/lift/ethercat_frame.cpp
  src/lift/ethercat_coe.cpp
  src/lift/pdo_mapping.cpp
  src/lift/cia402.cpp
  src/lift/feedback_continuity.cpp
  src/lift/lift_units.cpp
  src/lift/zero_offset_store.cpp
  src/lift/lift_hardware.cpp
  src/waist/waist_hardware.cpp
  src/waist/waist_init.cpp
  src/waist/waist_read.cpp
  src/waist/waist_write.cpp
  src/waist/waist_utils.cpp
  src/waist/waist_transport.cpp
  src/waist/waist_canopen_io.cpp
  src/waist/waist_feedback.cpp
)

find_library(
  RUCKIG_LIBRARY
  NAMES ruckig
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/lib/ruckig
  NO_DEFAULT_PATH
  REQUIRED
)

add_library(joint_lift_controller SHARED
  src/lift/lift_controller.cpp
  src/lift/lift_motion_profile.cpp
)
target_compile_features(joint_lift_controller PUBLIC cxx_std_17)
target_include_directories(joint_lift_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(joint_lift_controller PUBLIC
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  realtime_tools
  robot_control_msg
  std_msgs
  std_srvs
  trajectory_msgs
)
target_link_libraries(joint_lift_controller PRIVATE ${RUCKIG_LIBRARY})
set_property(
  TARGET joint_lift_controller APPEND PROPERTY INSTALL_RPATH
  "$ORIGIN"
)

add_executable(lift_keyboard_teleop
  src/lift/lift_keyboard_teleop.cpp
)
target_compile_features(lift_keyboard_teleop PUBLIC cxx_std_17)
ament_target_dependencies(lift_keyboard_teleop
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

if(TARGET EtherLab::ethercat AND EtherCAT_INCLUDE_DIR AND EtherCAT_LIBRARY)
  target_compile_definitions(${PROJECT_NAME} PRIVATE JOINT_HARDWARE_HAS_ETHERLAB=1)
  target_link_libraries(${PROJECT_NAME} PRIVATE EtherLab::ethercat)
  get_filename_component(ETHERLAB_LIBRARY_DIR "${EtherCAT_LIBRARY}" DIRECTORY)
  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY INSTALL_RPATH "${ETHERLAB_LIBRARY_DIR}")
  message(STATUS "joint_hardware: EtherLab/IgH backend enabled")
else()
  message(STATUS "joint_hardware: EtherLab/IgH backend disabled (Mock/Unavailable only)")
endif()

pluginlib_export_plugin_description_file(
  hardware_interface
  joint_hardware_plugin.xml
)
pluginlib_export_plugin_description_file(
  controller_interface
  joint_lift_controller_plugin.xml
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS joint_lift_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES ${RUCKIG_LIBRARY}
  DESTINATION lib
)

install(
  TARGETS lift_keyboard_teleop
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    joint_hardware_plugin.xml
    joint_lift_controller_plugin.xml
    README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config description
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/lift_ethercat_preflight.sh
    scripts/lift_ethercat_host_setup.sh
    scripts/robot_ethercat_host_setup.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # xmllint needs the ROS schema over the network. Keep package tests offline.
  set(AMENT_LINT_AUTO_EXCLUDE "ament_cmake_uncrustify;ament_cmake_xmllint")
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_lift_units test/test_lift_units.cpp)
  target_link_libraries(test_lift_units ${PROJECT_NAME})
  target_include_directories(test_lift_units PRIVATE ${PROJECT_SOURCE_DIR}/include)

  ament_add_gtest(test_feedback_continuity test/test_feedback_continuity.cpp)
  target_link_libraries(test_feedback_continuity ${PROJECT_NAME})
  target_include_directories(test_feedback_continuity PRIVATE ${PROJECT_SOURCE_DIR}/include)

  ament_add_gtest(test_lift_motion_profile
    test/test_lift_motion_profile.cpp
    src/lift/lift_motion_profile.cpp
  )
  target_include_directories(test_lift_motion_profile PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty
  )
  target_link_libraries(test_lift_motion_profile ${RUCKIG_LIBRARY})

  ament_add_gtest(test_lift_controller test/test_lift_controller.cpp)
  target_link_libraries(test_lift_controller joint_lift_controller)
  target_include_directories(test_lift_controller PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty
  )
  ament_target_dependencies(test_lift_controller
    controller_interface
    hardware_interface
    rclcpp
    robot_control_msg
    std_msgs
    std_srvs
    trajectory_msgs
  )
  set_tests_properties(
    test_lift_controller
    PROPERTIES ENVIRONMENT
    "ROS_LOG_DIR=/tmp/joint_hardware_ros_logs;ROS_DOMAIN_ID=155;ROS_LOCALHOST_ONLY=1")

  ament_add_gtest(test_cia402 test/test_cia402.cpp)
  target_link_libraries(test_cia402 ${PROJECT_NAME})
  target_include_directories(test_cia402 PRIVATE ${PROJECT_SOURCE_DIR}/include)

  ament_add_gtest(test_zero_offset test/test_zero_offset.cpp)
  target_link_libraries(test_zero_offset ${PROJECT_NAME})
  target_include_directories(test_zero_offset PRIVATE ${PROJECT_SOURCE_DIR}/include)

  ament_add_gtest(test_mock_lift_integration test/test_mock_lift_integration.cpp)
  target_link_libraries(test_mock_lift_integration ${PROJECT_NAME})
  target_include_directories(test_mock_lift_integration PRIVATE ${PROJECT_SOURCE_DIR}/include)
  set_tests_properties(
    test_mock_lift_integration
    PROPERTIES ENVIRONMENT
    "ROS_LOG_DIR=/tmp/joint_hardware_ros_logs;ROS_DOMAIN_ID=156;ROS_LOCALHOST_ONLY=1")

  ament_add_gtest(test_ethercat_frames test/test_ethercat_frames.cpp)
  target_link_libraries(test_ethercat_frames ${PROJECT_NAME})
  target_include_directories(test_ethercat_frames PRIVATE ${PROJECT_SOURCE_DIR}/include)

  ament_add_gtest(test_waist_can test/test_waist_can.cpp)
  target_link_libraries(test_waist_can ${PROJECT_NAME})
  target_include_directories(test_waist_can PRIVATE ${PROJECT_SOURCE_DIR}/include)

  ament_add_gtest(test_usb_bridge_hub test/test_usb_bridge_hub.cpp)
  target_link_libraries(test_usb_bridge_hub ${PROJECT_NAME} Threads::Threads)
  target_include_directories(test_usb_bridge_hub PRIVATE ${PROJECT_SOURCE_DIR}/include)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
