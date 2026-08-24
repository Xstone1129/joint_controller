cmake_minimum_required(VERSION 3.8)
project(erobot_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(
  Robot_arm_hardware_interface
  SHARED
  src/erobot_hw_arm.cpp
)

target_compile_features(Robot_arm_hardware_interface PUBLIC cxx_std_17)

target_include_directories(Robot_arm_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(Robot_arm_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_definitions(Robot_arm_hardware_interface PRIVATE
  "ROBOT_ARM_HARDWARE_INTERFACE_BUILDING_DLL"
)

install(
  DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(
  hardware_interface
  Robot_arm_hardware_interface.xml
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_arm_axis_mapping test/test_arm_axis_mapping.cpp)
  if(TARGET test_arm_axis_mapping)
    target_include_directories(test_arm_axis_mapping PRIVATE include)
  endif()
endif()

install(TARGETS
  Robot_arm_hardware_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
