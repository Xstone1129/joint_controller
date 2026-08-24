cmake_minimum_required(VERSION 3.8)
project(erobot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CONTROLLER_DEPENDS
  ament_index_cpp
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  robot_control_msg
  urdf
  Eigen3
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/ruckig)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/ruckig/httplib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/ruckig/json)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/ruckig/nlohmann)

find_library(
  RUCKIG_LIB
  NAMES ruckig
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/lib/ruckig
  NO_DEFAULT_PATH
)

find_package(Eigen3 REQUIRED)
set(ROS_HUMBLE_PINOCCHIO_DIR "/opt/ros/humble/lib/x86_64-linux-gnu/cmake/pinocchio")
if(EXISTS "${ROS_HUMBLE_PINOCCHIO_DIR}/pinocchioConfig.cmake")
  set(pinocchio_DIR "${ROS_HUMBLE_PINOCCHIO_DIR}" CACHE PATH "Pinocchio CMake package path" FORCE)
endif()
find_package(pinocchio REQUIRED CONFIG)
find_package(urdfdom REQUIRED)
include_directories(${urdfdom_INCLUDE_DIRS})

find_package(ament_cmake REQUIRED)
foreach(DEP ${CONTROLLER_DEPENDS})
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(/usr/include/eigen3)
include_directories(/usr/include/eigen3/Eigen)
include_directories(/usr/include/eigen3/Eigen/src)

if(Eigen3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIRS})
endif()

add_library(i_traj ${CMAKE_CURRENT_SOURCE_DIR}/src/traj.cpp)
set_target_properties(i_traj PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(i_traj ${RUCKIG_LIB})
target_include_directories(i_traj PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(i_traj PRIVATE WITH_CLOUD_CLIENT)

add_library(erobot_controller SHARED
  src/erobot_controller_arm.cpp
  src/CommandProcessor_arm.cpp
  src/CubicPlanner.cpp
  ../thirdparty/fileoption/fileoption.cpp
)

target_compile_features(erobot_controller PUBLIC cxx_std_17)

target_include_directories(erobot_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(erobot_controller PUBLIC
  ${CONTROLLER_DEPENDS}
)

target_link_libraries(erobot_controller PRIVATE
  i_traj
  ${RUCKIG_LIB}
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  urdfdom::urdfdom_model
)

target_compile_definitions(erobot_controller PRIVATE
  "EROBOT_CONTROLLER_BUILDING_DLL"
)

pluginlib_export_plugin_description_file(
  controller_interface
  erobot_controller_plugin.xml
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  erobot_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch/
  config/
  DESTINATION share/${PROJECT_NAME}
)

install(FILES erobot_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_heavy_arm_stream test/test_heavy_arm_stream.cpp)
  target_link_libraries(test_heavy_arm_stream erobot_controller)
  ament_target_dependencies(test_heavy_arm_stream
    controller_interface
    hardware_interface
    rclcpp
  )

  install(TARGETS
    erobot_controller
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(RUCKIG_LIB)
  get_filename_component(RUCKIG_ABS_PATH ${RUCKIG_LIB} ABSOLUTE)
  install(FILES ${RUCKIG_ABS_PATH}
    DESTINATION lib
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${CONTROLLER_DEPENDS})
ament_package()
