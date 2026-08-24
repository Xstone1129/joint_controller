cmake_minimum_required(VERSION 3.8)
project(teleop_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



set(CONTROLLER_DEPENDS
  rclcpp
  robot_control_msg
  std_msgs
  geometry_msgs
  sensor_msgs
)

# 查找Pinocchio和Eigen依赖
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# ruckig headers & lib
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty)
find_library(RUCKIG_LIB 
    NAMES ruckig
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/lib/ruckig
    NO_DEFAULT_PATH
)
# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
foreach(DEP ${CONTROLLER_DEPENDS})
  find_package(${DEP} REQUIRED)
endforeach()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(joy REQUIRED)
find_package(robot_control_msg REQUIRED)


# 安装Python节点
install(PROGRAMS
  src/ik_arm_control.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
