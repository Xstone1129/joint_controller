cmake_minimum_required(VERSION 3.5)
project(erobot_igh_driver)

set(CMAKE_BUILD_TYPE Debug)  # or Release, RelWithDebInfo, etc.
#find_package(ament_cmake REQUIRED)

# EtherLab
set(ETHERLAB_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(ETHERCAT_LIB ethercat HINTS ${ETHERLAB_DIR}/lib)

set(IGH_COMMON_INCLUDE_DIRS
  ${ETHERLAB_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(IGH_DRIVER_SOURCES
  src/igh_driver.cpp
  src/igh_driver_runtime.cpp
  src/igh_driver_state_machine.cpp
  src/igh_driver_flow.cpp
  src/igh_driver_master.cpp
)

add_executable(
  igh_driver
  ${IGH_DRIVER_SOURCES}
)

target_compile_features(igh_driver PRIVATE cxx_std_14)
  
target_include_directories(
  igh_driver
  PRIVATE
  ${IGH_COMMON_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(igh_driver ${ETHERCAT_LIB})

add_executable(
  igh_sensor_driver
  src/igh_sensor_driver.cpp
)

target_compile_features(igh_sensor_driver PRIVATE cxx_std_14)

target_include_directories(
  igh_sensor_driver
  PRIVATE
  ${IGH_COMMON_INCLUDE_DIRS}
)

target_link_libraries(igh_sensor_driver ${ETHERCAT_LIB})
