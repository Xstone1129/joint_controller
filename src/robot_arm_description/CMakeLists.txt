cmake_minimum_required(VERSION 3.10.2)

project(robot_arm_description)

# 查找所需的包
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 安装 Python 启动文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装其他资源
install(DIRECTORY 
    config
    meshes
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
