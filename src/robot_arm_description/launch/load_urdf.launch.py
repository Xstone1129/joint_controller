import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro  # 导入xacro模块

def generate_launch_description():
    # 设置包名和路径
    package_name = 'robot_arm_description'
    urdf_file_name = 'right_left_arm.urdf'  # 替换为您的实际URDF文件名
    # 注意：如果使用xacro，则文件扩展名可能是.xacro，这里假设已经处理成.urdf或者使用xacro文件

    # 获取包路径
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_file_path = os.path.join(pkg_share, 'urdf', urdf_file_name)
    
    # 使用xacro处理URDF文件（如果文件是xacro格式）
    # 如果是纯URDF文件，我们也可以直接读取，但为了通用性，我们使用xacro处理（即使是URDF，处理也不会改变）
    doc = xacro.process_file(urdf_file_path)
    robot_description_content = doc.toprettyxml(indent='  ')
    
    # 配置参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config = LaunchConfiguration('rviz_config')
    gui = LaunchConfiguration('gui', default='true')
    
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'view_robot.rviz')
    
    # 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file'
    )
    
    declare_gui_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
    
    # 机器人状态发布节点
    params = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # # 根据参数选择GUI或非GUI关节状态发布器
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher_gui' if gui == 'true' else 'joint_state_publisher',
    #     executable='joint_state_publisher_gui' if gui == 'true' else 'joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[{'source_list': ['joint_states']}]
    # )
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )   
    # RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # 创建启动描述
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_rviz_config_cmd,
        declare_gui_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
