# 导入库
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,TimerAction,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    prefix = "xterm -hold -e"

    declare_xacro = DeclareLaunchArgument(
        name = 'xacro_path',
        default_value=get_package_share_directory("dog_bringup") + "/resource/model/dog_single.xacro"
    )
    xacro_path = LaunchConfiguration("xacro_path")

    declare_manager = DeclareLaunchArgument(
        name = 'manager_path',
        default_value=get_package_share_directory("dog_bringup") + "/launch/dog_controller_single.yaml"
    )
    manager_path = LaunchConfiguration("manager_path")

    robot_description_content = ParameterValue(
                Command(["xacro ", xacro_path]),
                value_type=str
    )

    statepub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
        remappings=[("robot_description", "/controller_manager/robot_description")],
        output="screen"
        )
    # 发布机器人的urdf描述，提供给ros2 control

    controler_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[manager_path],
        output="screen"
    )
    # 加载控制管理器

    controler_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["base_controller"],
        output="screen"
    )
    # 加载主控制器


    timeLine0 = TimerAction(
        period=0.0,
        actions=[statepub_node],
    )

    timeLine1 = TimerAction(
        period=1.0,
        actions=[controler_manager],
    )

    timeLine2 = TimerAction(
        period=2.0,
        actions=[controler_spawner],
    )
    
    ld = LaunchDescription([
            declare_xacro,
            declare_manager,
            timeLine0,
            timeLine1,
            timeLine2
        ])
    
    return ld