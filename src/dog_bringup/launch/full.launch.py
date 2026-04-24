# 导入库
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.actions import DeclareLaunchArgument,TimerAction,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit,OnProcessStart
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    prefix = "xterm -hold -e"

    declare_xacro = DeclareLaunchArgument(
        name = 'xacro_path',
        default_value=get_package_share_directory("dog_bringup") + "/resource/model/dog.xacro"
    )
    xacro_path = LaunchConfiguration("xacro_path")

    declare_manager = DeclareLaunchArgument(
        name = 'manager_path',
        default_value=get_package_share_directory("dog_bringup") + "/launch/dog_controller.yaml"
    )
    manager_path = LaunchConfiguration("manager_path")

    robot_description_content = ParameterValue(
                Command(["xacro ", xacro_path]),
                value_type=str
    )

    zeroSetter_node = Node(
        package="dog_hardware",
        executable="zeroSetter",
        output = "screen"
    )

    statepub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
        output="screen"
        )
    # 发布机器人的urdf描述，提供给ros2 control

    controler_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[manager_path],
    )
    # 加载控制管理器

    controler_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["base_controller"],
    )
    # 加载主控制器

    event_handler0 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=zeroSetter_node,
            on_exit=[controler_manager]
        )
    )

    event_handler1 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controler_manager,
            on_start=[controler_spawner]
        )
    )
    
    ld = LaunchDescription([
            declare_xacro,
            declare_manager,

            statepub_node,
            zeroSetter_node,
            event_handler0,
            event_handler1,
        ])
    
    return ld