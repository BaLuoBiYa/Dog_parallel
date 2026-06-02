# 导入库
import os
import tempfile
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,TimerAction,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("dog_bringup")
    prefix = "xterm -hold -e"

    declare_xacro = DeclareLaunchArgument(
        name = 'xacro_path',
        default_value=pkg_share + "/resource/model/dog_single.xacro"
    )
    xacro_path = LaunchConfiguration("xacro_path")

    # 生成带正确路径的 controller 参数 YAML
    manager_template = os.path.join(pkg_share, "launch", "dog_controller_single.yaml")
    with open(manager_template, 'r') as f:
        raw = f.read()
    raw = raw.replace('$(find dog_bringup)', pkg_share)
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    tmp.write(raw)
    tmp.close()
    manager_path = tmp.name

    declare_manager = DeclareLaunchArgument(
        name = 'manager_path',
        default_value=manager_path
    )

    robot_description_content = ParameterValue(
                Command(["xacro ", xacro_path]),
                value_type=str
    )

    web_bridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        output = "screen"
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
            web_bridge_node,
            timeLine0,
            timeLine1,
            timeLine2
        ])
    
    return ld