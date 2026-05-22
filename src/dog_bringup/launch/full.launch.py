# 导入库
import os
import tempfile
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.actions import DeclareLaunchArgument,TimerAction,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit,OnProcessStart
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("dog_bringup")
    prefix = "xterm -hold -e"

    declare_xacro = DeclareLaunchArgument(
        name = 'xacro_path',
        default_value=pkg_share + "/resource/model/dog.xacro"
    )
    xacro_path = LaunchConfiguration("xacro_path")

    # 生成带正确路径的 controller 参数 YAML
    manager_template = os.path.join(pkg_share, "launch", "dog_controller.yaml")
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

    zeroSetter_node = Node(
        package="dog_hardware",
        executable="zeroSetter",
        output = "screen"
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

    # ZeroSetter 写入的是 install 空间的 motors.yaml，
    # 此处回调将其同步回 src 源目录，避免 colcon build 后丢失校准结果
    install_motors = os.path.join(pkg_share, "parameters", "motors.yaml")
    src_motors = os.path.join(
        os.path.dirname(__file__), "..", "parameters", "motors.yaml"
    )
    src_motors = os.path.abspath(src_motors)

    sync_motors_yaml = ExecuteProcess(
        cmd=["cp", install_motors, src_motors],
        name="sync_motors_yaml",
        output="screen"
    )

    event_handler_sync = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=zeroSetter_node,
            on_exit=[sync_motors_yaml]
        )
    )

    event_handler2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controler_spawner,
            on_exit=[web_bridge_node]
        )
    )
    
    ld = LaunchDescription([
            declare_xacro,
            declare_manager,

            statepub_node,
            zeroSetter_node,
            event_handler_sync,
            event_handler0,
            event_handler1,
            event_handler2,
        ])
    
    return ld