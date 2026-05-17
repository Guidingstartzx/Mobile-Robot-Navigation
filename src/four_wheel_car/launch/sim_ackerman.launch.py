from pathlib import Path

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = "four_wheel_car"
    share_dir = Path(get_package_share_directory(package_name))
    prefix_dir = Path(get_package_prefix(package_name))

    world_file = share_dir / "worlds" / "ackerman_world.sdf"
    urdf_file = share_dir / "urdf" / "ackerman_car.urdf.xacro"
    model_path = share_dir / "models"
    plugin_path = prefix_dir / "lib"
    rviz_config = share_dir / "config" / "four_wheel_car.rviz"

    gz_args = LaunchConfiguration("gz_args")
    start_teleop = LaunchConfiguration("teleop")
    start_rviz = LaunchConfiguration("rviz")

    robot_description = ParameterValue(
        Command(["xacro", " ", str(urdf_file)]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_args",
                default_value="-r",
                description="Arguments passed to gz sim before the world file.",
            ),
            DeclareLaunchArgument(
                "teleop",
                default_value="false",
                description=(
                    "Start keyboard teleop inside this launch. Leave false and run "
                    "teleop.launch.py from an interactive terminal for normal use."
                ),
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz with robot and trajectory marker displays.",
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    str(model_path),
                    ":",
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                value=[
                    str(plugin_path),
                    ":",
                    EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description},
                    {"use_sim_time": True},
                ],
            ),
            ExecuteProcess(
                cmd=["gz", "sim", gz_args, str(world_file)],
                output="screen",
            ),
            Node(
                package=package_name,
                executable="keyboard_teleop",
                name="keyboard_teleop",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(start_teleop),
                parameters=[
                    {"cmd_vel_topic": "/cmd_vel"},
                    {"linear_step": 0.10},
                    {"angular_step": 0.20},
                    {"max_linear": 1.50},
                    {"max_angular": 2.00},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(rviz_config)],
                output="screen",
                condition=IfCondition(start_rviz),
            ),
        ]
    )
