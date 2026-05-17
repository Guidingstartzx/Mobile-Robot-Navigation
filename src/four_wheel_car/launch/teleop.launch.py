from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    teleop_command = [
        "ros2 run four_wheel_car keyboard_teleop --ros-args ",
        "-p cmd_vel_topic:=/cmd_vel ",
        "-p linear_step:=", LaunchConfiguration("linear_step"), " ",
        "-p angular_step:=", LaunchConfiguration("angular_step"), " ",
        "-p max_linear:=", LaunchConfiguration("max_linear"), " ",
        "-p max_angular:=", LaunchConfiguration("max_angular"), "; ",
        "exec bash",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("linear_step", default_value="0.10"),
            DeclareLaunchArgument("angular_step", default_value="0.20"),
            DeclareLaunchArgument("max_linear", default_value="1.50"),
            DeclareLaunchArgument("max_angular", default_value="2.00"),
            ExecuteProcess(
                cmd=[
                    "x-terminal-emulator",
                    "-e",
                    "bash",
                    "-lc",
                    teleop_command,
                ],
                output="screen",
            ),
        ]
    )
