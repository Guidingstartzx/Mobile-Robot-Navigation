import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = "four_wheel_car"
    share_dir = Path(get_package_share_directory(package_name))
    prefix_dir = Path(get_package_prefix(package_name))
    
    # 1. 核心路径定义
    world_file = share_dir / "worlds" / "ackerman_smallroom.world"
    urdf_file = share_dir / "urdf" / "ackerman_car.urdf.xacro"
    model_path = share_dir / "models"
    plugin_path = prefix_dir / "lib"

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 我们为阿克曼小车定制的 Nav2 参数文件路径
    nav2_ackerman_params_teb_path = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_ackerman_params_teb.yaml')

    # 2. 解析机器人 Xacro 描述
    robot_description = ParameterValue(
        Command(["xacro", " ", str(urdf_file)]),
        value_type=str,
    )

    return LaunchDescription([
        # 3. 注入 Gazebo 环境变量，确保能找到小车模型和 C++ 插件
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[str(model_path), ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")]
        ),
        SetEnvironmentVariable(
            name="GZ_SIM_SYSTEM_PLUGIN_PATH",
            value=[str(plugin_path), ":", EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value="")]
        ),

        # 4. 启动机器人状态发布器 (发布机器人各关节的静态 TF 变换)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}, {"use_sim_time": True}],
        ),

        # 5. 正确启动 Gazebo 并强制加载包含雷达和小车的 smallroom.world
        ExecuteProcess(
            cmd=["gz", "sim", "-r", str(world_file)],
            output="screen",
        ),

        # 6. 数据桥接器 (极简版)
        # 注意：C++底层插件已自带ROS 2节点并发布了 /clock, /odom, /joint_states 和 TF
        # 所以这里【绝对不能】重复桥接，只桥接标准的雷达传感器数据即可！
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/scan_gz@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
            ],
            remappings=[
                ('/scan_gz', '/scan')
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # 7. 启动 SLAM 建图节点 (负责接收 /scan 并发布 map -> odom 的 TF 树)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'slam_launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # 8. 启动 Nav2 导航核心栈 (加载我们的专属阿克曼参数)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_ackerman_params_teb_path  # 加载修改过速度和转弯半径的配置文件
            }.items()
        ),

        # 9. 启动专为 Nav2 优化的 RViz 视图
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])