# four_wheel_car

ROS 2 Jazzy + Gazebo Sim demo for a simple four-wheel car:

- WASD keyboard commands publish center-of-mass velocity on `/cmd_vel`.
- Front wheels have steering and rolling DOF, and are the only active drive wheels.
- Rear wheels only roll passively.
- The Gazebo system plugin runs closed-loop PD torque control:
  `tau = kp * position_error + kd * velocity_error`.
- The plugin publishes `/joint_states`, `/odom`, `/imu/data`, `/tf`, and `/com_trajectory_marker`.
- It also publishes a Gazebo `/marker` line strip so the COM trajectory is drawn inside Gazebo.

## Ubuntu 24.04 / ROS 2 Jazzy dependencies

The package builds its ROS nodes without Gazebo development files, but the simulation plugin needs
Gazebo Sim development libraries. For the standard Jazzy + Gazebo Harmonic setup:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop xacro gz-harmonic libgz-sim8-dev libgz-transport13-dev libgz-msgs10-dev
```

If `gz sim -h` prints only the generic `gz` help and the command list contains only `help` and
`sdf`, Gazebo Sim is not installed. Install `gz-harmonic` first, then rebuild this workspace.
If apt cannot find `gz-harmonic`, add the Gazebo apt repository:

```bash
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install gz-harmonic
```

## Build

```bash
cd ~/four_wheel_demo
colcon build --symlink-install
source install/setup.bash
```

If Anaconda / Miniconda `base` is active, deactivate it before building. ROS 2 Jazzy and Gazebo
Harmonic on Ubuntu 24.04 should use the system Python and protobuf:

```bash
conda deactivate
cd ~/four_wheel_demo
colcon build --symlink-install --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

If your shell still injects Anaconda paths, use a clean build environment:

```bash
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_EXE -u CONDA_SHLVL \
  PATH=/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  colcon build --symlink-install --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

If CMake prints that Gazebo Sim development libraries were not found, install the Gazebo packages
above and rebuild. The runtime model expects `libfour_wheel_car_system.so`.

## Run

```bash
ros2 launch four_wheel_car sim.launch.py
```

Run teleop directly in a second interactive terminal:

```bash
source ~/four_wheel_demo/install/setup.bash
ros2 run four_wheel_car keyboard_teleop
```

Or ask launch to open a new terminal window for teleop:

```bash
source ~/four_wheel_demo/install/setup.bash
ros2 launch four_wheel_car teleop.launch.py
```

The launch form uses `x-terminal-emulator`, so it needs a desktop session. On SSH or headless
systems, use `ros2 run` directly.


Controls:

- `w` / `s`: increase or decrease linear x velocity.
- `a` / `d`: increase or decrease yaw rate.
- `space`: clear all velocity commands to zero.
- `q`: quit teleop.

Optional RViz view:

```bash
ros2 launch four_wheel_car sim.launch.py rviz:=true
```



#
colcon build --symlink-install --cmake-clean-cache --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DPython3_EXECUTABLE=/usr/bin/python3

ros2 launch four_wheel_car sim_ackerman.launch.py rviz:=true
ros2 run four_wheel_car keyboard_teleop 
