# Compile
colcon build --symlink-install --cmake-clean-cache --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DPython3_EXECUTABLE=/usr/bin/python3


# Run
ros2 launch four_wheel_car sim_ackerman.launch.py rviz:=true
ros2 run four_wheel_car keyboard_teleop 


# Navigation
### Ackerman
- RPP: ros2 launch four_wheel_car nav_ackerman.launch.py 
- TEB: ros2 launch four_wheel_car nav_ackerman_teb.launch.py

### Mecanum
ros2 launch four_wheel_car nav_mecanum.launch.py 