cmake_minimum_required(VERSION 3.16)
project(four_wheel_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  four_wheel_car/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_teleop
)

install(DIRECTORY
  config
  launch
  models
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

set(GZ_SIM_TARGET "")
set(GZ_PLUGIN_TARGET "")
set(GZ_TRANSPORT_TARGET "")
set(GZ_MSGS_TARGET "")

foreach(ver 10 9 8)
  if(NOT GZ_SIM_TARGET)
    find_package(gz-sim${ver} QUIET)
    if(gz-sim${ver}_FOUND)
      set(GZ_SIM_TARGET gz-sim${ver}::gz-sim${ver})
      message(STATUS "Found Gazebo Sim target: ${GZ_SIM_TARGET}")
    endif()
  endif()
endforeach()

foreach(ver 3 2)
  if(NOT GZ_PLUGIN_TARGET)
    find_package(gz-plugin${ver} QUIET)
    if(gz-plugin${ver}_FOUND)
      set(GZ_PLUGIN_TARGET gz-plugin${ver}::gz-plugin${ver})
      message(STATUS "Found Gazebo Plugin target: ${GZ_PLUGIN_TARGET}")
    endif()
  endif()
endforeach()

foreach(ver 15 14 13 12)
  if(NOT GZ_TRANSPORT_TARGET)
    find_package(gz-transport${ver} QUIET)
    if(gz-transport${ver}_FOUND)
      set(GZ_TRANSPORT_TARGET gz-transport${ver}::gz-transport${ver})
      message(STATUS "Found Gazebo Transport target: ${GZ_TRANSPORT_TARGET}")
    endif()
  endif()
endforeach()

foreach(ver 12 11 10 9)
  if(NOT GZ_MSGS_TARGET)
    find_package(gz-msgs${ver} QUIET)
    if(gz-msgs${ver}_FOUND)
      set(GZ_MSGS_TARGET gz-msgs${ver}::gz-msgs${ver})
      message(STATUS "Found Gazebo Msgs target: ${GZ_MSGS_TARGET}")
    endif()
  endif()
endforeach()

if(GZ_SIM_TARGET AND GZ_PLUGIN_TARGET AND GZ_TRANSPORT_TARGET AND GZ_MSGS_TARGET)
  add_library(four_wheel_car_system SHARED
    src/four_wheel_car_system.cpp
  )

  target_compile_features(four_wheel_car_system PUBLIC cxx_std_17)
  target_link_libraries(four_wheel_car_system
    ${GZ_SIM_TARGET}
    ${GZ_PLUGIN_TARGET}
    ${GZ_TRANSPORT_TARGET}
    ${GZ_MSGS_TARGET}
  )
  ament_target_dependencies(four_wheel_car_system
    builtin_interfaces
    rclcpp
    geometry_msgs
    nav_msgs
    rosgraph_msgs
    sensor_msgs
    std_msgs
    tf2_ros
    visualization_msgs
  )

  install(TARGETS four_wheel_car_system
    LIBRARY DESTINATION lib
  )
else()
  message(WARNING
    "Gazebo Sim development libraries were not found. "
    "The ROS package will build, but libfour_wheel_car_system.so will not be produced. "
    "Install Gazebo Harmonic / gz-sim dev libraries before running the simulation.")
endif()

ament_package()
