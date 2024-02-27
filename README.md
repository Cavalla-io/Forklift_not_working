# How to recreate error

First clone the package and then build it. 
dependencies are:  
  joint_state_publisher
  robot_localization
  robot_state_publisher
  ros2_control
  ros2_controllers
  rosidl_default_generators
  std_msgs
  rosidl_default_runtime
  rosidl_interface_packages
  ament_lint_auto
  ament_lint_common

Folders to ignore:
 forklift_control, forklift_description, and forklift_gazebo: leftover from a scavenged github repo

## To launch the sim
Run `ros2 launch basic_mobile_robot3 basic_mobile_bot_v5.launch.py slam:=True`
