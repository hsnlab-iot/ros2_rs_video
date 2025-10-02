StampedCode ROS2 message package.

Message:
- std_msgs/Header header
- builtin_interfaces/Time stamp
- uint32 code

Build:
  cd /home/ubuntu/ros2_ws
  colcon build --packages-select stamped_code_msgs
  source install/setup.bash