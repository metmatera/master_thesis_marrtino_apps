=== robot_teleop.launch ===
- arduino_robot
- keyboard/Joystick_control

$ roslaunch robot_teleop.launch (keyboard control)
$ roslaunch robot_teleop.launch use_joystick:=true (joystick control)


=== robot_teleop_nav.launch ===
- arduino_robot
- keyboard/Joystick_control
- xtion
- depth2laser
- gradient_based_navigation

$ roslaunch robot_teleop_nav.launch  (joystick control)
$ roslaunch robot_teleop_nav.launch use_joystick:=false (keyboard control)
$ rosrun rviz rviz -d robot_nav.rviz

=== remote teleop ===

(DLA wireless) 192.168.0.102 

export ROS_MASTER_URI=http://raspi302:11311/
rosbag record -o dellbox_luca /scan /odom /cmd_vel


