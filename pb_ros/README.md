The package is a integration of ROS,moveit and pybullet for a UR5 industrial manipulator. This is just a experimental demonstration. It is far from complete in terms of both execution and documentation. Many services and topics need to implemented along with camera and gripper functioning. The controller needs further tuning. To start the the robot run these commands in separate terminal.

This will launch ur5 in pybullet windows.
1--- roslaunch pb_ros bringup_ur5.launch

This is for moveit.
2--- roslaunch ur_moveit move_group.launch

This will open rviz
3--- roslaunch ur_moveit moveit_rviz.launch config:=true

