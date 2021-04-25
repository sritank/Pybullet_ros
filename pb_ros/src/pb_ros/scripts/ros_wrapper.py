#!/usr/bin/env python3
'''
This program is the ROS wrapper of pybullet simulataor.

'''
import rospy
# from pybullet_class import ur_robot
from conversions import RosConvert
from topics import joint_state,ArmCommand,actionserver



#path = "/home/amrut/ROS_Workspaces/pb_ros/src/pb_ros/urdf/urdf/real_arm.urdf"
# ur5 = ur_robot(path)

class ROS_Wrapper:
	def __init__(self,path,gui=True):
		self.bullet_obj = RosConvert(path,gui)
		self.joint_state = joint_state(rosconvert_object=self.bullet_obj)
		self.ArmCommand = ArmCommand(rosconvert_object=self.bullet_obj)
		self.actionserver = actionserver(rosconvert_object=self.bullet_obj)
		self.actionserver.start()

	def main_loop(self):
		self.joint_state.publish()

path = rospy.get_param('/pb_ros/robot_urdf_path',True)
rospy.init_node("Node",anonymous=True)
ros = ROS_Wrapper(path,gui=True)
rate = rospy.Rate(1000)
while not rospy.is_shutdown():
	ros.main_loop()
	ros.bullet_obj.robot.step()
	rate.sleep()


