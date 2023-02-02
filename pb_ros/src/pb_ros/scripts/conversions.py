#!/usr/bin/env python3
'''
This program converts the output of pybullet simulator into ROS message type.
This can even extract the data from ROS messages and pass it to the Pybullet
class.
'''
import numpy as np
import rospy
from pybullet_class import ur_robot
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from std_msgs.msg import Float64MultiArray
from pybullet_class import BtCamera


# path = "/home/amrut/Documents/sublime/src/ur5pybullet/urdf/real_arm.urdf"
class RosConvert(object):
	def __init__(self,path,gui=True):
		# super().__init__(ur_robot)
		self.loop_rate = 10
		self.count = 0
		self.robot = ur_robot(path,gui) # Passes robot path to pybullet class.
		self.jointstate = JointState()
		# self.arm_id = 0
		# self.camera = BtCamera(320, 240, 0.96, 0.01, 1.0, 0, 11)
		self.armcmd = Float64MultiArray() # ROS msg type to contain joint values
		                                  # for applying on joint controllers. 
		
	def getJointState(self,rate=20):  # Publish joint states
		val = self.robot.getJointStates()
		self.count += 1
		self.jointstate.header.seq = self.count
		self.jointstate.header.stamp = rospy.get_rostime()
		self.jointstate.name = val[0]
		self.jointstate.position = val[1]
		self.jointstate.velocity = val[2]
		return self.jointstate
	def armcommand(self,poses,vels=None): # The controller command,
		# Can provide position alone or both joint positions and velocities.
		# val = self.armcommand()
		self.robot.allJointMotorControl(JointPoses=poses,
		                                JointVels=vels,controllerType="position")
	def Armcommand(self,poses,vels=None):
		# The moveit alters the order of joints when it publishes data. This function
		# swaps the elbow joint the third position and third position to first.
		poses = list(poses)
		vels = list(vels)
		
		temp1 = poses[0]
		poses[0] = poses[2]
		poses[2] = temp1
		if vels != None:
			temp1 = vels[0]
			vels[0] = vels[2]
			vels[2] = temp1
		self.robot.allJointMotorControl(JointPoses=poses,
		                                JointVels=vels,controllerType="position")

									

	# def getTransforms(self):
	# 	linkstate = self.robot.getLinkStates()
	# 	# trans = []
	# 	for i in range(len(linkstate[0])-3):
	# 		# print(linkstate[0][i])
	# 		self.transforms.header.seq = 0
	# 		# self.transforms.header.stamp = rospy.Time.now()
	# 		self.transforms.header.frame_id = linkstate[0][i]

	# 		self.transforms.child_frame_id = linkstate[0][i+1]

	# 		self.transforms.transform.translation.x = linkstate[1][i][0]
	# 		self.transforms.transform.translation.y = linkstate[1][i][1]
	# 		self.transforms.transform.translation.z = linkstate[1][i][2]

	# 		self.transforms.transform.rotation.x = linkstate[2][i][0]
	# 		self.transforms.transform.rotation.y = linkstate[2][i][1]
	# 		self.transforms.transform.rotation.z = linkstate[2][i][2]
	# 		self.transforms.transform.rotation.x = linkstate[2][i][0]

	# 		self.tf.transforms.append(self.transforms)
	# 		# self.tf.transforms[i] = temp
	# 		# print("tttttttttttttttttttt i ",self.transforms)
	# 		# print(self.transforms)
	# 		# trans.append(self.transforms)
	# 		# print(trans)
		
	# 	i=0
	# 	# self.tf = trans
	# 	val = self.tf
	# 	# print("fff",val)
	# 	# print("sssssssssssssssssss",val)
	# 	self.tf = TFMessage()
	# 	self.transforms = TransformStamped()
	# 	return val
	# def tf_process(self):
	# 	for i in range(7):
	# 		val = TFMessage()
	# 		self.tf.transforms.append(val)
	




# rospy.init_node('my_node_name', anonymous=True)
# rc = RosConvert(path)
# rc.getJointState()
# rc.armcommand([1,0,0,0,0,0,0,0])
# while not rospy.is_shutdown():
# 	rc.getJointState()
# 	print("Hello")

