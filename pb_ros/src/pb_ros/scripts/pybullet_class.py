#!/usr/bin/env python3

import numpy as np
import pybullet as p
import pybullet_data
import time
import ipdb

from spatial import Rotation, Transform
import open3d as o3d
from perception import CameraIntrinsic
# p.connect(p.GUI)
# pid = p.isConnected()
# print(pid)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# robot=p.loadURDF("/home/amrut/Documents/python files/ur_description/urdf/ur5_robot.urdf",basePosition=[0.,0.,0.],baseOrientation=[0.,0.0,0.,1],useFixedBase=1)
# plane = p.loadURDF("plane.urdf",basePosition=[0,0,0],useFixedBase=1) 
# robot = p.loadURDF("/home/amrut/Documents/sublime/src/ur5pybullet/urdf/real_arm.urdf",basePosition=[0.,0.,1.0],baseOrientation=[0.,0.0,0.,1],useFixedBase=1)
#path = "/home/amrut/Documents/sublime/src/ur5pybullet/urdf/real_arm.urdf"
'''
 ur_robot is the main pybullet object on which the entire package has been built.
 This object loads the robot, extract its joint and linkstates. It can apply force
 or torque on joints. Presently it supports position and velocity controllers.
'''
class ur_robot:
	def __init__(self,path,gui=True,basePosition=[0,0,0],baseOrientation=[0,0,0,1],useFixedBase=1):
		self.start_bullet(gui=gui) # If true enables Graphical interface.
		# Loads the robot and a plane at desired position and orientation
		self.robot = p.loadURDF(path,basePosition=basePosition,baseOrientation=baseOrientation,useFixedBase=1)
		self.arm_id = 0
		self._plane = p.loadURDF("plane.urdf",basePosition=[0,0,-2],baseOrientation=[0,0,0,1],useFixedBase=1)
		self._numjoints = p.getNumJoints(self.robot)
		self._linkNames = []
		self._jointNames = []
		self._jointTypes = []
		# p.resetJointState(self.robot,jointIndex=1,targetValue=1)
		self.getJointNames()
		self._controllers = []
		self._simRate = 100
		self._joints_pos = []
		self._joints_vel = []
		self._joints_eff = []
		# self.camera = BtCamera(p, 320, 240, 0.96, 0.01, 1.0, 0, 5)
		# print("instantiated camera with and bullet object view matrix is")
		# print(self.camera.proj_mat)
		self.setGravity()

	def getJointNames(self):
		''' The JointVal must be an integer'''
		jointTypes = ["JOINT_REVOLUTE", "JOINT_PRISMATIC", "JOINT_SPHERICAL", "JOINT_PLANAR", "JOINT_FIXED"]
		self._revotuteJoints = []
		self._revotuteJointNames = []
		for i in range(self._numjoints):
			data = p.getJointInfo(self.robot,jointIndex=i)
			# print(data)
			self._linkNames.append(data[-5].decode("utf-8"))
			self._jointNames.append(data[1].decode("utf-8"))
			self._jointTypes.append(jointTypes[int(data[2])])
			if (int(data[2]) != 4):
				self._revotuteJoints.append(i)
				self._revotuteJointNames.append(self._jointNames[i]) 
		# print(self._jointNames,self._revotuteJoints)
	
	def resetJointState(self,jointName,jointVal):
		Jointindex = self._jointNames.index(jointName)
		p.resetJointState(self.robot,jointIndex=Jointindex,targetValue=1)
		print("Succesfully Reset joint {} to Joint value {}".format(jointName, jointVal))
	
	def resetJointStates(self,jointValues):
		jointNames = self._revotuteJoints
		print(jointNames)
		for i in range(len(jointNames)):
			p.resetJointState(self.robot, jointIndex=jointNames[i], targetValue=jointValues[i])
		
		print("success\n")

	def resetBasePosOrn(self,basePos,baseOrn):
		''' Reset Robot Base postion and Orientation'''
		p.resetBasePositionAndOrientation(self.robot,basePosition=basePos,baseOrientation=baseOrn)

	def getBasePosOrn(self):
		pos,orn = p.getBasePositionAndOrientation(self.robot)
		return (pos,orn)

	def singleJointMotorControl(self,jointName,jointVal,controllerType):
		''' Controller Types are position or velocity'''
		Jointindex = self._jointNames.index(jointName)

		if (controllerType == 'position'):
			p.setJointMotorControl2(self.robot,jointIndex=Jointindex,targetPosition=jointVal,controlMode=p.POSITION_CONTROL)
		else:
			p.setJointMotorControl2(self.robot,jointIndex=Jointindex,targetPosition=jointVal,controlMode=p.VELOCITY_CONTROL)
		for _ in range(self._simRate):
			p.stepSimulation()
			# time.sleep(0.0001)

	def allJointMotorControl(self,JointPoses=None,JointVels=None,controllerType='position'):

		if (controllerType == 'position'):

			if JointVels == None:
				p.setJointMotorControlArray(self.robot,
				                        jointIndices=self._revotuteJoints[:6],
				                        targetPositions=JointPoses,
										controlMode=p.POSITION_CONTROL)
			else:
				p.setJointMotorControlArray(self.robot,
				                        jointIndices=self._revotuteJoints[:6],
				                        targetPositions=JointPoses,
										targetVelocities=JointVels,
										controlMode=p.POSITION_CONTROL)
										# forces=6*[10],
										# positionGains=6*[1],
										# velocityGains=6*[0.5])
		else:
			p.setJointMotorControlArray(self.robot,
				                        jointIndices=self._revotuteJoints,
				                        targetVelocities=JointVels,
										controlMode=p.VELOCITY_CONTROL)
		
		for _ in range(self._simRate):
			p.stepSimulation()
			time.sleep(0.0001)
		# print(self.getLinkStates())

	def setGravity(self,val=[0,0,-9.81]):
		''' Gravity value in a list'''
		p.setGravity(*val)
		print("Set Gravity value as {}".format(val))

	def getJointStates(self):
		""" Get Joints position and velocity """

		for i in self._revotuteJoints:
			data = p.getJointState(self.robot,jointIndex=i)
			self._joints_pos.append(data[0])
			self._joints_vel.append(data[1])
			self._joints_eff.append(data[3])
		val = (self._revotuteJointNames,self._joints_pos,self._joints_vel)
		self._joints_pos,self._joints_vel,self._joints_eff = [],[],[]
		# print(val)
		return val

	def start_bullet(self,gui=True):
		if (gui == True):
			p.connect(p.GUI)
		else:
			p.connect(p.DIRECT)
			pass
		p.setAdditionalSearchPath(pybullet_data.getDataPath())
		# p.resetDebugVisualizerCamera(1.2, 30, -30, [0.4, 0.0, 0.2])

	def step(self):
		p.stepSimulation()

	def getLinkStates(self):
		pos ,orn = [],[]
		for i in range(len(self._linkNames)):
			data = p.getLinkState(self.robot,linkIndex=i,computeForwardKinematics=1)
			pos.append(data[-2])
			orn.append(data[-1])
		
	def getInitialLinkStates(self):
		self.initial_linkstates = self.getLinkStates()

# path = "/home/amrut/Documents/sublime/src/ur5pybullet/urdf/real_arm.urdf"	
# ur5 = ur_robot(path)
# # ur5.allJointMotorControl([0.1,0,0,0,0,0,0,0])
# # time.sleep(2)
# # ur5.resetJointStates([2,0,0,0,0,0,0,0])
# # ur5.resetJointState(jointName='shoulder_lift_joint',jointVal=-1)
# ur5.getJointStates()
# # print(p.getLinkStates(ur5.robot,linkIndices=list(range(ur5._numjoints))))

# while (1):
# 	ur5.getLinkStates()
# 	pass

class BtCamera:
    def __init__(
        self, 
        width,
        height,
        vfov,
        near,
        far,
        body_uid=None,
        link_id=None,
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
    ):
        f, cx, cy = height / (2.0 * np.tan(vfov / 2.0)), width / 2.0, height / 2.0
        self.intrinsic = CameraIntrinsic(width, height, f, f, cx, cy)
        self.near = near
        self.far = far
        fov, aspect = np.rad2deg(vfov), width / height
        self.proj_mat = p.computeProjectionMatrixFOV(fov, aspect, near, far)
        self.body_uid = body_uid
        self.link_id = link_id
        self.renderer = renderer

    def get_image(self, pose=None):
        # ipdb.set_trace()
        # print(self.bullet_obj.getLinkState(self.body_uid, self.link_id, computeForwardKinematics=1))
        # print(p.getLinkState(self.body_uid, self.link_id, computeForwardKinematics=1))
        if pose is None:
            # ipdb.set_trace()
            r = p.getLinkState(self.body_uid, self.link_id, computeForwardKinematics=1)
            pose = Transform(Rotation.from_quat(r[5]), r[4])
        R, t = pose.rotation, pose.translation
        # view_mat = p.computeViewMatrix(t, R.apply([0, 0, 1]) + t, R.apply([0, -1, 0]))
        view_mat = p.computeViewMatrix(t, R.apply([1, 0, 0]), R.apply([0, 0, -1]))
        # view_mat = p.computeViewMatrix(cameraEyePosition=[0, 0, 1],
        #                                   cameraTargetPosition=[0, 0, 0],
        #                                   cameraUpVector=[1, 0, 0])
        result = p.getCameraImage(
            self.intrinsic.width,
            self.intrinsic.height,
            view_mat,
            self.proj_mat,
            renderer=self.renderer,
        )		

        color = result[2][:, :, :3]
        depth = self.far * self.near / (self.far - (self.far - self.near) * result[3])
        mask = result[4]
        return color, depth, mask