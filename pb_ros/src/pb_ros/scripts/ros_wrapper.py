#!/usr/bin/env python3
'''
This program is the ROS wrapper of pybullet simulataor.

'''
import rospy
# from pybullet_class import ur_robot
from conversions import RosConvert
from topics import joint_state,ArmCommand,actionserver
import numpy as np
import skimage
import ipdb
import cv_bridge
from threading import Thread
from sensor_msgs.msg import JointState, Image, CameraInfo

from cam_conversions import to_camera_info_msg
from pybullet_class import BtCamera
# from .pybullet_class import CameraIntrinsic

#path = "/home/amrut/ROS_Workspaces/pb_ros/src/pb_ros/urdf/urdf/real_arm.urdf"
# ur5 = ur_robot(path)

class ROS_Wrapper:
	def __init__(self,path,gui=True):
		self.bullet_obj = RosConvert(path,gui)
		self.joint_state = joint_state(rosconvert_object=self.bullet_obj)
		self.ArmCommand = ArmCommand(rosconvert_object=self.bullet_obj)
		self.actionserver = actionserver(rosconvert_object=self.bullet_obj)
		self.plugin = CameraPlugin(BtCamera(320, 240, 0.96, 0.01, 1.0, self.bullet_obj.robot.arm_id, 6))
		# self.plugin = CameraPlugin(self.bullet_obj.robot.camera)
		self.plugin.thread.start()
		
		self.actionserver.start()
		

	def main_loop(self):
		# ipdb.set_trace()
		self.joint_state.publish()

class Plugin:
    """A plugin that spins at a constant rate in its own thread."""

    def __init__(self, rate):
        self.rate = rate
        self.thread = Thread(target=self.loop, daemon=True)
        self.is_running = False

    def activate(self):
        self.is_running = True

    def deactivate(self):
        self.is_running = False

    def loop(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.is_running:
                self.update()
            rate.sleep()

    def update(self):
        raise NotImplementedError


class CameraPlugin(Plugin):
    def __init__(self, camera, name="camera", rate=5):
        super().__init__(rate)
        self.camera = camera
        self.name = name
        self.cam_noise = rospy.get_param("~cam_noise", False)
        self.cv_bridge = cv_bridge.CvBridge()
        self.init_publishers()

    def init_publishers(self):
        topic = self.name + "/depth/camera_info"
        self.info_pub = rospy.Publisher(topic, CameraInfo, queue_size=10)
        topic = self.name + "/depth/image_rect_raw"
        self.depth_pub = rospy.Publisher(topic, Image, queue_size=10)

    def update(self):
        stamp = rospy.Time.now()
        msg = to_camera_info_msg(self.camera.intrinsic)
        # msg = self.camera.intrinsic
        msg.header.frame_id = self.name + "_optical_frame"
        msg.header.stamp = stamp
        self.info_pub.publish(msg)

        # ipdb.set_trace()
        _, depth, _ = self.camera.get_image()

        if self.cam_noise:
            depth = apply_noise(depth)

        msg = self.cv_bridge.cv2_to_imgmsg((1000 * depth).astype(np.uint16))
        msg.header.stamp = stamp
        self.depth_pub.publish(msg)

def apply_noise(img, k=1000, theta=0.001, sigma=0.005, l=4.0):
    # Multiplicative and additive noise
    img *= np.random.gamma(k, theta)
    h, w = img.shape
    noise = np.random.randn(int(h / l), int(w / l)) * sigma
    img += skimage.transform.resize(noise, img.shape, order=1, mode="constant")
    return img



path = rospy.get_param('/pb_ros/robot_urdf_path',True)
rospy.init_node("Node",anonymous=True)
ros = ROS_Wrapper(path,gui=True)
rate = rospy.Rate(1000)
while not rospy.is_shutdown():
	ros.main_loop()
	ros.bullet_obj.robot.step()
	# ipdb.set_trace()
	ros.plugin.activate()
	rate.sleep()


