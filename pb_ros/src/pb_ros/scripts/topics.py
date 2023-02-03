#!/usr/bin/env python3

import rospy
import time
import actionlib
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint,JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import JointTolerance


class joint_state:
    # This class publishes the jointstates
    def __init__(self,rosconvert_object):
        self.obj = rosconvert_object
        self._pub_jointstate = rospy.Publisher("joint_states",JointState,
                                                latch=True,queue_size=1)
    
    def publish(self):
        val = self.obj.getJointState()
        self._pub_jointstate.publish(val)
# This is the class for executing controller action
class ArmCommand:
    def __init__(self,rosconvert_object):
        self.obj = rosconvert_object
        self._sub = rospy.Subscriber("/arm_controller/command",
                                      Float64MultiArray,callback=self.callback)
    def callback(self,data):
        self.obj.armcommand(data.data)
# The actionserver class that moveit subscribes.
class actionserver:
    def __init__(self,rosconvert_object):
        self._simTime = 1/240
        self.goal = FollowJointTrajectoryGoal()                #Goal
        self.result = FollowJointTrajectoryResult()            # The final result
        self.feedback = FollowJointTrajectoryFeedback()     # Feedback after each
                                                            # execution
    
        self._error = JointTrajectoryPoint()                # Error 
        self._joint_tol = JointTolerance()                  # Joint Tolerance
        self.F = Float64MultiArray()                        
        # Topic 
        self.obj = rosconvert_object                        # rosconversion object
        # Ros action server
        self.server = actionlib.SimpleActionServer('arm_controller/follow_joint_trajectory',
                                    FollowJointTrajectoryAction,self.callback,False)
        self.feedback.joint_names = self.obj.robot._jointNames
        self.server.start()
    def callback(self,goal):
        # self.JointTraj = self.goal.trajectory
        # print("The Goal =",goal)
        # print("=================================")
        # Get start time
        # print(goal)
        self.start_time = time.time()
        for i in range(len(goal.trajectory.points)):
            # print("Hii")
            # self._joint_tol.position = goal.path_tolerance[i].position
            # self._joint_tol.velocity = goal.path_tolerance[i].velocity
            # self._joint_tol.acceleration = goal.path_tolerance[i].acceleration
            self.__pos_err = goal.trajectory.points[i].positions
            self.__vel_err = goal.trajectory.points[i].velocities
            
            self.obj.Armcommand(goal.trajectory.points[i].positions,     # publish both
                                    goal.trajectory.points[i].velocities) # pos and vel
                
            # print("Feedback",rospy.wait_for_message("/joint_states",JointState))
            self.feedback.desired = goal.trajectory.points[i]
            self.send_feedback(i)
            # print("--------------------------------")
            # print("Feedback= ",self.feedback)
        rospy.loginfo("Successfully Reached Goal")
        self.result.error_code = 0      # set error code for successful execution
        self.server.set_succeeded(self.result,"Successfully Reached Goal (:")
    def start(self):
        self.server.start()
    def send_feedback(self,count):   # function fo sending feedback
        data = rospy.wait_for_message("/joint_states",JointState)
        pos = data.position[:6]
        vel = data.velocity[:6]
        self.feedback.actual.positions = data.position
        self.feedback.actual.velocities = data.velocity
        self.feedback.error.positions = np.array(self.__pos_err) - np.array(pos)
        self.feedback.error.velocities = np.array(self.__vel_err) - np.array(vel)
        self.feedback.error.accelerations = 6*[0.0]
        self.feedback.header.seq = count
        self.feedback.header.stamp = rospy.get_rostime()

        duration = time.time() - self.start_time
        self.feedback.desired.time_from_start = rospy.Duration(duration)
        self.feedback.actual.time_from_start = rospy.Duration(duration)
        self.feedback.error.time_from_start = rospy.Duration(duration)
        
        if self.check_tolerance():
            self.server.publish_feedback(self.feedback)
        else:
            self.result.error_code = -4
            self.server.set_aborted(self.result,"Oops some error occurred ):")
    
    def check_tolerance(self): # not implemented
        # if np.max(self.feedback.error.positions) < self._joint_tol.position and np.max(self.feedback.error.velocities) < self._joint_tol.velocity :
        #     return True
        return True
    def abort_action(self):
        self.result.error_code = -1
        self.server.set_aborted(self.result,"OOps some error occurred")
    