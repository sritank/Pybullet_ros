#!/usr/bin/env python

import numpy as np
import rospy
import time
import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint,JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult


class actionserver:
    def __init__(self):
        self.goal = FollowJointTrajectoryGoal()
        self.result = FollowJointTrajectoryResult()
        self.feedback = FollowJointTrajectoryFeedback()
        self.JointTraj = JointTrajectory()
        self.F = Float64MultiArray()
        # Topic 
        self.arm_pub = rospy.Publisher("/arm_controller/command",Float64MultiArray,
                                        latch=True,queue_size=1)
    def callback(self,goal):
        self.JointTraj = self.goal.trajectory
        # print(self.JointTraj)
        print("yuppppppppppppppppppp")
        self.extract_joint_vals(goal)
    def extract_joint_vals(self,goal):
        self.joint_vals = [] 
        print(goal.trajectory.points)
        print("len",self.JointTraj.points) 
        for i in range(len(goal.trajectory.points)):
            print("eeeeeeeeeeeeeeeeee")
            val = [list(goal.trajectory.points[i].positions), list(goal.trajectory.points[i].velocities)]
            self.joint_vals.append(val)
        self.execute()
        server.set_succeeded(self.result)
    def execute(self):
        for i in range(len(self.joint_vals)):
            print(self.joint_vals[i][0])
            # print("Chhhhhhhhhhhhhhhheeeeeeeeeee")
            self.publish(self.joint_vals[i][0] + [0.0,0.0])
        print("Done")
    def publish(self,value):
        self.F.data = list(value)
        self.arm_pub.publish(self.F)


    




rospy.init_node("moveit_action_server")
obj = actionserver()
server = actionlib.SimpleActionServer('arm_controller/follow_joint_trajectory',
                                    FollowJointTrajectoryAction,obj.callback,False)
server.start()
rospy.spin()
