import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# Initialize moveit commander, node, planning scene and move_group
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander("arm")

# Set a goal position
pose_goal = geometry_msgs.msg.Pose()
# pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.position.y = 0.1
pose_goal.position.z = 0.4

# move manipulator to the desired goal.
move_group.set_pose_target(pose_goal)
print(move_group.plan())

# The planned motion object conatining the trajectory .
# Move to the desired pose. 
plan = move_group.go(wait=True)
# Clear the target from buffer
move_group.clear_pose_targets()



# joint_goal = move_group.get_current_joint_values()
# print(joint_goal)
# joint_goal[0] = 0
# joint_goal[1] = -pi/4
# joint_goal[2] = 0
# joint_goal[3] = -pi/2
# joint_goal[4] = 0
# joint_goal[5] = pi/3
# # joint_goal[6] = 0


# # The go command can be called with joint values, poses, or without any
# # parameters if you have already set the pose or joint target for the group
# move_group.go(joint_goal, wait=True)
# joint_goal = move_group.get_current_joint_values()
# print(joint_goal)


# get the current position.
current_pose = move_group.get_current_pose()
print(current_pose)