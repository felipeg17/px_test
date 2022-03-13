# Python 2/3 compatibility imports
from __future__ import print_function
# from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import time
from tf.transformations import *
from spatialmath import *
from spatialmath.base import *
import roboticstoolbox as rtb


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

np.set_printoptions(suppress=True)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python", anonymous=False)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "px_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_planner_id("PRMkConfigDefault")
move_group.set_planning_time(5)

# Joint positions
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0
joint_goal[3] = 0
# Joint motion
move_group.go(joint_goal, wait=True)
move_group.stop()

joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -np.pi/6
joint_goal[2] = -np.pi/2
joint_goal[3] = -1*np.pi/3
# Joint motion
move_group.go(joint_goal, wait=True)
move_group.stop()

time.sleep(0.1)
print(move_group.get_current_joint_values())
print(move_group.get_current_pose().pose)
current_pose = moveit_commander.conversions.pose_to_list(move_group.get_current_pose().pose)
# print(current_pose[3:])
euler_angles = euler_from_quaternion(current_pose[3:])
# print(euler_angles)
# print(euler_matrix(euler_angles[0],euler_angles[1],euler_angles[2]))
origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
R1 = rotation_matrix(np.pi/2,xaxis)
# print(R1)
R2 = rotation_matrix(np.pi,yaxis)
# print(R2)
R = concatenate_matrices(R2,R1)
# print(R)
qua_goal = quaternion_from_matrix(R)
# print(qua_goal)
# WS Motion
# pose_goal = geometry_msgs.msg.Pose()
# pose_goal.orientation.x = qua_goal[0]
# pose_goal.orientation.y = qua_goal[1]
# pose_goal.orientation.z = qua_goal[2]
# pose_goal.orientation.w = qua_goal[3]
# pose_goal.orientation.x = current_pose[0]
# pose_goal.orientation.y = current_pose[1]
# pose_goal.orientation.z = current_pose[2]
# pose_goal.orientation.w = current_pose[3]
# pose_goal.position.x = 0.12
# pose_goal.position.y = 0
# pose_goal.position.z = 0.09
# move_group.set_pose_target(pose_goal)
# plan = move_group.go(wait=True)
# move_group.stop()
# move_group.clear_pose_targets()

# time.sleep(0.1)
# print(rotx(np.pi/2)@roty(2*np.pi/3))
# print(rotx(np.pi/2).dot(roty(2*np.pi/3)))
# print(SO3.Rx(np.pi/2)*(SO3.Ry(2*np.pi/3)))

# R = trotx(np.pi/2)@troty(np.pi)
# qua_goal = quaternion_from_matrix(R)
# current_pose = moveit_commander.conversions.pose_to_list(move_group.get_current_pose().pose)
# pose_goal = geometry_msgs.msg.Pose()
# pose_goal.orientation.x = current_pose[3]
# pose_goal.orientation.y = current_pose[4]
# pose_goal.orientation.z = current_pose[5]
# pose_goal.orientation.w = current_pose[6]
# pose_goal.position.x = 0.15
# pose_goal.position.y = 0
# pose_goal.position.z = 0.09
# move_group.set_pose_target(pose_goal)
# plan = move_group.go(wait=True)
# move_group.stop()
# move_group.clear_pose_targets()


current_pose = moveit_commander.conversions.pose_to_list(move_group.get_current_pose().pose)
pose_goal = geometry_msgs.msg.Pose()
R = trotx(np.pi/2)@troty(np.pi)
qua_goal = quaternion_from_matrix(R)
pose_goal.orientation.x = qua_goal[0]
pose_goal.orientation.y = qua_goal[1]
pose_goal.orientation.z = qua_goal[2]
pose_goal.orientation.w = qua_goal[3]
pose_goal.position.x = current_pose[0]
pose_goal.position.y = current_pose[1]
pose_goal.position.z = current_pose[2]
move_group.set_pose_target(pose_goal)
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

# # Cartesian Motion
waypoints = []
wpose = move_group.get_current_pose().pose
scale = 1
wpose.position.z -= scale * 0.05  # First move up (z)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x -= scale * 0.05  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))


(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
print(fraction)

# # Display planned motion (replay)
# display_trajectory = moveit_msgs.msg.DisplayTrajectory()
# display_trajectory.trajectory_start = robot.get_current_state()
# display_trajectory.trajectory.append(plan)
# display_trajectory_publisher.publish(display_trajectory)

# # Execute motion
move_group.execute(plan, wait=True)

# # Add new objects
# box_pose = geometry_msgs.msg.PoseStamped()
# box_pose.header.frame_id = "link_6"
# box_pose.pose.orientation.w = 1.0
# box_pose.pose.position.x = 0.1  # above the panda_hand frame
# box_name = "box"
# scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))
# time.sleep(0.5)

# grasping_group = "irb120_arm"
# touch_links = robot.get_link_names(group=grasping_group)
# scene.attach_box(eef_link, box_name, touch_links=touch_links)
# time.sleep(0.1)

# waypoints = []
# wpose = move_group.get_current_pose().pose
# wpose.position.x -= scale * 0.1 
# waypoints.append(copy.deepcopy(wpose))
# (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
# move_group.execute(plan, wait=True)

# scene.remove_attached_object(eef_link, name=box_name)
# time.sleep(0.5)


# waypoints = []
# wpose = move_group.get_current_pose().pose
# wpose.position.x -= scale * 0.1 
# waypoints.append(copy.deepcopy(wpose))
# (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
# move_group.execute(plan, wait=True)

# time.sleep(0.5)
# scene.remove_world_object(box_name)

