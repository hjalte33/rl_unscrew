#!/usr/bin/env python

import sys
import time
import moveit_commander
import moveit_msgs.msg
import rospy

rospy.init_node('robot_start_config_node', anonymous=True)

#Setup moveit for interfacing and controlling the UR5 robot in Gazebo
moveit_commander.roscpp_initialize(sys.argv)

#creating a RobotCommander object, which acts as an interface to our robot.
robot = moveit_commander.RobotCommander()

#creating a PlanningSceneInterface object, which is an interface to the world that surrounds the robot.
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")

group_variable_values = group.get_current_joint_values()
#Now, we modify the value of 1 of the joints, and set this new joint value as a target.
group_variable_values[0] = 0
group_variable_values[1] = -1.5707
group_variable_values[2] = 1.5707
group_variable_values[3] = -1.5707
group_variable_values[4] = -1.5707
group_variable_values[5] = 0
group.set_joint_value_target(group_variable_values)
#Finally, we just compute the plan for the new joint space goal.
plan2 = group.plan()
#By executing this line of code, you will be telling your robot to execute the last trajectory that has been set for the Planning Group
group.go(wait=True)
time.sleep(5)

current_pose = group.get_current_pose()
current_pose.pose.position.x = 0.5
current_pose.pose.position.y = 0.2
current_pose.pose.position.z = 0.25

group.set_pose_target(current_pose)
group.plan()

#executes the trajectory that has been set for the Planning Group
group.go(wait=True)
time.sleep(5)