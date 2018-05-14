#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_bit', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
#Next, we are getting the current values of the joints.
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
rospy.sleep(5)

moveit_commander.roscpp_shutdown()
