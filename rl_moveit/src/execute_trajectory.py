#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander #allow us to communicate with the MoveIt Rviz interface
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("manipulator")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
#Next, we are getting the current values of the joints.
group_variable_values = group.get_current_joint_values()
#Now, we modify the value of 2 of the joints, and set this new joint value as a target.
group_variable_values[3] = 1.5
group_variable_values[5] = 1.5
group.set_joint_value_target(group_variable_values)

#Finally, we just compute the plan for the new joint space goal.
plan2 = group.plan()
#By executing this line of code, you will be telling your robot to execute the last trajectory that has been set for the Planning Group
group.go(wait=True)
#You can get the reference frame for a certain group by executing this line:
print "Reference frame: %s" % group.get_planning_frame()
#You can get the end-effector link for a certaing group executing this line:
print "End effector: %s" % group.get_end_effector_link()
#You can get a list with all the groups of the robot like this:
print "Robot Groups:"
print robot.get_group_names()
#You can get the current values of the joints like this:
print "Current Joint Values:"
print group.get_current_joint_values()
#You can also get the current Pose of the end-effector of the robot like this:
print "Current Pose:"
print group.get_current_pose()
#Finally, you can check the general status of the robot like this:
print "Robot State:"
print robot.get_current_state()

rospy.sleep(5)
#At the end, we just shut down the moveit_commander module.
moveit_commander.roscpp_shutdown()