#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander #allow us to communicate with the MoveIt Rviz interface
import moveit_msgs.msg
import geometry_msgs.msg

#Here, we are just initializing the moveit_commander module.
moveit_commander.roscpp_initialize(sys.argv)
#Here, we are just initializing a ROS node.
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
#Here, we are creating a RobotCommander object, which basically is an interface to our robot.
robot = moveit_commander.RobotCommander()
#Here, we creating a PlanningSceneInterface object, which basically is an interface to the world that surrounds the robot.
scene = moveit_commander.PlanningSceneInterface() 
#Here, we create a MoveGroupCommander object, which is an interface to the manipulator group of joints 
group = moveit_commander.MoveGroupCommander("manipulator")
#Here we are defining a Topic Publisher, which will publish into the /move_group/display_planned_path topic. By publishing into this topic, we will be able to visualize the planned motion through the MoveIt Rviz interface.
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
#Here we are creating a Pose object, which is the type of message that we will send as a goal. Then, we just give values to the variables that will define the goal Pose
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.3
pose_target.position.y = 0
pose_target.position.z = 1.1
group.set_pose_target(pose_target)
#Finally, we are telling the "manipulator" group we created previously, to calculate the plan.
plan1 = group.plan()

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