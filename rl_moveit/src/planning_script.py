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
group = moveit_commander.MoveGroupCommander("arm") #should be tool
#Here we are defining a Topic Publisher, which will publish into the /move_group/display_planned_path topic. By publishing into this topic, we will be able to visualize the planned motion through the MoveIt Rviz interface.
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
#Here we are creating a Pose object, which is the type of message that we will send as a goal. Then, we just give values to the variables that will define the goal Pose
pose_target = geometry_msgs.msg.Pose()
pose_target.position.x = 0.5
pose_target.position.y = 0
pose_target.position.z = 0.01
group.set_pose_target(pose_target)
#Finally, we are telling the "manipulator" group we created previously, to calculate the plan.
plan1 = group.plan()
#By executing this line of code, you will be telling your robot to execute the last trajectory that has been set for the Planning Group
#group.go(wait=True) #Here we are sending the commands to the real robot (in this case gazebo)
rospy.sleep(5)
#At the end, we just shut down the moveit_commander module.
moveit_commander.roscpp_shutdown()
