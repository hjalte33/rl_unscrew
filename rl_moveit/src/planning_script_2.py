#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander #allow us to communicate with the MoveIt Rviz interface
import moveit_msgs.msg
import geometry_msgs.msg

def callback(data):

    #pose_target = geometry_msgs.msg.Pose()
    pose_target = data
    group.set_pose_target(pose_target)

    #group.set_path_constraints()

    #Finally, we are telling the "manipulator" group we created previously, to calculate the plan.
    plan1 = group.plan()

    #By executing this line of code, you will be telling your robot to execute the last trajectory that has been set for the Planning Group
    group.go(wait=True)

    print group.get_current_pose()
    rospy.sleep(5)

#Here, we are just initializing the moveit_commander module.
moveit_commander.roscpp_initialize(sys.argv)

#Initializing a ROS node.
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

#Here, we are creating a RobotCommander object, which basically is an interface to our robot.
robot = moveit_commander.RobotCommander()

#Here, we creating a PlanningSceneInterface object, which basically is an interface to the world that surrounds the robot.
scene = moveit_commander.PlanningSceneInterface() 

#Here, we create a MoveGroupCommander object, which is an interface to the manipulator group of joints 
group = moveit_commander.MoveGroupCommander("arm")

#Here we are defining a Topic Publisher, which will publish into the /move_group/display_planned_path topic. By publishing into this topic, we will be able to visualize the planned motion through the MoveIt Rviz interface.
#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)


while not rospy.is_shutdown():
    rospy.Subscriber("pose", geometry_msgs.msg.Pose, callback)
    rospy.spin()
