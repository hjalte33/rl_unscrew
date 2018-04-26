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
waypoints = []

# start with the current pose
waypoints.append(group.get_current_pose().pose)

# first orient gripper and move forward (+x)
wpose = geometry_msgs.msg.Pose()
wpose.orientation.w = 1.0
wpose.position.x = waypoints[0].position.x + 0.1
wpose.position.y = waypoints[0].position.y
wpose.position.z = waypoints[0].position.z
waypoints.append(copy.deepcopy(wpose))

# second move down
wpose.position.z -= 0.10
waypoints.append(copy.deepcopy(wpose))

# third move to the side
wpose.position.y += 0.05
waypoints.append(copy.deepcopy(wpose))
group.set_pose_target(pose_target)
#Finally, we are telling the "manipulator" group we created previously, to calculate the plan.
(plan3, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold

print "============ Waiting while RVIZ displays plan3..."
rospy.sleep(5)
#By executing this line of code, you will be telling your robot to execute the last trajectory that has been set for the Planning Group
#group.go(wait=True) #Here we are sending the commands to the real robot (in this case gazebo)
rospy.sleep(5)
#At the end, we just shut down the moveit_commander module.
moveit_commander.roscpp_shutdown()
