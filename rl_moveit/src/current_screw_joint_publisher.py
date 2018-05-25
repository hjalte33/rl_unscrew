#! /usr/bin/env python
import sys
import rospy
import moveit_commander #allow us to communicate with the MoveIt Rviz interface
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs

#initializing the moveit_commander module.
moveit_commander.roscpp_initialize(sys.argv)

#initializing this as a ROS node.
rospy.init_node('get_current_screw_joint_publisher', anonymous=True)

#creating a RobotCommander object, which acts as an interface to our robot.
robot = moveit_commander.RobotCommander()

#creating a PlanningSceneInterface object, which is an interface to the world that surrounds the robot.
scene = moveit_commander.PlanningSceneInterface() 

#Creating a MoveGroupCommander object, which is an interface to the manipulator group of joints 
group = moveit_commander.MoveGroupCommander("tool")

# print the end effector link so we know what frame is used (should be ee_tip)
print group.get_end_effector_link()

#Setup /screw_joint_value publisher
screw_pub = rospy.Publisher("screw_joint_value", std_msgs.msg.Float32, queue_size=1)

while not rospy.is_shutdown():
    # #*************** Uncomment this block if the node should send random points to the action function
    # #Gives a random number between 1 and 6, to move the robot randomly with the 6 actions.
    # action = random.randint(1,6)
    
    # #Calls move function, which sends the wanted movement to the robot
    #rospy.Subscriber("action_move", std_msgs.msg.Int8, move_action)
    screw_joint_value = group.get_current_joint_values()
    print screw_joint_value
    screw_pub.publish(screw_joint_value[0])
    #rospy.spin()
    



    #**************** Uncomment this block if the node should subscribe to a topic for receiving points.
    #rospy.Subscriber("pose", geometry_msgs.msg.Pose, callback)
    #rospy.spin()

