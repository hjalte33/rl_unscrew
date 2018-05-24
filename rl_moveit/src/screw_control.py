#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander #allow us to communicate with the MoveIt Rviz interface
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs
import random

#####################################################
#This node either functions as a subscriber to receive pose throug a topic and then execute it, so the robot in gaze will move
#Or keeps randomly picks between the 6 actions and executes them until stopped.
#####################################################

#callback function for the subscriber. This is used when using the node as a subscriber. The function makes the robot execute the received position from the topic.
def callback(pose_target):


    group.set_pose_target(pose_target)

    ##Calculate the plan to the position
    plan1 = group.plan()

    #executes the trajectory that has been set for the Planning Group
    group.go(wait=True)

    #prints the current position
    print group.get_current_pose(end_effector_link="ee_tip")

    #Sleeps in seconds
    rospy.sleep(5)

def move_action(action):
    
    #how much the robot should move for each step
    step_size = -0.25

    #The search space needs to correspend to what's in the boundaries dict
    #boundaries = {'x_low': 0.496, 'x_high': 0.498, 'y_low': 0.003, 'y_high': 0.005, 'z_low': 0.0055, 'z_high': 0.0075}
    #boundaries = {'x_low': 0.300, 'x_high': 0.600, 'y_low': 0.003, 'y_high': 0.400, 'z_low': 0.005, 'z_high': 0.300}

    #Get the current posistion
    current_joint_value = group.get_current_joint_values()

    print current_joint_value
    #6 different move actions. if statements to check which one is wanted.
    #Action 0: move step_size in x+ direction
    #Action 1: move step_size in x- direction
    #Action 2: move step_size in y+ direction
    #Action 3: move step_size in y- direction
    #Action 4: move step_size in z+ direction
    #Action 5: move step_size in z- direction

    if action.data == 6:
        current_joint_value[0] = current_joint_value[0] + step_size
        move(current_joint_value)
    #Returns the pose after the action has been accounted for
    #return current_pose

def move(joint_value_target):
    #if move_action returned false, the robot should not move and this function then returns False
    #if pose_target == False:
    #    print "out of boundary"
    #    return False

    
    #send the position to the group object
    group.set_joint_value_target(joint_value_target)

    #Calculate the plan to the position
    plan1 = group.plan()

    #executes the trajectory that has been set for the Planning Group
    group.go(wait=True)
    
    #when playing around it is nice to print the current position
    #print group.get_current_pose()
    print "Pose target:"
    print joint_value_target
    #rospy.sleep(0.5)    


#initializing the moveit_commander module.
moveit_commander.roscpp_initialize(sys.argv)

#initializing this as a ROS node.
rospy.init_node('Screw_control_node', anonymous=True)

#creating a RobotCommander object, which acts as an interface to our robot.
robot = moveit_commander.RobotCommander()

#creating a PlanningSceneInterface object, which is an interface to the world that surrounds the robot.
scene = moveit_commander.PlanningSceneInterface() 

#Creating a MoveGroupCommander object, which is an interface to the manipulator group of joints 
group = moveit_commander.MoveGroupCommander("tool")

# print the end effector link so we know what frame is used (should be ee_tip)
print group.get_end_effector_link()

while not rospy.is_shutdown():
    # #*************** Uncomment this block if the node should send random points to the action function
    # #Gives a random number between 1 and 6, to move the robot randomly with the 6 actions.
    # action = random.randint(1,6)
    
    # #Calls move function, which sends the wanted movement to the robot
    rospy.Subscriber("action_move", std_msgs.msg.Int8, move_action)
    rospy.spin()
    



    #**************** Uncomment this block if the node should subscribe to a topic for receiving points.
    #rospy.Subscriber("pose", geometry_msgs.msg.Pose, callback)
    #rospy.spin()

