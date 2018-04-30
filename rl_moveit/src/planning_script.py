#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander #allow us to communicate with the MoveIt Rviz interface
import moveit_msgs.msg
import geometry_msgs.msg
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
    print group.get_current_pose()

    #Sleeps in seconds
    rospy.sleep(5)

def move_action(action):

    #how much the robot should move for each step
    step_size = 0.0005

    #The search space needs to correspend to what's in the boundaries dict
    boundaries = {'x_low': 0.496, 'x_high': 0.498, 'y_low': 0.003, 'y_high': 0.005, 'z_low': 0.0055, 'z_high': 0.0075}
    
    #Get the current posistion
    current_pose = group.get_current_pose()

    #6 different move actions. if statements to check which one is wanted.
    #Action 1: move step_size in x direction
    #Action 2: move step_size in -x direction
    #Action 3: move step_size in y direction
    #Action 4: move step_size in -y direction
    #Action 5: move step_size in z direction
    #Action 6: move step_size in -z direction

    if action == 1:
        
        #makes sure that if executing this action, that the robot does not get out of the workspace. If so, return False.
        if (current_pose.pose.position.x > (boundaries['x_high'] - 0.0001)):
            return False
        
        #If the robot is clear to go, the x position is increased by the step_size.
        else:
            current_pose.pose.position.x = current_pose.pose.position.x + step_size

    elif action == 2:
        if (current_pose.pose.position.x < (boundaries['x_low'] + 0.0001)):
            return False
        else: 
            current_pose.pose.position.x = current_pose.pose.position.x - step_size

    elif action == 3:
        if (current_pose.pose.position.y > (boundaries['y_high'] - 0.0001)):
            return False
        else: 
            current_pose.pose.position.y = current_pose.pose.position.y + step_size

    elif action == 4:
        if (current_pose.pose.position.y < (boundaries['y_low'] + 0.0001)):
            return False
        else: 
            current_pose.pose.position.y = current_pose.pose.position.y - step_size

    elif action == 5:
        if (current_pose.pose.position.z > (boundaries['z_high'] - 0.0001)):
            return False
        else: 
            current_pose.pose.position.z = current_pose.pose.position.z + step_size

    elif action == 6:
        if (current_pose.pose.position.z < (boundaries['z_low'] + 0.0001)):
            return False
        else: 
            current_pose.pose.position.z = current_pose.pose.position.z - step_size

    #If garbage is send to the function return "Invalid aciton"
    else: return "Invalid action"

    #Returns the pose after the action has been accounted for
    return current_pose

def move(pose_target):
    #if move_action returned false, the robot should not move and this function then returns False
    if pose_target == False:
        print "out of boundary"
        return False

    #send the position to the group object
    group.set_pose_target(pose_target)

    #Calculate the plan to the position
    plan1 = group.plan()

    #executes the trajectory that has been set for the Planning Group
    group.go(wait=True)
    
    #when playing around it is nice to print the current position
    print group.get_current_pose()
    rospy.sleep(0.1)


#initializing the moveit_commander module.
moveit_commander.roscpp_initialize(sys.argv)

#initializing this as a ROS node.
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

#creating a RobotCommander object, which acts as an interface to our robot.
robot = moveit_commander.RobotCommander()

#creating a PlanningSceneInterface object, which is an interface to the world that surrounds the robot.
scene = moveit_commander.PlanningSceneInterface() 

#Creating a MoveGroupCommander object, which is an interface to the manipulator group of joints 
group = moveit_commander.MoveGroupCommander("arm")

while not rospy.is_shutdown():
    #*************** Uncomment this block if the node should send random points to the action function
    #Gives a random number between 1 and 6, to move the robot randomly with the 6 actions.
    action = random.randint(1,6)
    
    #Calls move function, which sends the wanted movement to the robot
    move(move_action(action))
    



    # #**************** Uncomment this block if the node should subscribe to a topic for receiving points.
    # rospy.Subscriber("pose", geometry_msgs.msg.Pose, callback)
    # rospy.spin()