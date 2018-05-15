#! /usr/bin/env python
import sys
import rospy
import std_msgs
import geometry_msgs.msg
import random

def action_execution(current_pose):
    action = random.randint(0,5)
    print action
    action_pub.publish(action)
    rospy.sleep(0.5)
    


rospy.init_node('simple_rl_node', anonymous=True)

action_pub = rospy.Publisher('/action_move', std_msgs.msg.Int8, queue_size=5)

while not rospy.is_shutdown():
    # #Calls move function, which sends the wanted movement to the robot
    rospy.Subscriber("/curr_pose", geometry_msgs.msg.PoseStamped, action_execution)
    rospy.spin()