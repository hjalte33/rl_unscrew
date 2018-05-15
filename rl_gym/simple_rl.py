#! /usr/bin/env python
import sys
import numpy
import rospy
import std_msgs
import geometry_msgs.msg
import random

import qlearn


def main_loop(current_pose):

    state = get_state(current_pose)

    action = qlearn.chooseAction(state)

    # Execute the action and get feedback

    action_pub.publish(action)
    rospy.sleep(0.5)


    if not(is_done()):
        state = nextState
    else:
        last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
        break

    print action
    
    
def is_done(state):

    return 0




#ROS node initialise
rospy.init_node('simple_rl_node', anonymous=True)

#Setup publisher to publish actions
action_pub = rospy.Publisher('/action_move', std_msgs.msg.Int8, queue_size=5)

#Setup RL/qlearn parameters:
last_time_steps = numpy.ndarray(0)
qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                alpha=0.2, gamma=0.8, epsilon=0.9)

initial_epsilon = qlearn.epsilon
epsilon_discount = 0.9986

start_time = time.time()
episode = 0
total_episodes = 10
highest_reward = 0
reward = 0


while not rospy.is_shutdown() and episode <= total_episodes:
    # #Calls move function, which sends the wanted movement to the robot
    rospy.Subscriber("/curr_pose", geometry_msgs.msg.PoseStamped, main_loop)
    rospy.spin()