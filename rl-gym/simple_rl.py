#! /usr/bin/env python
import sys
import numpy
import rospy
import std_msgs
import time
import geometry_msgs.msg
import random
from std_srvs.srv import Empty

import qlearn


def main_loop(current_pose):

    state = get_state(current_pose, 1)

    action = qlearn.chooseAction(state)

    # Execute the action and get feedback
    action_pub.publish(action)
    rospy.sleep(0.5)
 
    next_state = get_state(current_pose, 0)
    done = is_done(state, current_pose)
    reward = get_reward(done, next_state)

    cumulated_reward += reward

    if highest_reward < cumulated_reward:
        highest_reward = cumulated_reward

    qlearn.learn(state, action, reward, next_state)

    if not(done):
        state = next_state
    else:
        last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
        episode = total_episodes
        done = True

    if action_nr >= total_action_ep:
        episode += 1
        action_nr = 0
        print('Reset simulaiton')
        rospy.ServiceProxy('/reset_robot', Empty)


    print "EP: ", episode, "     ", "action: ", action, "     ", "reward: ", reward
    


def get_state(current_pose, pos_state):
    if pos_state == 1:
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y
        z = current_pose.pose.position.z
        force_z = None
        while force_z == None:
            try:
                force_z = rospy.wait_for_message('/FTsensor_topic', geometry_msgs.msg.WrenchStamped, timeout=1)
            except:
                print "ERROR: Can't read /FTsensor_topic --> trying again.."

    elif pos_state == 0:
        curr_pos = None
        while curr_pos == None:
            try:
                curr_pos = rospy.wait_for_message('/curr_pose', geometry_msgs.msg.PoseStamped, timeout=1)
            except:
                print "ERROR: Can't read /curr_pose --> trying again.."

        x = curr_pos.pose.position.x
        y = curr_pos.pose.position.y
        z = curr_pos.pose.position.z

        force_z = None
        while force_z == None:
            try:
                force_z = rospy.wait_for_message('/FTsensor_topic', geometry_msgs.msg.WrenchStamped, timeout=1)
            except:
                print "ERROR: Can't read /FTsensor_topic --> trying again.."
                
    state = {}
    state['x'] = x
    state['y'] = y
    state['z'] = z
    state['force_z'] = force_z
    return state

def is_done(data, current_pose):
    #print "im in get_state"
    max_ft = -20
    done = False
    if (max_ft > data.wrench.force.z or out_of_bounds(current_pose) == True):
        done = True
    return done

def out_of_bounds(current_pose):
    boundaries = {'x_low': 0.300, 'x_high': 0.600, 'y_low': 0.003, 'y_high': 0.400, 'z_low': 0.005, 'z_high': 0.300}
    if (current_pose.pose.position.x > (boundaries['x_high'] - 0.0001)):
        return True

    elif (current_pose.pose.position.x < (boundaries['x_low'] + 0.0001)):
        return True

    elif (current_pose.pose.position.y > (boundaries['y_high'] - 0.0001)):
        return True

    elif (current_pose.pose.position.y < (boundaries['y_low'] + 0.0001)):
        return True

    elif (current_pose.pose.position.z > (boundaries['z_high'] - 0.0001)):
        return True

    elif (current_pose.pose.position.z < (boundaries['z_low'] + 0.0001)):
        return True

    else:
        return False   

def get_reward(done, next_state):
    reward = 0
    if not done:
        if action == 5:
            reward = 1
        else:
            reward = 0

    else:
        if (out_of_bounds(next_state) == True):
            reward = -200
        else:
            reward = 100
            reward += 2.5/(next_state[2])

    return reward


#ROS node initialise
rospy.init_node('simple_rl_node', anonymous=True)

#Setup publisher to publish actions
action_pub = rospy.Publisher('/action_move', std_msgs.msg.Int8, queue_size=5)

#Setup RL/qlearn parameters:
last_time_steps = numpy.ndarray(0)
qlearn = qlearn.QLearn(actions=range(6),
                alpha=0.2, gamma=0.8, epsilon=0.9)

initial_epsilon = qlearn.epsilon
epsilon_discount = 0.9986

start_time = time.time()
episode = 0
total_episodes = 10
highest_reward = 0
action_nr = 0
total_action_ep = 500



while not rospy.is_shutdown() and episode <= total_episodes:
    # #Calls move function, which sends the wanted movement to the robot
    rospy.Subscriber("/curr_pose", geometry_msgs.msg.PoseStamped, main_loop)
    rospy.spin()