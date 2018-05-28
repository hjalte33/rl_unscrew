import gym
from gym import error, spaces, utils
from gym.utils import seeding

import numpy as np

import rospy
import std_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Trigger

class UnscrewingEnv(gym.Env):

    #Init function to initialise class and setup topics and services needed
    def __init__(self):
        #Init ros node
        rospy.init_node('Env_setup_node', anonymous=True)

        #Setup publisher to publish actions
        self.action_pub = rospy.Publisher('/action_move', std_msgs.msg.Int8, queue_size=1)

        #Setup rosservice to reset robot
        self.reset_robot = rospy.ServiceProxy('/reset_robot', Trigger)

        #Robot initializing boundaries and step size
        #self.step_size = 0.01
        self.boundaries = {'x_low': 0.478915, 'x_high': 0.521085, 'y_low': -0.021225, 'y_high': 0.021225, 'z_low': 0.02, 'z_high': 0.04}

        #Define actions and reward range
        self.action_space = spaces.Discrete(7) #x+, x-, y+, y-, z+, z-, (add when unscrewing is an action too)
        self.reward_range = (-np.inf, np.inf)

        self.screw_touch_state = False

        self._seed()

    #Seed function for updating random generator correctly
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    #Step function which publishes actions and evaluates state/reward
    def step(self, action):
        #Get the current_pose
        current_pose = None
        while current_pose == None:
            try:
                current_pose = rospy.wait_for_message('/curr_pose', geometry_msgs.msg.PoseStamped, 1)
            except:
                print "ERROR: Can't read /curr_pose --> trying again.."

        #Execute action by publishing action to the topic "/action_move"
        self.action_pub.publish(action)

        #Move_group can't execute actions quickly enough. Delay of 0.5s is needed.
        rospy.sleep(0.25)

        #Get new state after action, and check if episode is done
        state = self.get_state(current_pose, 0)
        done = self.is_done(state, current_pose)

        
        #Evaluate rewards depending on if episode is done/not done
        if not done:
            if state['z'] < 0.0295:
                self.screw_touch_state = True

            if state['force_z'] < 0 and action == 6:
                reward = 2

            else:
                reward = 0

        else:
            if (self.out_of_bounds(current_pose) == True):
                reward = -10
            else:
                if self.screw_touch_state == True:
                    reward = 30
                    reward += 3000*state['z']

                else:
                    reward = 10
                    reward += 1.5/(current_pose.pose.position.z)
                

        return state, reward, done, {}


    #Function to call, to check if robot is out of bounds (boundaires defined in __init__)
    def out_of_bounds(self, current_pose):
        if (current_pose.pose.position.x > (self.boundaries['x_high'] - 0.0001)):
            return True

        elif (current_pose.pose.position.x < (self.boundaries['x_low'] + 0.0001)):
            return True

        elif (current_pose.pose.position.y > (self.boundaries['y_high'] - 0.0001)):
            return True

        elif (current_pose.pose.position.y < (self.boundaries['y_low'] + 0.0001)):
            return True

        elif (current_pose.pose.position.z > (self.boundaries['z_high'] - 0.0001)):
            return True

        elif (current_pose.pose.position.z < (self.boundaries['z_low'] + 0.0001)):
            return True

        else:
            return False


    def discretise_data(self, state):
        newstate = {}
        newstate['x'] = round(state['x'], 4)
        newstate['y'] = round(state['y'], 4)
        newstate['z'] = round(state['z'], 4)
        newstate['force_z'] = round(state['force_z'], 1)
        newstate['screw_joint_value'] = round(state['screw_joint_value'], 3)

        return newstate


    # Function to call to get current state. two ways to call it. By knowing
    # the current pose, then pos_state = 1, if not the function will automatically
    # get the current_pose, then pos_state = 0.
    def get_state(self, current_pose, pos_state):
        if pos_state == 1:
            x = current_pose.pose.position.x
            y = current_pose.pose.position.y
            z = current_pose.pose.position.z

        elif pos_state == 0:
            # Loop to get current pose:
            curr_pos = None
            while curr_pos == None:
                try:
                    curr_pos = rospy.wait_for_message('/curr_pose', geometry_msgs.msg.PoseStamped, timeout=1)
                except:
                    print "ERROR: Couldn't read /curr_pose --> trying again.."

            x = curr_pos.pose.position.x
            y = curr_pos.pose.position.y
            z = curr_pos.pose.position.z

        #loop to get force in z direction
        force_z = None
        while force_z == None:
            try:
                force_z = rospy.wait_for_message('/FTsensor_topic', geometry_msgs.msg.WrenchStamped, timeout=1)
            except:
                print "ERROR: Couldn't read /FTsensor_topic --> trying again.."

        screw_joint_value = None
        while screw_joint_value == None:
            try:
                screw_joint_value = rospy.wait_for_message('/screw_joint_value', std_msgs.msg.Float32, timeout=1)
            except:
                print "ERROR: Couldn't read /screw_joint_value topic --> trying again.."

        # Collecting the data read above into a state dict, and returning it  
        state = {}
        state['x'] = x
        state['y'] = y
        state['z'] = z
        state['force_z'] = force_z.wrench.force.z
        state['screw_joint_value'] = screw_joint_value.data
        return self.discretise_data(state)

    # Function to call to check if episode is done. It is done if max force in z direction
    # is read to be larger than threshold, or if the manipulator is out of boundaries
    # It returns True/False depending on whether the episode is done.
    def is_done(self, data, current_pose):
        max_ft = -10
        done = False
        if (max_ft > data['force_z'] or self.out_of_bounds(current_pose) == True):
            done = True
        return done


    # Reset function that calls rosservice /reset_robot to reset robot position
    def reset(self):
        print("Reset Simulation")

        self.screw_touch_state = False
        
        #Call rosservice to reset robot:
        reset_succes = self.reset_robot.call()
        rospy.sleep(1)

        # Get new pose state after reset and return it.
        current_pose = None
        while current_pose == None:
            try:
                current_pose = rospy.wait_for_message('/curr_pose', geometry_msgs.msg.PoseStamped, 1)
            except:
                print "ERROR: Can't read /curr_pose --> trying again.."


        return self.get_state(current_pose, 1) 
