import gym
from gym import error, spaces, utils
from gym.utils import seeding

import numpy as np

import rospy
import std_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Trigger

class UnscrewingEnv(gym.Env):

    #metadata = {'render.modes': ['human']}

    def __init__(self):
        #Init ros node
        rospy.init_node('Env_setup_node', anonymous=True)

        #Setup publisher to publish actions
        self.action_pub = rospy.Publisher('/action_move', std_msgs.msg.Int8, queue_size=1)

        self.reset_robot = rospy.ServiceProxy('/reset_robot', Trigger)

        #Robot initializing boundaries and step size
        self.step_size = 0.01
        self.boundaries = {'x_low': 0.45, 'x_high': 0.55, 'y_low': -0.05, 'y_high': 0.05, 'z_low': 0.022, 'z_high': 0.04}

        #Define actions and reward range
        self.action_space = spaces.Discrete(6) #x+, x-, y+, y-, z+, z-, (add when unscrewing is an action too)
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def _seed(self, seed=None):
        #print "im in _seed"
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        current_pose = None
        while current_pose == None:
            try:
                current_pose = rospy.wait_for_message('/curr_pose', geometry_msgs.msg.PoseStamped, 1)
            except:
                print "Couldn't read current position"

        #Execute action by publishing action to the topic "/action_move"
        self.action_pub.publish(action)

        rospy.sleep(0.5)

        state = self.get_state(current_pose, 0)

        done = self.is_done(state, current_pose)

        if not done:
            if action == 5:
                reward = 1
            else:
                reward = 0

        else:
            if (self.out_of_bounds(current_pose) == True):
                reward = -200
            else:
                reward = 100
                reward += 2.5/(current_pose.pose.position.z)

        return state, reward, done, {}


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


    def get_state(self, current_pose, pos_state):
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
        state['force_z'] = force_z.wrench.force.z
        return state


    def is_done(self, data, current_pose):
        #print "im in get_state"
        #print data
        max_ft = -20
        done = False
        if (max_ft > data['force_z'] or self.out_of_bounds(current_pose) == True):
            done = True
        return done

    def reset(self):
        print("Reset Simulation")
        reset_succes = self.reset_robot.call()
        rospy.sleep(1)
        #print reset_succes, type(reset_succes)

        current_pose = None
        while current_pose == None:
            try:
                current_pose = rospy.wait_for_message('/curr_pose', geometry_msgs.msg.PoseStamped, 1)
            except:
                print "Couldn't read current position"


        return self.get_state(current_pose, 1) 
