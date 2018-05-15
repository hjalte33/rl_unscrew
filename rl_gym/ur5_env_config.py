#!/usr/bin/env python
import sys
sys.path.append('../')
import gym
import rospy
import copy
import roslaunch
import time
import numpy as np
import geometry_msgs
import std_msgs

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from std_srvs.srv import Empty

from gym.utils import seeding

#current_pose = 0

class UR5env(gazebo_env.GazeboEnv):
    
    def __init__(self):
        # Launch the simulation with the given launchfile name
        #gazebo_env.GazeboEnv.__init__(self, "/home/studerende/18gr660/catkin_ws/src/rl_project/rl_gazebo_sim/launch/rl_world.launch") #FULL PATH PLEASE
        #controller = open_launch_file()

        rospy.init_node('Env_setup_node', anonymous=True)
        print "Im in init"
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.action_pub = rospy.Publisher('/action_move', std_msgs.msg.Int8, queue_size=5)


        #rospy.Subscriber("/FTsensor_topic", WrenchStamped, self.ftsensor_callback)

        #Robot initializing boundaries and step size
        self.step_size = 0.01
        self.boundaries = {'x_low': 0.300, 'x_high': 0.600, 'y_low': 0.003, 'y_high': 0.400, 'z_low': 0.005, 'z_high': 0.300}

        #Define actions and reward range
        self.action_space = spaces.Discrete(6) #x+, x-, y+, y-, z+, z-, (add when unscrewing is an action too)
        self.reward_range = (-np.inf, np.inf)

        self._seed()        

    #def get_curr_pos(self, current_pos):
    #    current_pose = current_pos

    def out_of_bounds(self):
        current_pose = None
        while current_pose == None:
            try:
                current_pose = rospy.wait_for_message('/curr_pose', geometry_msgs.msg.PoseStamped, 1)
            except:
                print "Couldn't read current position"

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

    def get_state(self,data):
        current_pose = None
        while current_pose == None:
            try:
                current_pose = rospy.wait_for_message('/curr_pose', geometry_msgs.msg.PoseStamped, 1)
            except:
                print "Couldn't read current position"

        #print "im in get_state"
        max_ft = -50
        done = False
        if (max_ft > data.wrench.force.z or self.out_of_bounds(current_pose) == True):
            done = True
        return done

    def _seed(self, seed=None):
        #print "im in _seed"
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # def move(self, pose_target):
    #     print "im in move"
    #     #send the position to the group object
    #     group.set_pose_target(pose_target)

    #     #Calculate the plan to the position
    #     group.plan()

    #     #executes the trajectory that has been set for the Planning Group
    #     group.go(wait=True)
        
    #     #rospy.sleep(0.5) #Check this delay

    def _step(self, action):
        #print "I'm in _step"

        current_pose = None
        while current_pose == None:
            try:
                current_pose = rospy.wait_for_message('/curr_pose', geometry_msgs.msg.PoseStamped, 1)
            except:
                print "Couldn't read current position"

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print "/gazebo/unpause_physics service call failed"

        target_pose = current_pose

        #6 different move actions. if statements to check which one is wanted.
        #Action 1: move step_size in x+ direction
        #Action 2: move step_size in x- direction
        #Action 3: move step_size in y+ direction
        #Action 4: move step_size in y- direction
        #Action 5: move step_size in z+ direction
        #Action 6: move step_size in z- direction
        print "ACTION IS: ", action
        if action == 0:
            if (current_pose.pose.position.x > (self.boundaries['x_high'] - 0.0001)):
                print("Out of bounds!!")
                #break
            else:
                target_pose.pose.position.x = current_pose.pose.position.x + self.step_size
                #self.move(target_pose)
                self.action_pub.publish(action)

        elif action == 1:
            if (current_pose.pose.position.x < (self.boundaries['x_low'] + 0.0001)):
                print("Out of bounds!!")
                #break
            else: 
                target_pose.pose.position.x = current_pose.pose.position.x - self.step_size
                #self.move(target_pose)
                self.action_pub.publish(action)

        elif action == 2:
            if (current_pose.pose.position.y > (self.boundaries['y_high'] - 0.0001)):
                print("Out of bounds!!")
                #break
            else: 
                target_pose.pose.position.y = current_pose.pose.position.y + self.step_size
                #self.move(target_pose)
                self.action_pub.publish(action)

        elif action == 3:
            if (current_pose.pose.position.y < (self.boundaries['y_low'] + 0.0001)):
                print("Out of bounds!!")
                #break
            else: 
                target_pose.pose.position.y = current_pose.pose.position.y - self.step_size
                #self.move(target_pose)
                self.action_pub.publish(action)

        elif action == 4:
            if (current_pose.pose.position.z > (self.boundaries['z_high'] - 0.0001)):
                print("Out of bounds!!")
                #break
            else: 
                target_pose.pose.position.z = current_pose.pose.position.z + self.step_size
                #self.move(target_pose)
                self.action_pub.publish(action)

        elif action == 5:
            if (current_pose.pose.position.z < (self.boundaries['z_low'] + 0.0001)):
                print("Out of bounds!!")
                #break
            else: 
                target_pose.pose.position.z = current_pose.pose.position.z - self.step_size
                #self.move(target_pose)
                self.action_pub.publish(action)

        #If garbage is sent to the function return "Invalid aciton"
        else: 
            print "ACTION IS: ", action
            print "Invalid action"
            

        
        data = None
        while data is None:
            data = rospy.wait_for_message('/FTsensor_topic', geometry_msgs.msg.WrenchStamped, timeout=5)

        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        done = self.get_state(data)

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

        state = data.wrench.force.z

        return state, reward, done, {}
    

    def _reset(self):
        #print "im in _reset"
        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        data = None
        #read laser data  
        while data is None:
            
            try:
                #print "something"
                data = rospy.wait_for_message('/FTsensor_topic', geometry_msgs.msg.WrenchStamped, timeout=5)
                #print data
            except:
                print "In except case - doesn't get any data"
                pass

        # rospy.wait_for_service('/gazebo/pause_physics')
        # try:
        #     #resp_pause = pause.call()
        #     self.pause()
        # except (rospy.ServiceException) as e:
        #     print ("/gazebo/pause_physics service call failed")

        #state = self.get_state(data)
        state = data.wrench.force.z

        return state
