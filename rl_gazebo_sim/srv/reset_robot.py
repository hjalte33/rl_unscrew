#! /usr/bin/env python
import sys
import rospy
import time
from std_srvs.srv import Trigger,Empty
import moveit_commander #allow us to communicate with the MoveIt Rviz interface
import moveit_msgs.msg

#from gazebo_msgs.srv import *
#from controller_manager.controller_manager_interface import SwitchController

class RobotReset ():
    def __init__(self):
        rospy.init_node('reset_server')    

        self.robot_name = 'robot' 

        #creating a RobotCommander object, which acts as an interface to our robot.
        self.robot = moveit_commander.RobotCommander()
        
        ## creating a PlanningSceneInterface object, which is an interface to the world that surrounds the robot.
        # scene = moveit_commander.PlanningSceneInterface() 

        #Creating a MoveGroupCommander object, which is an interface to the manipulator group of joints 
        self.group = moveit_commander.MoveGroupCommander("arm")

        s = rospy.Service('reset_robot', Empty, self.reset_func)
        print "Ready to reset robot."

    #callback function for the subscriber. This is used when using the node as a subscriber. The function makes the robot execute the received position from the topic.
    def reset_func(self, req):
        # joints = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
        # target = [0, -1.5707, 1.5707, -1.5707, -1.5707, 0]       
        joint_target = {'shoulder_pan_joint' : 0,
                        'shoulder_lift_joint': -1.5707,
                        'elbow_joint'       : 1.5707,
                        'wrist_1_joint'     : -1.5707,
                        'wrist_2_joint'     : -1.5707,
                        'wrist_3_joint'     : 0 }
        print(joint_target)
        
        #send the position to the group object
        self.group.set_joint_value_target(joint_target)

        #Calculate the plan to the position
        plan1 = self.group.plan()

        #executes the trajectory that has been set for the Planning Group
        self.group.go(wait=True)
        
        #when playing around it is nice to print the current position
        print self.group.get_current_pose()
        rospy.sleep(0.1)


        # print('pausing')
        # rospy.ServiceProxy('/gazedo/pause_physics', Empty)
        # time.sleep(1)
        # print('stopping controllers')
        # controller_switcher = rospy.ServiceProxy('controller_manager/switch_controller',SwitchController)
        # controller_switcher.call([], ["arm_controller","joint_state_controller","screw_controller"], 1)
        # time.sleep(1)
        # destroyer = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        # destroyer.call(self.robot_name)
        # spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        
        # #model_name model_xml robot_namespace initial_pose reference_frame
        # spawner.call(self.robot_name, self.robot_urdf, "", self.robot_pose, "world")
        # print('setting joint values')
        # joints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        # joints.call("robot", "robot_description", ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"], [0, -1.5707, 1.5707, -1.5707, -1.5707, 0])
        # time.sleep(1)
        print('reset world')
        rospy.ServiceProxy('/gazebo/reset_world', Empty)
        print('reset simulaiton')
        rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        time.sleep(1)
        # print('unpause')
        # rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # time.sleep(1)
        # print('start controllers')
        # controller_switcher.call(["arm_controller","joint_state_controller","screw_controller"], [], 1)
        


if __name__ == "__main__":
    resetter = RobotReset()
    rospy.spin()