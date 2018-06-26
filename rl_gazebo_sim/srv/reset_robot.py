#! /usr/bin/env python
import sys
import rospy
import time
import rospkg
from std_srvs.srv import Trigger,Empty
import moveit_commander #allow us to communicate with the MoveIt Rviz interface
import moveit_msgs.msg
from geometry_msgs.msg import *
from gazebo_msgs.srv import *
from controller_manager.controller_manager_interface import SwitchController

class RobotReset ():
    def __init__(self):
        rospy.init_node('reset_server')    

        # Robot name used upon destroying and spawning a robot.
        self.robot_name = 'robot' 

        # creating a RobotCommander object, which acts as an interface to our robot.
        self.robot = moveit_commander.RobotCommander()
        
        # Creating a MoveGroupCommander object, which is an interface to the manipulator group of joints 
        try:
            self.group = moveit_commander.MoveGroupCommander("arm")
        except Exception as e:
            # If the movegroup is not ready yet wait 1 sec and try again. 
            print(e)
            print('trying again in 1 sec')
            rospy.sleep(1)

        self.pauser_srv = rospy.ServiceProxy('/gazedo/pause_physics', Empty)

        self.reset_world_srv = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        
        self.reset_simulation_srv = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        
        self.unpauser_srv = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        # Fetch the path to the robot urdf file
        ros_pack = rospkg.RosPack()
        robot_path = ros_pack.get_path('rl_gazebo_sim')
        robot_path += '/ur5_robot_w_tool/ur5_robot_w_tool.urdf'

        # Read the urdf dile.
        with open(robot_path, "r") as f:
            self.robot_urdf = f.read()
        
        # Create the pose for the robot spawn point.
        self.robot_pose = Pose(Point(x=0,y=0,z=1.005), Quaternion(x=0,y=0,z=0,w=1))

        # Create a spawner and a destreyer for the robot using gazebo service.
        self.spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        self.destroyer = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        # Create a joint setter using gazebo service.
        self.set_joints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        # # Uncomment this spawner call if you want to spawn the robot from this reset file.
        # # So far my experience is that destroying and respawning breaks the controller. 
        # # call inputs are: model_name model_xml robot_namespace initial_pose reference_frame
        # self.spawner.call(self.robot_name, self.robot_urdf, "", self.robot_pose, "world")

        # Make this class a service with the self.reset_func as the call function. 
        s = rospy.Service('reset_robot', Trigger, self.reset_func_mover)
        print "Ready to reset robot."

    # Callback function for the subscriber. This is used when using the node as
    # a subscriber. The function makes the robot execute the received position 
    # from the topic.
    def reset_func_mover(self, req):
        """Resetting function that resets the robot position by moving 
           the robot with the controller. This seems to work, but not 
           garanteed to work since the state of the controller and robot
           is unknown.
        
        Arguments:
            req {req} -- Input from service caller. Not used.
        """
        
        # Dict describing the joint states for the robot in home position. 
        # Joint target to reset to just above screw
        
        joint_target = {'shoulder_pan_joint' : -0.219421051071226,
                        'shoulder_lift_joint': -1.4877301174597237,
                        'elbow_joint'       : 1.9075886985532051,
                        'wrist_1_joint'     : -1.9902674414916985,
                        'wrist_2_joint'     : -1.5717293284405551,
                        'wrist_3_joint'     : -0.10938365036715503 }
        '''
        #Joint target to reset to corner of box:
        joint_target = {'shoulder_pan_joint' : -0.19561383140555133,
                        'shoulder_lift_joint': -1.5215440713145796,
                        'elbow_joint'       : 1.9440776426924025,
                        'wrist_1_joint'     : -1.9919776630124097,
                        'wrist_2_joint'     : -1.5712127876636952,
                        'wrist_3_joint'     : -0.09724737704377517 }
        '''
        print('The joint target is: %s' % joint_target)
        
        #send the position to the group object
        self.group.set_joint_value_target(joint_target)

        #Calculate the plan to the position
        plan1 = self.group.plan()

        # Executes the trajectory that has been set for the Planning Group
        # Wait untill move instruction is done
        self.group.go(wait=True)
        rospy.sleep(0.1)

        # Reset the screw and the world.
        self.reset_world_srv.call()

        #return true to indicate success
        return [True,'succeded resetting the robot.']




if __name__ == "__main__":
    print('Waiting for the controller to start')
    rospy.wait_for_service('/move_group/get_loggers')
    print ('The controller seems to be starded so I proceed to launch the reset service')
    resetter = RobotReset()
    rospy.spin()