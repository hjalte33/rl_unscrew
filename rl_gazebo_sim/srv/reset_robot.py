#! /usr/bin/env python
import sys
import rospy
import time
import rospkg
from std_srvs.srv import Trigger,Empty
from std_msgs.msg import Float64
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
        #self.robot = moveit_commander.RobotCommander()
        
        # Creating a MoveGroupCommander object, which is an interface to the manipulator group of joints 
        #self.group = moveit_commander.MoveGroupCommander("arm")

        self.pauser_srv = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpauser_srv = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        
        self.reset_world_srv = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        
        self.reset_simulation_srv = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        
               
        # Create the pose for the robot spawn point.
        self.robot_pose = Pose(Point(x=0,y=0,z=1.005), Quaternion(x=0,y=0,z=0,w=1))

        # Create a joint setter using gazebo service.
        self.set_joints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        self.set_gravity = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)



        # Make this class a service with the self.reset_func as the call function. 
        s = rospy.Service('reset_robot', Trigger, self.reset_func_new)
        print "Ready to reset robot."


    def reset_func_mover(self, req):
        """Callback resetting function that resets the robot position by moving 
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

    def reset_func_new(self,req):
        
        print('Pausing')
        self.pauser_srv.call()

        #set up the physics so we can change the gravity to 0
        time_step = Float64(0.001)
        max_update_rate = Float64(1000.0)
        
        gravity = geometry_msgs.msg.Vector3()
        gravity.x = 0.0
        gravity.y = 0.0
        gravity.z = 0.0
        
        ode_config = gazebo_msgs.msg.ODEPhysics()
        ode_config.auto_disable_bodies = False
        ode_config.sor_pgs_precon_iters = 0
        ode_config.sor_pgs_iters = 50
        ode_config.sor_pgs_w = 1.3
        ode_config.sor_pgs_rms_error_tol = 0.0
        ode_config.contact_surface_layer = 0.001
        ode_config.contact_max_correcting_vel = 0.0
        ode_config.cfm = 0.0
        ode_config.erp = 0.2
        ode_config.max_contacts = 20
        
        self.set_gravity(time_step.data, max_update_rate.data, gravity, ode_config)

        print('Reset simulaiton')
        self.reset_simulation_srv.call()

        print('Setting joint values to home position')
        self.set_joints.call(self.robot_name, "robot_description", ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"], [0, -1.5707, 1.5707, -1.5707, -1.5707, 0])
        
        time.sleep(2)

        gravity.z = -9.82
        self.set_gravity(time_step.data, max_update_rate.data, gravity, ode_config)
                
        print('Unpause')
        self.unpauser_srv.call()   


if __name__ == "__main__":
    #print('Waiting for the controller to start')
    #rospy.wait_for_service('/move_group/get_loggers')
    #print ('The controller seems to be starded so I proceed to launch the reset service')
    resetter = RobotReset()
    rospy.spin()