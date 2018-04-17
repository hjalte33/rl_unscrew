#!/usr/bin/env python

#cubic_interpolation
#Nick Pestell 2016

from math import pi
import rospy
import moveit_commander
import actionlib
from tf import transformations
from control_msgs.msg import *
from trajectory_msgs.msg import * 
from sensor_msgs.msg import *
import geometry_msgs
import copy 
import sys
from scipy.interpolate import PchipInterpolator
import numpy as np
import matplotlib.pyplot as plt

class waypoints:
	def __init__(self):
		rospy.init_node('speed_test', anonymous=True)
		self.group = moveit_commander.MoveGroupCommander("manipulator")
		self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)

	#TAKES INPUT PARAMETERS AND COMPUTES JOINT STATES FOR THE DESIRED TRAJECTORY
	#WITH EE-POINT SEPERATION AT 0.001m
	def generate_plan(self,points):
		self.group.clear_pose_targets()

		waypoints = []
		waypoints.append(self.group.get_current_pose().pose)

		
		wpose = geometry_msgs.msg.Pose()
		q = transformations.quaternion_from_euler(points[3], points[4], points[5], "sxyz")
		wpose.position.x = points[0]
		wpose.position.y = points[1]
		wpose.position.z = points[2]
		wpose.orientation.x = q[0]
		wpose.orientation.y = q[1]
		wpose.orientation.z = q[2]
		wpose.orientation.w = q[3]
		waypoints.append(copy.deepcopy(wpose))

		(self.plan, fraction) = self.group.compute_cartesian_path(waypoints,0.001,0.0)
		print self.plan
		return self.plan

	#EXECUTES THE PLANNED TRAJECTORY BY PUBLISHING ON 'follow_joint_trajectory/goal'
	#AT A FREQUENCY DEFINED BY THE SPECIFIED VELOCITY.
	def move_robot(self,plan,avgV):
		pose = FollowJointTrajectoryGoal()
		pose.trajectory = JointTrajectory()
		times = self.interpolate(avgV) 

		pose.trajectory.joint_names = plan.joint_trajectory.joint_names
		pose.trajectory.points = plan.joint_trajectory.points
		pose.trajectory.points[0].velocities=[0]*6
		pose.trajectory.points[0].time_from_start = rospy.Duration(0.0)
		for i in range(1,len(pose.trajectory.points)):
			pose.trajectory.points[i].velocities=[0]*6
			pose.trajectory.points[i].time_from_start = rospy.Duration(times(i*0.001))

		self.client.wait_for_server()
		self.client.send_goal(pose)
		self.client.wait_for_result()
		return self.client.get_result()

	#GENERATES TIME PARAMTERIZATION FOR 1D MOTION USING A MONOTONIC CUBIC INTERPOLATOR
	def interpolate(self,avgV):
		dist = len(self.plan.joint_trajectory.points)*0.001
		totT = dist/avgV

		t = np.array(np.linspace(0,totT, num=2, endpoint=True))
		x = np.array(np.linspace(0,dist , num=2, endpoint=True))
		f1 = PchipInterpolator(t,x)

		t1 = np.array(np.linspace(0,totT, num = 30, endpoint=True))
		
		f2 = PchipInterpolator(f1(t1),t1)
		
		#x1 = np.array(np.linspace(0,dist, num=30, endpoint=True))
		#plt.plot(t1, f1(t1), 'x', f2(x1),x1 ,'o')
		#plt.show()

		return f2

if __name__ == "__main__":
		coordinates = [float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]),float(sys.argv[4]),float(sys.argv[5]),float(sys.argv[6])]
		exp = waypoints()
		exp.move_robot(exp.generate_plan(coordinates),float(sys.argv[7]))
