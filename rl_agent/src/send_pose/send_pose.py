#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_msgs.msg
import moveit_commander #allow us to communicate with the MoveIt Rviz interface
import geometry_msgs.msg

def talker():
    pub = rospy.Publisher('pose', geometry_msgs.msg.Pose)
    rospy.init_node('send_pose', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    #Here, we create a MoveGroupCommander object, which is an interface to the manipulator group of joints 
    group = moveit_commander.MoveGroupCommander("arm")
    #This is how we get the current position of the end effector:
    print "Current Pose:"
    print group.get_current_pose()

    # run the ros node
    while not rospy.is_shutdown():
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.x = 0.0
        pose_target.orientation.y = 0.0
        pose_target.orientation.z = 0.0
        pose_target.orientation.w = 3.0
        pose_target.position.x = 0.5
        pose_target.position.y = 0.5
        pose_target.position.z = 0.3
        #rospy.loginfo(pose_target)
        #pub.publish(pose_target)
        print "Current Pose:"
        print group.get_current_pose()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass