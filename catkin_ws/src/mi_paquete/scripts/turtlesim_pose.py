#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
#task 1. import the Pose type from the module turtlesim


def poseCallback(pose_message):
    print(pose_message.theta)
    #task 4. display the x, y, and theta received from the message
    print("pose callback")
    print('x = ',pose_message.x)
    print('y = ',pose_message.y)
    print('yaw = ',pose_message.theta) 

if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)        
        rospy.loginfo("Nodo suscriptor inicializado")
        #task 2. subscribe to the topic of the pose of the Turtlesim
        sub = rospy.Subscriber('/turtle1/pose', Pose, poseCallback)
        #task 3. spin
        rospy.spin() 

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")