#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32 

def callback(msg):
    print(msg.data)

rospy.init_node('nodo_subscriber')
rospy.loginfo("Nodo suscriptor inicializado")

sub = rospy.Subscriber('contador', Int32, callback)
rospy.spin() 