#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32 

if __name__ == '__main__':
    rospy.init_node('nodo_publicador')
    rospy.loginfo('Nodo inicializado')

    pub = rospy.Publisher('contador', Int32, queue_size=10)

    rate = rospy.Rate(2)
    count = 0

    while not rospy.is_shutdown():
        pub.publish(count)
        rospy.loginfo(count)
        count += 1
        rate.sleep()