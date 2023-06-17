#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
from std_srvs.srv import Empty
import tf

def lectureOdom(odom_msg):
    print("-----------------------------------------")
    print("-----------------------------------------")
    print("Leyendo informaciòn de /odom ...")
    # posicion en x y y del turtlebot
    print('Posicion en X = ',odom_msg.pose.pose.position.x)
    print('Posicion en Y = ', odom_msg.pose.pose.position.y )
    # velocidad lineal en x y velocidad angular en z
    print('Velocidad lineal X = ',odom_msg.twist.twist.linear.x)
    print('Velocidad angular Z = ',odom_msg.twist.twist.angular.z)

    # valores de los cuaterniones
    print('Qx = ',odom_msg.pose.pose.orientation.x)
    print('Qy = ',odom_msg.pose.pose.orientation.y)
    print('Qz = ',odom_msg.pose.pose.orientation.z)
    print('Qw = ',odom_msg.pose.pose.orientation.w)
    
    # lista de cuaterniones que luego seran convertidos a grados
    quaternion = (
    odom_msg.pose.pose.orientation.x,
    odom_msg.pose.pose.orientation.y,
    odom_msg.pose.pose.orientation.z,
    odom_msg.pose.pose.orientation.w)
    
    # conversion de cuaterniones a grados
    rpy = tf.transformations.euler_from_quaternion(quaternion)
    # extraccion de los respectivos valores de los angulos en grados
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]

    # mostrar los angulos convertidos
    # print(math.degrees(roll), ' ', math.degrees(pitch), ' ', math.degrees(yaw))
    print('La orientaciòn del robot es: ',math.degrees(yaw))
    print("-----------------------------------------")
    print("-----------------------------------------")

if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot_sub_odom', anonymous=True)
       
        position_topic = "/odom"
        pose_subscriber = rospy.Subscriber(position_topic, Odometry, lectureOdom) 
        rospy.spin()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")