#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

x=0
y=0
z=0
yaw=0

def poseCallback(pose_message):
    global x
    global y, yaw
    x= pose_message.x
    y= pose_message.y
    yaw = pose_message.theta

def move(velocity_publisher,speed, distance, is_forward):
    velocity_message = Twist()
    
    # ubicacion actual
    global x,y
    x0=x
    y0=y

    # movimiento lineal hacia delante o atras
    if (is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10) 

    while True :
        rospy.loginfo("Turtlesim moviendose!")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()

        # calculo de la distanca movida
        distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print(distance_moved)
        # validar si la distancia por mover supera a la distancia establecida
        if  (distance_moved>=distance):
            rospy.loginfo("Distancia alcanzada")
            break
            
    # detener el robot una vez completada la trayectoria
    velocity_message.linear.x =0
    velocity_publisher.publish(velocity_message)
    

if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_movimiento_pos', anonymous=True)

        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)

 
        time.sleep(2)
        print ('Movimiento en lìnea recta: ')
        move (velocity_publisher,1.0, 3.0, is_forward=True)
        #time.sleep(2)
        #print ('Inicia reset: ')
        #rospy.wait_for_service('reset')
        #reset_turtle = rospy.ServiceProxy('reset', Empty)
        #reset_turtle()
        #print ('Termina reset: ')
        #rospy.spin()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")