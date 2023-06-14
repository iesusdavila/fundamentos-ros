#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

vels = Twist()

def scan_callback(a):
    move(a)

def move(s):
    loop_rate = rospy.Rate(10) # 10 mensajes por segundo

    # fijar velocidad lineal en x en 0.4
    vels.linear.x = 0.8
    # fijar velocidad angular alrededor de z en 0
    vels.angular.z =0
    
    obst_frente = s.ranges[0]
    obst_izquierda = s.ranges[90]
    obst_derecha = s.ranges[270]
    obst_atras = s.ranges[180]

    print(f'Poseen {len(s.ranges)} mediciones en el Laser Scan')
    print(f'Distancia al frente: {obst_frente}')
    print(f'Distancia a la izquierda: {obst_izquierda}')
    print(f'Distancia a la derecha: {obst_derecha}')
    print(f'Distancia a atras: {obst_atras}')

    if(obst_frente < 0.5 or obst_atras < 0.5 or obst_derecha < 0.5 or obst_izquierda < 0.5):
        vels.linear.x =0
        pub.publish(vels)
    if(vels.linear.x ==0):
        print("TurtleBot girando... ")
        print("Definiendo sentido de giro...")
        rotate(s)
        pub.publish(vels)

    loop_rate.sleep()
    pub.publish(vels)

def rotate(b):
    loop_rate = rospy.Rate(10)

    obst_frente = b.ranges[0]
    obst_izquierda = b.ranges[90]
    obst_derecha = b.ranges[270]
    obst_atras = b.ranges[180]

    if (obst_frente < 3 or obst_izquierda < 3):
        vels.angular.z=1
        print("TurtleBot girando en sentido horario")
        pub.publish(vels)
    elif (obst_atras < 3 or obst_derecha < 3):
        print("TurtleBot girando en sentido anti-horario")
        vels.angular.z=-1
        pub.publish(vels)
    else:
        pub.publish(vels)

    pub.publish(vels)
    loop_rate.sleep()
    

if __name__ == "__main__":
    rospy.init_node('scan_node12', anonymous=True)
    pub= rospy.Publisher('/cmd_vel' , Twist , queue_size=100)
    rospy.Subscriber("scan", LaserScan, scan_callback)
    time.sleep(2)
    rospy.spin()