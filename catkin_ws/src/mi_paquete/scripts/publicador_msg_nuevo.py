#!/usr/bin/env python3

import rospy
from mi_paquete.msg import Complex
from random import random 


if __name__ == '__main__':
    rospy.init_node('nodo_publicador_msg_nuevo')
    rospy.loginfo('Nodo publicador para mensaje nuevo inicializado')

    pub = rospy.Publisher('complejos', Complex, queue_size=10)

    rate = rospy.Rate(2)
    count = 0

    while not rospy.is_shutdown():
        mensaje = Complex()
        mensaje.real = random()
        mensaje.imaginary = random()
        rospy.loginfo(mensaje)

        pub.publish(mensaje)
        rate.sleep()