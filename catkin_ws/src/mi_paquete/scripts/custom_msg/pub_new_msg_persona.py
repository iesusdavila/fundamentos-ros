#!/usr/bin/env python3

import rospy
from mi_paquete.msg import Persona
from random import random 


if __name__ == '__main__':
    rospy.init_node('nodo_publicador_msg_nuevo_persona')
    rospy.loginfo('Nodo publicador para mensaje nuevo persona inicializado')

    pub = rospy.Publisher('personas', Persona, queue_size=10)

    rate = rospy.Rate(2)
    count = 0

    while not rospy.is_shutdown():
        nueva_persona = Persona()
        nueva_persona.nombre = "Iesus"
        nueva_persona.edad = 23
        nueva_persona.estatura = random()*100
        nueva_persona.peso = random()*100
        rospy.loginfo(nueva_persona)

        pub.publish(nueva_persona)
        rate.sleep()