#!/usr/bin/env python3

import rospy
from mi_paquete.srv import Mi_servicio, Mi_servicioResponse

def contar_palabras(request):
    return Mi_servicioResponse(len(request.palabra.split()))

def mi_servicio_server():
    rospy.init_node('nodo_servicio_servidor')
    rospy.loginfo("Nodo del servidor de servicio inicializado")
    rospy.loginfo(contar_palabras)

    service = rospy.Service('contador_palabras', Mi_servicio, contar_palabras)

    rospy.spin()

if __name__ == '__main__':
    mi_servicio_server()

