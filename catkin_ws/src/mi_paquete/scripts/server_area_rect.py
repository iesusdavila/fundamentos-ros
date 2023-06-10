#!/usr/bin/env python3

import rospy 
from mi_paquete.srv import Area_rectangle, Area_rectangleResponse

def calcular_area_rect(request):
    print(f'Retornando el calculo del area mediante {request.ancho} * {request.alto}')
    return Area_rectangleResponse(request.ancho*request.alto)

def mi_servicio_server():
    rospy.init_node('nodo_servicio_servidor')
    rospy.loginfo("Nodo del servidor de servicio inicializado")
    rospy.loginfo(calcular_area_rect)
		
    service = rospy.Service('calculadora_area_rect', Area_rectangle, calcular_area_rect)
		
    rospy.spin()

if __name__ == '__main__':
    mi_servicio_server()