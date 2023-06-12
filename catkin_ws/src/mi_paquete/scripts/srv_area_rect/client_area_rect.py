#!/usr/bin/env python3

import rospy # libreria de python para ros
from mi_paquete.srv import Area_rectangle 
import sys 

def mi_servicio_client():
    rospy.wait_for_service('calculadora_area_rect')
    try:
        mi_servicio = rospy.ServiceProxy('calculadora_area_rect', Area_rectangle)
        
        print(f'Realizando la peticion al servicio {mi_servicio.resolved_name}')

        x = float(sys.argv[1])
        y = float(sys.argv[2])

        area_rectangle = mi_servicio(x,y)
				
        return area_rectangle.area
        
    except rospy.ServiceException as e:
        rospy.logerr("Error en el servicio: %s" % e)

if __name__ == '__main__':
    rospy.init_node('nodo_servicio_cliente')    
    rospy.loginfo("Nodo del servidor de servicio inicializado")
    
    print(f'El area del rectangulo es: {mi_servicio_client()}')