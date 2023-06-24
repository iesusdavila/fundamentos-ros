#!/usr/bin/env python3
import rospy
from mi_paquete.srv import Mi_servicio
import sys


def mi_servicio_client():
    rospy.wait_for_service('contador_palabras')
    try:
        mi_servicio = rospy.ServiceProxy('contador_palabras', Mi_servicio)
        texto_ingresado = ' '.join(sys.argv[1:])
        contar_palabras = mi_servicio(texto_ingresado)

        return {
            "palabra":texto_ingresado,
            "contador":contar_palabras.contador
            }
        
    except rospy.ServiceException as e:
        rospy.logerr("Error en el servicio: %s" % e)

if __name__ == '__main__':
    rospy.init_node('nodo_servicio_cliente')    
    rospy.loginfo("Nodo del servidor de servicio inicializado")
    
    palabra, contador = mi_servicio_client().values()
    print('"',palabra,'"', 'contiene', contador, 'palabras.')

