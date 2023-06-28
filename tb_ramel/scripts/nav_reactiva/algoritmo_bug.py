#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Constantes de control
MAX_LINEAR_VELOCITY = 0.2  # Velocidad lineal máxima del robot
MAX_ANGULAR_VELOCITY = 0.5  # Velocidad angular máxima del robot
MIN_DISTANCE = 1  # Distancia mínima para considerar que el obstáculo ha sido evitado

# Variables de control
obstacle_detected = False  # Indicador de detección de obstáculo
obstacle_distance = 0.0  # Distancia al obstáculo más cercano

# Publicador de comandos de velocidad
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# callback que se ejecuta cada vez que se llega un valor nuevo de LaserScan
def laser_callback(msg):
    global obstacle_detected, obstacle_distance

    # Obtener la distancia más cercana del láser
    obstacle_distance = min(msg.ranges)

    # Verificar si se ha detectado un obstáculo
    if obstacle_distance < MIN_DISTANCE:
        obstacle_detected = True
    else:
        obstacle_detected = False

# ----- algoritmo bug -----
def bug_algorithm():
    # Crear objeto Twist para comandos de velocidad
    cmd_vel = Twist()

    # Velocidad lineal inicial
    cmd_vel.linear.x = MAX_LINEAR_VELOCITY

    # Velocidad angular inicial
    cmd_vel.angular.z = 0.0

    rate = rospy.Rate(10)  # Frecuencia de publicación de comandos de velocidad (10 Hz)

    while not rospy.is_shutdown():
        if obstacle_detected:
            # Girar en sentido antihorario si hay un obstáculo
            cmd_vel.angular.z = MAX_ANGULAR_VELOCITY
        else:
            # Continuar moviéndose hacia adelante si no hay obstáculos
            cmd_vel.angular.z = 0.0

        # Publicar los comandos de velocidad
        cmd_vel_pub.publish(cmd_vel)

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('algoritmo_navegacion_bug')

    # Suscribirse al tópico del láser
    rospy.Subscriber('/scan', LaserScan, laser_callback)

    # Iniciar el algoritmo "bug algorithm"
    bug_algorithm()
