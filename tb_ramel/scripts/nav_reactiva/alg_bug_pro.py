#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Constantes de control
MAX_LINEAR_VELOCITY = 0.2  # Velocidad lineal máxima del robot
MAX_ANGULAR_VELOCITY = 0.5  # Velocidad angular máxima del robot
MIN_DISTANCE = 1  # Distancia mínima para considerar un obstaculo
GOAL_POSITION = [-1, 2.0]  # Posición objetivo

# Variables de control
obstacle_detected = False  # Indicador de detección de obstáculo
obstacle_distance = 0.0  # Distancia al obstáculo más cercano
robot_pose = [0.0, 0.0, 0.0]  # Posición y orientación del robot [x, y, theta]

# Publicadores de comandos de velocidad
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# callback se ejecuta cada vez que llega un nuevo valor de LaserScan
def laser_callback(msg):
    global obstacle_detected, obstacle_distance

    # Obtener la distancia más cercana del láser
    obstacle_distance = min(msg.ranges)

    # Verificar si se ha detectado un obstáculo
    if obstacle_distance < MIN_DISTANCE:
        obstacle_detected = True
    else:
        obstacle_detected = False

# callback se ejecuta cada vez que llega un nuevo valor de Odom
def odom_callback(msg):
    global robot_pose

    # Obtener la posición y orientación del robot de los mensajes de odometría
    orientation = msg.pose.pose.orientation
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    _, _, yaw = euler_from_quaternion(quaternion)

    posicion_x = msg.pose.pose.position.x
    posicion_y = msg.pose.pose.position.y
    
    print(f'Posicion en las coordenadas: ({posicion_x},{posicion_y})')
    robot_pose[0] = posicion_x
    robot_pose[1] = posicion_y
    robot_pose[2] = yaw # yaw es rotacion sobre el eje z

# ------ algoritmo bug ------
def bug_algorithm():
    # Crear objeto Twist para comandos de velocidad
    cmd_vel = Twist()

    rate = rospy.Rate(10)  # Frecuencia de publicación de comandos de velocidad (10 Hz)

    while not rospy.is_shutdown():
        if obstacle_detected:
            # Calcular el ángulo hacia el objetivo deseado
            desired_angle = calculate_desired_angle()

            # Calcular el ángulo hacia el objetivo considerando la orientación actual del robot
            angle = desired_angle - robot_pose[2]

            # Limitar el ángulo a un rango de -pi a pi
            angle = normalize_angle(angle)

            # Girar en sentido antihorario si hay un obstáculo
            cmd_vel.angular.z = MAX_ANGULAR_VELOCITY if angle > 0 else -MAX_ANGULAR_VELOCITY

            # Calcular la distancia al objetivo deseado
            distance = calculate_distance_to_goal()

            # Detenerse si se encuentra cerca del objetivo
            print(f'La distancia hasta el objetivo es: {GOAL_POSITION[0] - robot_pose[0]},{GOAL_POSITION[1] - robot_pose[1]}')
            print(f'La distancia objetivo es: {GOAL_POSITION[0]},{GOAL_POSITION[1]}')

            if distance == 0:
                cmd_vel.linear.x = 0.0
            else:
                cmd_vel.linear.x = MAX_LINEAR_VELOCITY
        else:
            # Continuar moviéndose hacia adelante si no hay obstáculos
            cmd_vel.linear.x = MAX_LINEAR_VELOCITY
            cmd_vel.angular.z = 0.0

        # Publicar los comandos de velocidad
        cmd_vel_pub.publish(cmd_vel)

        rate.sleep()

# calculo del angulo por medio de la tg() hacia el objetivo
def calculate_desired_angle():
    # Calcular el ángulo hacia el objetivo deseado - tg(opuesto/adyacente)
    dx = GOAL_POSITION[0] - robot_pose[0]
    dy = GOAL_POSITION[1] - robot_pose[1]
    return math.atan2(dy, dx)

# calculo de la distancia por medio de pitagoras hacia el objetivo
def calculate_distance_to_goal():
    # Calcular la distancia al objetivo deseado - pitagoras
    dx = GOAL_POSITION[0] - robot_pose[0]
    dy = GOAL_POSITION[1] - robot_pose[1]
    return math.sqrt(dx**2 + dy**2)

# calculo de la normalizacion del angulo de -pi a pi
def normalize_angle(angle):
    # Limitar el ángulo a un rango de -pi a pi
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

if __name__ == '__main__':
    rospy.init_node('nav_alg_bug_odom')

    # Suscribirse al tópico del láser
    rospy.Subscriber('/scan', LaserScan, laser_callback)

    # Suscribirse al tópico de odometría
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Iniciar el algoritmo "bug algorithm"
    bug_algorithm()
