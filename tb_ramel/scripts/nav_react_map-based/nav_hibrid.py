#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseActionGoal
from calculos_vectoriales import calculate_desired_angle,calculate_distance_to_goal,normalize_angle

# Constantes de control
MAX_LINEAR_VELOCITY = 0.2  # Velocidad lineal máxima del robot
MAX_ANGULAR_VELOCITY = 0.5  # Velocidad angular máxima del robot
MIN_DISTANCE = 0.5  # Distancia mínima para considerar que el obstáculo ha sido evitado
GOAL_POSITION = [-1.0, 3.0]  # Posición objetivo

# Variables de control
obstacle_detected = False  # Indicador de detección de obstáculo
obstacle_distance = 0.0  # Distancia al obstáculo más cercano
robot_pose = [0.0, 0.0, 0.0]  # Posición y orientación del robot [x, y, theta]
map_data = None  # Datos del mapa

# Publicadores de comandos de velocidad y objetivo de navegación
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)

##########################################
### CALLBACKS => LaserScan, ODOM y MAP ###
##########################################
def laser_callback(msg):
    global obstacle_detected, obstacle_distance

    # Obtener la distancia más cercana del láser
    obstacle_distance = msg.ranges[0]

    # Verificar si se ha detectado un obstáculo
    print("---------------------")
    print(obstacle_distance)
    if obstacle_distance < MIN_DISTANCE:
        obstacle_detected = True
    else:
        obstacle_detected = False

def odom_callback(msg):
    global robot_pose

    # Obtener la posición y orientación del robot de los mensajes de odometría
    orientation = msg.pose.pose.orientation
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    _, _, yaw = euler_from_quaternion(quaternion)

    robot_pose[0] = msg.pose.pose.position.x
    robot_pose[1] = msg.pose.pose.position.y
    robot_pose[2] = yaw

def map_callback(msg):
    global map_data
    map_data = msg
##########################################
##########################################

# ------ NAVEGACION HIBRIDA 
def hybrid_navigation():
    # Crear objeto Twist para comandos de velocidad
    cmd_vel = Twist()

    rate = rospy.Rate(10)  # Frecuencia de publicación de comandos de velocidad (10 Hz)

    while not rospy.is_shutdown():
        if obstacle_detected:
            # Evitar obstáculos mediante navegación reactiva
            reactive_navigation(cmd_vel)
        else:
            # Seguir la ruta planificada en el mapa
            global_navigation(cmd_vel)

        # Publicar los comandos de velocidad
        cmd_vel_pub.publish(cmd_vel)

        rate.sleep()


# ------ NAVEGACION REACTIVA ------
def reactive_navigation(cmd_vel):
    # Implementar tu lógica de navegación reactiva aquí
    # Puedes utilizar la distancia al obstáculo más cercano y la orientación del robot

    # Ejemplo: Girar en sentido antihorario si hay un obstáculo
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = MAX_ANGULAR_VELOCITY


# ------ NAVEGACION GLOBAL ------
def global_navigation(cmd_vel):
    # Implementar tu lógica de navegación global aquí
    # Puedes utilizar el mapa y la posición objetivo

    # Ejemplo: Calcular el ángulo hacia el objetivo deseado
    desired_angle = calculate_desired_angle(GOAL_POSITION,robot_pose)

    # Calcular el ángulo hacia el objetivo considerando la orientación actual del robot
    angle = desired_angle - robot_pose[2]

    # Limitar el ángulo a un rango de -pi a pi
    angle = normalize_angle(angle)

    # Girar hacia el objetivo
    cmd_vel.angular.z = MAX_ANGULAR_VELOCITY if angle > 0 else -MAX_ANGULAR_VELOCITY

    # Calcular la distancia al objetivo
    distance = calculate_distance_to_goal(GOAL_POSITION,robot_pose)

    # Avanzar hacia el objetivo con velocidad lineal proporcional a la distancia
    if distance <= 0.3:
        cmd_vel.linear.x = 0.0
    else:
        cmd_vel.linear.x = min(MAX_LINEAR_VELOCITY, distance)

if __name__ == '__main__':
    rospy.init_node('hybrid_navigation')

    # Suscribirse al tópico del láser
    rospy.Subscriber('/scan', LaserScan, laser_callback)

    # Suscribirse al tópico de odometría
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Suscribirse al tópico del mapa
    rospy.Subscriber('/map', OccupancyGrid, map_callback)

    # Iniciar la navegación híbrida
    hybrid_navigation()
