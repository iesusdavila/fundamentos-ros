#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

obstacle_detected = False

def obstacle_detect(scan):
    global obstacle_detected

    print("Detectando obstaculos con el escaner...")

    obst_frente = scan.ranges[0]
    obst_izquierda = scan.ranges[90]
    obst_derecha = scan.ranges[270]
    obst_atras = scan.ranges[180]

    print(f'El obstaculo al frente esta a: ${obst_frente}')
    if obst_frente < 0.75:
        obstacle_detected = True
        rospy.loginfo("¡Peligro - Obstáculo detectado al frente!")

def move_to_goal(xGoal,yGoal):
    global obstacle_detected

    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Esperando respuesta del servidor para mover el robot...")

    goal = MoveBaseGoal()
    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    if obstacle_detected:
        rospy.loginfo("¡Peligro - Obstáculo detectado! Cancelando navegación.")
        obstacle_detected = False
        return False

    rospy.loginfo("Moviendo el robot hasta la posicion de meta...")
    ac.send_goal(goal)

    ac.wait_for_result(rospy.Duration(60))

    # si se detecta un obstaculo se cancela la navegacion
    while ac.get_state() == GoalStatus.ACTIVE:
        if obstacle_detected:
            rospy.loginfo("¡Se ha detectado un obstáculo! Cancelando movimiento...")
            ac.cancel_goal()
            obstacle_detected = False
            return False            

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("TurtleBot completo su navegacion de manera exitosa.")
        return True
    else:
        rospy.loginfo("Hubieron fallas en la navegaciòn del TurtleBot.")
        return False

def move_to_multiple_goals(goal_list):
    for goal in goal_list:
        x_goal, y_goal = goal
        result = move_to_goal(x_goal, y_goal)
        
        if not result:
            rospy.loginfo("Error en la navegación. Abortando movimiento hacia la siguiente ubicación objetivo.")
            break

if __name__ == '__main__':
    rospy.init_node('map_navigation', anonymous=False)
    #x_goal = 3
    #y_goal = 2

    rospy.Subscriber('/scan', LaserScan, obstacle_detect)

    #move_to_goal(x_goal,y_goal)
    print('Comenzando a mover al robot')
    goal_list = [(1, 2.5), (1.75, 1.25), (1, 3), 1, 2]  # Lista de coordenadas objetivo
    move_to_multiple_goals(goal_list)


    rospy.spin()