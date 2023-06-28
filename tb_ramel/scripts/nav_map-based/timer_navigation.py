#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

def move_to_goal(xGoal,yGoal):

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

    rospy.loginfo("Moviendo el robot hasta la posicion de meta...")
    ac.send_goal(goal)

    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("TurtleBot completo su navegacion de manera exitosa.")
        return True
    else:
        rospy.loginfo("Hubieron fallas en la navegaciòn del TurtleBot.")
        return False

def check_goal_status(event):
    rospy.loginfo("Tiempo límite alcanzado. Deteniendo la navegación.")
    # Aquí puedes agregar las acciones que deseas realizar cuando se alcanza el tiempo límite
    # en este caso si pasan 2 minutos que vuelva a la posiciòn de inicio
    move_to_goal(-3,1)

if __name__ == '__main__':
    rospy.init_node('map_navigation', anonymous=False)
    x_goal = 3
    y_goal = 2

    # espero 2 minutos para saber si he recibido una respuesta, caso contrario ejecuto check_goal_status
    # oneshot=True indica si se va a ejecutar una sola vez ese timer
    tiempo_limite = 120.0
    rospy.Timer(rospy.Duration(tiempo_limite), check_goal_status, oneshot=True)
   
    print('Comenzando a mover al robot')
    move_to_goal(x_goal,y_goal)
    rospy.spin()