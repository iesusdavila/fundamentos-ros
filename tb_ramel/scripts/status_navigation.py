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

def feedback_callback(feedback):
    # Se ejecuta cada vez que se recibe retroalimentación del estado del robot
    current_state = feedback.status_list[0].status

    if current_state == GoalStatus.ACTIVE:
        rospy.loginfo("El robot está en movimiento hacia la posición objetivo...")
    elif current_state == GoalStatus.PREEMPTED:
        rospy.loginfo("El objetivo fue cancelado o reemplazado por uno nuevo.")
    elif current_state == GoalStatus.SUCCEEDED:
        rospy.loginfo("El robot alcanzó la posición objetivo con éxito.")
    elif current_state == GoalStatus.ABORTED:
        rospy.loginfo("La navegación del robot fue abortada debido a un error.")
    elif current_state == GoalStatus.REJECTED:
        rospy.loginfo("El objetivo fue rechazado por el sistema de navegación.")
    elif current_state == GoalStatus.PREEMPTING:
        rospy.loginfo("El objetivo actual está siendo reemplazado por uno nuevo.")

if __name__ == '__main__':
    rospy.init_node('map_navigation', anonymous=False)
    x_goal = 3
    y_goal = 2
   
    # Suscriptor al topic /move_base/status
    rospy.Subscriber('/move_base/status', GoalStatusArray, feedback_callback)

    print('Comenzando a mover al robot')
    move_to_goal(x_goal,y_goal)
    rospy.spin()