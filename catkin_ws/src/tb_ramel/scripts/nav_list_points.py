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

def move_to_multiple_goals(goal_list):
    for goal in goal_list:
        x_goal, y_goal = goal
        result = move_to_goal(x_goal, y_goal)
        
        if not result:
            rospy.loginfo("Error en la navegación. Abortando movimiento hacia la siguiente ubicación objetivo.")
            break

if __name__ == '__main__':
    rospy.init_node('map_navigation', anonymous=False)
    print('Comenzando a mover al robot')

    #x_goal = 3
    #y_goal = 2
    #move_to_goal(x_goal,y_goal)

    goal_list = [(1, 2.5), (3, 4), (5, 6)]  # Lista de coordenadas objetivo
    move_to_multiple_goals(goal_list)
    rospy.spin()