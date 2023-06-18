#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

def move_to_goal(xGoal,yGoal):

    # definimos una accion del robot
    # EL nombre de la accion tiene que ser uno existente, en este caso se usa 
    # move_base pero no es el unico que ofrece Ros.
    # Tiene dos parametros: nombre de la accion y el tipo de mensaje de la accion
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    # esperamos 5 segundos hasta que se logre conectar al servidor
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")

    # creamos un objeto para definir el tipo de mensaje a enviar en la accion
    goal = MoveBaseGoal()
    
    # conectamos al frame_id, dicho frame tiene que existir
    goal.target_pose.header.frame_id = "map"
    # registramos la hora de la accion
    goal.target_pose.header.stamp = rospy.Time.now()

    # definimos la posicion y rotacion final de nuestro robot
    # para la posicion creamos un mensaje de tipo Point
    goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Moviendo el robot hasta la posicion de meta ...")
    # enviamos la info del mensaje de tipo MoveBaseGoal para mover el robot
    ac.send_goal(goal)

    # esperamos a que se complete la accion durante 60 segundos
    ac.wait_for_result(rospy.Duration(60))

    # validamos si la accion fue completada
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("TurtleBot completo su navegacion de manera exitosa.")
        return True
    else:
        rospy.loginfo("Hubieron fallas en la navegaciòn del TurtleBot.")
        return False

if __name__ == '__main__':
   rospy.init_node('map_navigation', anonymous=False)
   x_goal = 1
   y_goal = 2.5
   print('Comenzando a mover al robot')
   move_to_goal(x_goal,y_goal)
   rospy.spin()