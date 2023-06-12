#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

def poseCallback(pose_message):
    global x
    global y, yaw
    x= pose_message.x
    y= pose_message.y
    yaw = pose_message.theta

# publicador, velocidad, disntancia en el eje x, va hacia delante?
def move(velocity_publisher,speed, distance, is_forward):
    velocity_message = Twist()
    
    # ubicacion actual
    global x,y
    x0=x
    y0=y

    # movimiento lineal hacia delante o atras
    if (is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10) 

    while True :
        rospy.loginfo("Turtlesim moviendose!")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()

        # calculo de la distanca movida
        distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print(distance_moved)
        # validar si la distancia por mover supera a la distancia establecida
        if  (distance_moved>=distance):
            rospy.loginfo("Distancia alcanzada")
            break
            
    # detener el robot una vez completada la trayectoria
    velocity_message.linear.x =0
    velocity_publisher.publish(velocity_message)

# velocidad angular, angulo de rotacion, va en sentido horario?
def rotate (angular_speed_degree, relative_angle_degree, clockwise):
    
    global yaw
    velocity_message = Twist()
    velocity_message.linear.x=0
    velocity_message.linear.y=0
    velocity_message.linear.z=0
    velocity_message.angular.x=0
    velocity_message.angular.y=0
    velocity_message.angular.z=0

    # obtener la velocidad actual
    theta0=yaw

    # transformar de grados a radianes
    angular_speed=math.radians(abs(angular_speed_degree))
    
    # validar si el giro es en sentido horario
    if (clockwise):
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    # movimiento del angulo empieza en 0
    angle_moved = 0.0
    loop_rate = rospy.Rate(10) 


    cmd_vel_topic='/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True :
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        # tiempo actual en segundos
        t1 = rospy.Time.now().to_sec()

        # formula de movimiento angular theta=deltaT*velocidadAngular
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()

        # validar si alzamos la distancia angular establecida            
        if  (current_angle_degree>relative_angle_degree):
            rospy.loginfo("Rotacion alcanzada")
            break

    # cuando termine el giro paramos al robot
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)

# posicion final: eje x, eje y
def go_to_goal(x_goal, y_goal):
    global x
    global y, yaw

    velocity_message = Twist()
    cmd_vel_topic='/turtle1/cmd_vel'

    while (True):
        # ------------- VELOCIDAD LINEAL - CONTROL P -------------
        # factor k lineal
        K_linear = 0.5 
        # calculo de la distancia usando la posicion de inicio y la posicion de fin
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        # velocidad lineal final
        linear_speed = distance * K_linear

        # ------------- VELOCIDAD ANGULAR - CONTROL P -------------
        # factor k angular
        K_angular = 4.0
        # calculo del angulo de rotacion usando la posicion de inicio y la posicion de fin
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        
        # velocidad angular final
        angular_speed = (desired_angle_goal-yaw)*K_angular

        # ------------- ASIGNAR LAS VELOCIDADES -------------
        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        
        velocity_publisher.publish(velocity_message)
        
        print('x=', x, 'y=',y)

        if (distance <0.01):
            print('Posicion final alcanzada')
            break

# angulo deseado
def setDesiredOrientation(desired_angle_radians):
    # validar si el angulo es menor al angulo deseado menos el actual
    relative_angle_radians = desired_angle_radians - yaw
    if relative_angle_radians < 0:
        clockwise = 1 # 1 provoca True (giro horario)
    else:
        clockwise = 0 # 0 provoca False (giro anti-horario)
    print (relative_angle_radians)
    print (desired_angle_radians)
    # llama a la funcion para rotar el robot
    rotate(30 ,math.degrees(abs(relative_angle_radians)), clockwise)

if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_movimiento_pos', anonymous=True)

        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)

        time.sleep(2)
        #move (velocity_publisher,1.0, 3.0, is_forward=True)
        #rotate (30, 45, clockwise=False)
        #go_to_goal(8, 7)
        setDesiredOrientation(math.radians(90))

        #time.sleep(2)
        #print ('Inicia reset: ')
        #rospy.wait_for_service('reset')
        #reset_turtle = rospy.ServiceProxy('reset', Empty)
        #reset_turtle()
        #print ('Termina reset: ')
        #rospy.spin()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")