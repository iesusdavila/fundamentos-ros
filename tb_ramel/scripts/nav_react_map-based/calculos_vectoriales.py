import math

def calculate_desired_angle(GOAL_POSITION, robot_pose):
    # Calcular el ángulo hacia el objetivo deseado
    dx = GOAL_POSITION[0] - robot_pose[0]
    dy = GOAL_POSITION[1] - robot_pose[1]
    return math.atan2(dy, dx)

def calculate_distance_to_goal(GOAL_POSITION, robot_pose):
    # Calcular la distancia al objetivo deseado
    dx = GOAL_POSITION[0] - robot_pose[0]
    dy = GOAL_POSITION[1] - robot_pose[1]
    return math.sqrt(dx**2 + dy**2)

def normalize_angle(angle):
    # Limitar el ángulo a un rango de -pi a pi
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle