#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import heapq
import cv2
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped


def occupancy_grid_to_matrix(occupancy_grid, max_distance=20):
    # Conversion du grid en matrice
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    data = np.array(occupancy_grid.data, dtype=np.int8).reshape((height, width))

    # Remplacement des valeurs non atteignables (-1) par 100 pour simplifier le traitement
    data[data == -1] = 100
    
    # Convertir les données pour OpenCV (0 pour libre, 255 pour obstacles)
    binary_map = np.where(data > 50, 255, 0).astype(np.uint8)

    # Calculer la distance transformée (distance à l'obstacle le plus proche)
    distance_transform = cv2.distanceTransform(255 - binary_map, cv2.DIST_L2, 5)

    # Normaliser la distance transformée en valeurs de 0 (obstacle) à 100 (libre mais proche d'un obstacle)
    normalized_distance = np.clip(distance_transform / max_distance * 100, 0, 100).astype(np.int8)

    # Combiner les obstacles originaux avec la distance transformée
    progressive_map = np.where(binary_map == 255, 100, 100 - normalized_distance)

    return progressive_map



def heuristic(a, b): #Distance diagonale
    dx = abs(a[0] - b[0]) 
    dy = abs(a[1] - b[1]) 
    h = (dx+dy)+(math.sqrt(2) - 2)*min(dx,dy)
    return h

def astar(grid, start, goal):
    #Algo choisi : Astar avec heuristique distance diagonale
    height, width = grid.shape
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {} #Historique des positions
    g_score = {start: 0} #Couts depuis le départ
    f_score = {start: heuristic(start, goal)} #cout heuristique jusqu'à arrivée

    while open_list:
        _, current = heapq.heappop(open_list)
        if current == goal: # Path found
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Itération à l'envers

        for dx, dy in [(1, 1),(-1,1),(1,-1),(-1,-1),(-1, 0), (1, 0), (0, -1), (0, 1)]: #déplacement élémentaires selon une direction (-x +x -y +y et diagonales 1 1)
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < height and 0 <= neighbor[1] < width and grid[neighbor[0], neighbor[1]] < 75: #ssi dans la grille
                tentative_g_score = g_score[current] + 1 + grid[neighbor[0], neighbor[1]] #privilégie les diagonales car coûts égals au déplacements élémentaires
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
    return None  # No path found


class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=True)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.goal_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.goal = None
        self.grid = None
        self.map_info = None
        self.map_received = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(f"Nouveau point d'arrivée reçu : {self.goal}")
        current_position = self.get_current_position()
        if current_position:
            self.plan_path(current_position)


    def map_callback(self, msg):
        self.grid = occupancy_grid_to_matrix(msg)
        self.map_info = msg.info
        self.map_received = 1 #Int to get map only one time 

    def plan_path(self, start):
        if self.grid is None:
            rospy.logwarn("Pas de map reçue!")
            return
        #Convert goal and start point
        start_cell = self.world_to_grid(start) 
        goal_cell = self.world_to_grid(self.goal)
        path = astar(self.grid, start_cell, goal_cell)
        if path:
            rospy.loginfo("Path trouvée")
            self.publish_path(path)
        else:
            rospy.logwarn("No path found")

#Convertion between the two plans
    def world_to_grid(self, pose): 
        x = int((pose[0] - self.map_info.origin.position.x) / self.map_info.resolution)
        y = int((pose[1] - self.map_info.origin.position.y) / self.map_info.resolution)
        return (y, x)

    def grid_to_world(self, cell):
        x = cell[1] * self.map_info.resolution + self.map_info.origin.position.x
        y = cell[0] * self.map_info.resolution + self.map_info.origin.position.y
        return (x, y)

    def publish_path(self, path):
        ros_path = Path()
        ros_path.header.frame_id = "map"
        for cell in path:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = self.grid_to_world(cell)
            pose.pose.position.z = 0
            ros_path.poses.append(pose)
        self.path_pub.publish(ros_path)

    def get_current_position(self):
        try:
            # Lookup transform from /map to /base_link (or the robot's main frame)
            transform: TransformStamped = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            # Extract position
            position = transform.transform.translation
            return (position.x, position.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Impossible de récupérer la position actuelle du robot.")
            return None



if __name__ == '__main__':
    planner = PathPlanner()
    
    while not planner.goal:
        rospy.loginfo("En attente du point à rallier")
        rospy.sleep(1)
    while not planner.map_received:
        rospy.loginfo("En attente de la réception de la carte...")
        rospy.sleep(1)
    
    rospy.spin()
