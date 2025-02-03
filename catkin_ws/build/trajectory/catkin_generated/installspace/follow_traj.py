#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import tf2_ros

class TrajectoryFollower:
    def __init__(self):
        rospy.init_node('trajectory_follower', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_sub = rospy.Subscriber('/planned_path', Path, self.path_callback)
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer) #Transformation stockage creation
        
        self.path = []
        self.current_target_index = 0 #targeted point index
        self.goal_tolerance = 0.4  # Tolerance to consider the robot has reached a point
        self.rate = rospy.Rate(100)  # 100 Hz
        self.reel_coef = 0.2

    def path_callback(self, msg):
        """Callback to receive the planned path."""
        rospy.loginfo("Trajectoire reçue!")
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses] #Path from /path_planning goes into argument
        self.current_target_index = 0 
        self.follow_trajectory() 

    def get_current_position(self):
        """Get the current position of the robot in the map frame."""
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0)) #get transform from the buffer
            position = transform.transform.translation #get the translation value from transform
            yaw = self.get_yaw_from_quaternion(transform.transform.rotation) #get the rotation value from transform
            return (position.x, position.y, yaw)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException): #Error exeception 
            rospy.logwarn("Impossible de récupérer la position actuelle du robot.")
            return None

    def get_yaw_from_quaternion(self, q): #Convert quaternion got from tf
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def follow_trajectory(self): #Fonction pricipale
        """Follow the trajectory point by point."""
        if not self.path:
            rospy.logwarn("Aucune trajectoire à suivre!")
            return

        rospy.loginfo("Début du suivi de trajectoire...")
        while self.current_target_index < len(self.path) and not rospy.is_shutdown():
            target_x, target_y = self.path[self.current_target_index]
            self.move_to_point(target_x, target_y)

        rospy.loginfo("Trajectoire terminée!")
        self.stop_robot()

    def move_to_point(self, target_x, target_y):
        """Move the robot to a specific point."""
        while not rospy.is_shutdown():
            current_position = self.get_current_position()
            if not current_position:
                continue  # Retry if position is not there yet
            x, y, yaw = current_position

            # Compute errors
            dx = target_x - x
            dy = target_y - y
            distance = math.sqrt(dx**2 + dy**2) #distance to target

            if distance < self.goal_tolerance:
                rospy.loginfo("Point atteint: (%.2f, %.2f)", target_x, target_y)
                self.current_target_index += 1
                return  # Move to the next point

            # Compute angular error
            target_theta = math.atan2(dy, dx)
            angle_error = target_theta - yaw

            # Normalize angle error
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            

            # Proportional control
            linear_velocity = 0.5 * distance *self.reel_coef # Linear gain
            angular_velocity = 0.9* angle_error * self.reel_coef  # Angular gain

            # Limit velocities
            linear_velocity = max(min(linear_velocity, 0.5*self.reel_coef), -0.5*self.reel_coef)
            angular_velocity = max(min(angular_velocity, 0.9*self.reel_coef), -0.9*self.reel_coef)

            # Publish velocity command
            cmd_vel = Twist()
            cmd_vel.linear.x = linear_velocity
            cmd_vel.angular.z = angular_velocity
            self.cmd_vel_pub.publish(cmd_vel)

            self.rate.sleep()

    def stop_robot(self):
        """Stop the robot."""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)


if __name__ == '__main__':
    try:
        follower = TrajectoryFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
