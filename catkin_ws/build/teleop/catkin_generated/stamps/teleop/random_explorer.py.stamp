#!/usr/bin/env python3

import rospy
import random
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class VelocityControl:

    def __init__(self):
        rospy.init_node('random_exploration')
        self.pepper_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1000)
        self.joy_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1000)
        self.coef_reel = 1.0 #coef to pass from the simulation in buvette map to real application
        self.minimal_range = 0.5  #can be changed in real application
        self.counter = 0
        self.sens = 1
        self.min_index = None
        self.min_range = None
        self.iteration_angle = 0

    def laser_callback(self, scan):
        valid_ranges = [r for r in scan.ranges if r > 0 and r != float('inf')] #check for the ranges that are comform to this definition
        if not valid_ranges: #no valid range in /scan msg
            return

        self.min_range = min(valid_ranges)
        self.min_index = valid_ranges.index(self.min_range)
        #print(self.min_index)
        #print(len(scan.ranges))

        if self.iteration_angle < 100 :

            if self.min_index < len(scan.ranges)/2 : 
                self.sens = 1
            else : 
                self.sens = -1

        #     self.counter = self.counter +1

        # if self.counter > 500 : #quand on a fait 500 fois de la ligne droite le robot change le sens de rotation quand il rencontre un objet
        #     twist_cmd = Twist()
        #     twist_cmd.linear.x = 0
        #     twist_cmd.angular.z = random.uniform(1.0, 2.0) * self.coef_reel
        #     self.pepper_publisher.publish(twist_cmd)
        #     return


        if self.counter < 520 :
            self.counter = 0

        
        #rospy.loginfo(f"Distance minimale détectée : {min_range:.2f} m")

        if self.min_range < self.minimal_range:
            self.avoid_obstacle()
  
        else:
            self.move_randomly()

    
    def avoid_obstacle(self):
        twist_cmd = Twist()
        twist_cmd.linear.x = -random.uniform(0.5, 0.8)  * self.coef_reel
        twist_cmd.angular.z = self.sens * random.uniform(0.5, 1.0) * self.coef_reel
        self.pepper_publisher.publish(twist_cmd)
        self.iteration_angle = self.iteration_angle + 5



    def move_randomly(self):
        self.iteration_angle = self.iteration_angle - 1
        twist_cmd = Twist()
        twist_cmd.linear.x = random.uniform(0.5, 0.8)  * self.coef_reel
        twist_cmd.angular.z = random.uniform(-1.0, 1.0) * self.coef_reel
        self.pepper_publisher.publish(twist_cmd)



def main():
    velocity_control = VelocityControl()
    rospy.spin()

if __name__ == '__main__':
    main()
