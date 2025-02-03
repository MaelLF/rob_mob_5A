#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class VelocityControl:

    def __init__(self):
        rospy.init_node('velocity_control')
        self.pepper_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.joy_subscriber = rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=10)
        self.linear_scale = 1.0
        self.angular_scale = 1.0
        self.reel_coef = 0.2
        self.wait_for_joy = 0
        self.last_twist = Twist()
        
        # Timer pour publier à une fréquence fixe
        self.publish_rate = rospy.Rate(10)  # 10 Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_velocity)
    
    def joy_callback(self, joy):
        # Ajuster les valeurs des axes en fonction d'une zone morte (seuil 0.15)
        self.wait_for_joy = 1
        axes = list(joy.axes)
        for i in range(len(axes)):
            if abs(axes[i]) < 0.15:
                axes[i] = 0.0

        # Calculer les vitesses linéaires et angulaires
        linear_speed_x = axes[1] * self.linear_scale  # Joystick gauche pour la vitesse linéaire
        angular_speed_z = axes[3] * self.angular_scale  # Joystick droit pour la vitesse angulaire

        # Mettre à jour la dernière commande de vitesse
        self.last_twist.linear.x = linear_speed_x * self.reel_coef
        self.last_twist.angular.z = angular_speed_z * self.reel_coef

    def publish_velocity(self, event):
        # Publier la dernière commande de vitesse à une fréquence fixe
        if self.wait_for_joy :
            self.pepper_publisher.publish(self.last_twist)

def main():
    velocity_control = VelocityControl()
    rospy.spin()

if __name__ == '__main__':
    main()
