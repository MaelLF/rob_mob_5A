#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class VelocityControl:

    def __init__(self):
        rospy.init_node('velocity_control')
        self.pepper_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1000)
        self.joy_subscriber = rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1000)
        self.linear_scale = 1.0
        self.angular_scale = 1.0
        self.reel_coef = 0.2

    def joy_callback(self, joy):
        # Adjust axes values based on deadzone (0.15 threshold)
        axes = list(joy.axes)
        for i in range(len(axes)):
            if abs(axes[i]) < 0.15:
                axes[i] = 0.0

        # Calculate linear and angular speeds
        linear_speed_x = axes[1] * self.linear_scale  # Left joystick for linear velocity
        linear_speed_y = axes[0] * self.linear_scale  # Right joystick for lateral velocity
        angular_speed_z = axes[3] * self.angular_scale  # Right joystick for angular velocity

        # Create Twist message with linear and angular velocities
        twist_cmd = Twist()
        twist_cmd.linear.x = linear_speed_x * self.reel_coef
        twist_cmd.linear.y = 0.0
        twist_cmd.linear.z = 0.0  # Not used in this example
        twist_cmd.angular.x = 0.0  # Not used in this example
        twist_cmd.angular.y = 0.0  # Not used in this example
        twist_cmd.angular.z = angular_speed_z * self.reel_coef

        # Publish Twist message
        self.pepper_publisher.publish(twist_cmd)


def main():
    velocity_control = VelocityControl()
    rospy.spin()

if __name__ == '__main__':
    main()
