#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class RobotController:
    def __init__(self):
    
        rospy.init_node('ToobaZahid_pubsub')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.initial_position = None
        self.goal_distance = 2.0  # Meters

    def odom_callback(self, msg):
    
        if self.initial_position is None:
            self.initial_position = msg.pose.pose.position

        current_position = msg.pose.pose.position
        distance = math.sqrt((current_position.x - self.initial_position.x)**2 +
                             (current_position.y - self.initial_position.y)**2)

        if distance >= self.goal_distance:
            self.publish_velocity(0, 0)  # Stop the robot
            rospy.signal_shutdown('Reached Goal Distance')
        else:
            self.publish_velocity(0.5, 0)  # Continue moving forward

    def publish_velocity(self, linear, angular):
        move_cmd = Twist()
        move_cmd.linear.x = linear
        move_cmd.angular.z = angular
        self.pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        controller = RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

