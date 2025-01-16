#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class MoveTurtleBot():

    def __init__(self):
        self.turtlebot_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.turtlebot_vel_publisher.get_num_connections()
            if connections > 0:
                self.turtlebot_vel_publisher.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.stop_turtlebot()
        self.ctrl_c = True

    def stop_turtlebot(self):
        rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    # Method to move straight
    def move_straight(self, moving_time, speed):
        self.cmd.linear.x = speed
        self.cmd.angular.z = 0.0
        i = 0
        rospy.loginfo("Moving straight!")
        while not self.ctrl_c and i < moving_time:
            self.publish_once_in_cmd_vel()
            i += 1
            self.rate.sleep()

    # Method to turn
    def turn(self, turning_time, angular_speed):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = angular_speed
        i = 0
        rospy.loginfo("Turning!")
        while not self.ctrl_c and i < turning_time:
            self.publish_once_in_cmd_vel()
            i += 1
            self.rate.sleep()

    # Method to move in a square
    def move_in_square(self, side_length, speed, angular_speed):
        for _ in range(4):
            self.move_straight(side_length, speed)
            self.turn(3, angular_speed)  # Increased turning time to ensure a 90-degree turn
        self.stop_turtlebot()

if __name__ == '__main__':
    rospy.init_node('move_turtlebot_square', anonymous=True)
    moveturtlebot_object = MoveTurtleBot()
    try:
        moveturtlebot_object.move_in_square(5, 0.2, 0.5)  # Adjusted angular speed
    except rospy.ROSInterruptException:
        pass

