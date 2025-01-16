#!/usr/bin/env python3

import rospy
from rss2_msgsrv_pkg.srv import turtlebot_move_square, turtlebot_move_squareResponse
from geometry_msgs.msg import Twist
import time

def move_straight(side_length):
    vel.linear.x = 0.2
    vel.angular.z = 0.0
    move_time = side_length / 0.2
    start_time = time.time()
    while (time.time() - start_time) < move_time:
        pw_pub.publish(vel)
        rate.sleep()
    vel.linear.x = 0.0
    pw_pub.publish(vel)
    rate.sleep()

def turn_90_degrees():
    vel.linear.x = 0.0
    vel.angular.z = 0.5
    turn_time = 1.57 / 0.5  # time to turn 90 degrees (1.57 radians)
    start_time = time.time()
    while (time.time() - start_time) < turn_time:
        pw_pub.publish(vel)
        rate.sleep()
    vel.angular.z = 0.0
    pw_pub.publish(vel)
    rate.sleep()

def move_square(side_length):
    for _ in range(4):
        move_straight(side_length)
        turn_90_degrees()

def my_callback(request):
    rospy.loginfo('Turtlebot_move_square_service has been called')
    success = True
    try:
        for _ in range(request.repetitions):
            move_square(request.sideLength)
    except Exception as e:
        rospy.logerr(f"Error during movement: {e}")
        success = False
    return turtlebot_move_squareResponse(success)

rospy.init_node('turtlebot_move_square_server')

pw_service = rospy.Service('/turtlebot_move_square_service', turtlebot_move_square, my_callback)

pw_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
vel = Twist()

rate = rospy.Rate(10)

rospy.loginfo('Service /turtlebot_move_square_service is ready!')

rospy.spin()

