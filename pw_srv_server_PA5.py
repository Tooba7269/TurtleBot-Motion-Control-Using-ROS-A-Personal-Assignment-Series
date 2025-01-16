#!/usr/bin/env python3

import rospy
from rss2_msgsrv_pkg.srv import srv_turtlebot_move, srv_turtlebot_moveResponse
from geometry_msgs.msg import Twist

def my_callback(request):
    rospy.loginfo('Turtlebot_move_service has been called')
    
    # for the first 20 seconds
    vel.linear.x = 0.2
    vel.angular.z = 0.2
    total_time = 0
    while total_time < 20:
        pw_pub.publish(vel)
        rospy.loginfo('Moving in a circle, time = %d', total_time)
        rate.sleep()
        total_time += 1

    # Stop for 5 seconds
    vel.linear.x = 0.0
    vel.angular.z = 0.0
    for _ in range(5):
        pw_pub.publish(vel)
        rospy.loginfo('Stopping, time = %d', total_time)
        rate.sleep()
        total_time += 1

    # Move along x-axis for 5 seconds
    vel.linear.x = 0.2
    vel.angular.z = 0.0
    for _ in range(5):
        pw_pub.publish(vel)
        rospy.loginfo('Moving along x-axis, time = %d', total_time)
        rate.sleep()
        total_time += 1

    # Stop the robot
    vel.linear.x = 0.0
    vel.angular.z = 0.0
    pw_pub.publish(vel)
    rospy.loginfo('Stopped, time = %d', total_time)
    
    return srv_turtlebot_moveResponse(True)

rospy.init_node('turtlebot_move_server')

# This is the service called ‘/turtlebot_move_service‘
pw_service = rospy.Service('/turtlebot_move_service', srv_turtlebot_move, my_callback)

pw_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
vel = Twist()

# Make sure counting second by second
rate = rospy.Rate(1)

rospy.loginfo('Service /turtlebot_move_service is ready!')

# Maintain the service
rospy.spin()

