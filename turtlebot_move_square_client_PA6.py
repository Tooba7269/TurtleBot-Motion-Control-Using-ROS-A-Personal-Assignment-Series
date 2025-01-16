#!/usr/bin/env python3

import rospy
from rss2_msgsrv_pkg.srv import turtlebot_move_square, turtlebot_move_squareRequest

rospy.init_node('turtlebot_move_square_client')

rospy.wait_for_service('/turtlebot_move_square_service')

try:
    turtlebot_service_client = rospy.ServiceProxy('/turtlebot_move_square_service', turtlebot_move_square)
    request = turtlebot_move_squareRequest()
    request.sideLength = 1.0  # Example side length
    request.repetitions = 2  # Example repetitions
    response = turtlebot_service_client(request)
    rospy.loginfo('Service call result: %s', response.success)
except rospy.ServiceException as e:
    rospy.logerr('Service call failed: %s', e)

