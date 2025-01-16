#!/usr/bin/env python3

import rospy
from rss2_msgsrv_pkg.srv import srv_turtlebot_move, srv_turtlebot_moveRequest

# The service client node
rospy.init_node('turtlebot_move_client')

# Wait for the service '/turtlebot_move_service' to run
# You need to start the service first
rospy.wait_for_service('/turtlebot_move_service')

# Connect to the service '/turtlebot_move_service'
turtlebot_service_client = rospy.ServiceProxy('/turtlebot_move_service', srv_turtlebot_move)

# Create a request instance
turtlebot_request_instance = srv_turtlebot_moveRequest()
turtlebot_request_instance.duration = 30  # Total duration is 30 seconds

# Send the request to the server through the connection built
feedback = turtlebot_service_client(turtlebot_request_instance)

# Show results after the service is called
rospy.loginfo(str(feedback))

rospy.loginfo('End of service call')


