#!/usr/bin/env python

import rospy 
from lablet_ap101_msgs.srv import MetresPerSecToKmph, MetresPerSecToKmphResponse

def server_callback(req):
    #This is the service server callback
    response = MetresPerSecToKmphResponse()
    response.kmph = req.m_per_sec * MetresPerSecToKmphResponse.CONVERSION_FACTOR
    response.received_robot_name = req.robot_name
    return response

if __name__ == "__main__":
    #Initialize ros node
    rospy.init_node("mpersec_to_kmph_server", anonymous=False)

    #Initialize the ROS service server
    rospy.Service("mpersec_to_kmph", MetresPerSecToKmph, server_callback)
    rospy.spin()
