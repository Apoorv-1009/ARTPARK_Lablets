#!/usr/bin/env python

import rospy 
from lablet_ap101_msgs.srv import LineGenerator,LineGeneratorResponse 
from numpy import linspace

def server_callback(req):
    #This is the service server callback
    response = LineGeneratorResponse()
    response.line_points = list(linspace(req.initial_value, req.final_value, req.line_resolution))
    rospy.loginfo(response)
    return response

if __name__ == "__main__":
    #Initialize ros node
    rospy.init_node("line_generator_server", anonymous=False)

    #Initialize the ROS service server
    rospy.Service("line_generator", LineGenerator, server_callback)
    rospy.spin()
