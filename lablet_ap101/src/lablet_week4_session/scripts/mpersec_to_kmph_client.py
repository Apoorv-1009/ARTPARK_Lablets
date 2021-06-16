#!/usr/bin/env python 

import rospy
from lablet_ap101_msgs.srv import MetresPerSecToKmph, MetresPerSecToKmphRequest
if __name__ == "__main__":
    #Initialize the client node
    rospy.init_node("mpersec_to_kmph_client", anonymous=False)

    rospy.logwarn("Waiting for service to be advertised...")
    #Wait for service to be advertised
    rospy.wait_for_service("mpersec_to_kmph")

    service_client = rospy.ServiceProxy("mpersec_to_kmph", MetresPerSecToKmph)
    client_req = MetresPerSecToKmphRequest()
    client_req.m_per_sec = rospy.get_param("~requested_speed")
    client_req.robot_name = MetresPerSecToKmphRequest.ROBOT_FETCH
    
    resp = service_client(client_req)
    rospy.loginfo(resp)
