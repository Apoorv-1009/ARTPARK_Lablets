#!/usr/bin/env python
import rospy

if __name__ == "__main__":
    rospy.init_node("ultrasound_sensor_driver", anonymous=False)
    rospy.loginfo("I process ultrasound sensor data")

    rospy.spin()

