#!/usr/bin/env python
import rospy

if __name__ == "__main__":
    rospy.init_node("battery_voltage_monitor", anonymous=False)
    rospy.loginfo("I process battery sensor information.")
    rospy.loginfo("Sensor sampling frequency is = %s",rospy.get_param("~sensor_freq"))
    rospy.spin()
