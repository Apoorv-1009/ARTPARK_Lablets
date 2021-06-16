#!/usr/bin/env python

import rospy 
from lablet_ap101_msgs.msg import StudentAttendance

def student_attendance_cb(msg):
    rospy.loginfo("First student: %s", msg.student_names[0])

if __name__ == "__main__":
    rospy.init_node("student_attendance_subscriber", anonymous=False)
    rospy.Subscriber('student_attendance_data', StudentAttendance, student_attendance_cb)
    rospy.spin()
