#!/usr/bin/env python
import rospy 
from lablet_ap101_msgs.msg import StudentAttendance

if __name__ == "__main__":
    rospy.init_node("student_attendance_publisher", anonymous=False)
    rospy.loginfo("I publish attendance data.")
    
    pub = rospy.Publisher('student_attendance_data', StudentAttendance, queue_size = 10)
    delay = rospy.Duration(1)   #Publish this topic every 5 seconds
    #Duration is in seconds

    #Create message object
    student_attendance = StudentAttendance()
    
    while not rospy.is_shutdown():
        student_attendance.student_names= ["Roger", "Rafa", "Serena"]
        student_attendance.attendance_hours = [20.0, 19.0, 23.0]
        student_attendance.student_location.x = 0.0
        student_attendance.student_location.y = 0.0
        student_attendance.student_location.z = 0.0

        pub.publish(student_attendance)
        rospy.loginfo("Successfully published topic.")
        rospy.sleep(delay)
        

    
