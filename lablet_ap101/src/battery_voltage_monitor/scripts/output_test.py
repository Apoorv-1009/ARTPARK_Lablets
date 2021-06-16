#!/usr/bin/env python
import rospy as ry

if __name__ == "__main__":
    ry.init_node("output_test", anonymous=False)
    ry.loginfo("This is an output test!")
    
    ry.spin()
