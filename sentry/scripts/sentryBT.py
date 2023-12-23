#! /usr/bin/env python
import rospy
if __name__ == "__main__":
    is_get = False
    while not rospy.is_shutdown():
        is_get = rospy.get_param("is_get") 
        print(is_get)