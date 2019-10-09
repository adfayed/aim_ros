#!/usr/bin/env python  
import rospy
from aim.srv import *

# Incase we need transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg


def main():
	rospy.init_node('car_manager_main')
    rate = rospy.Rate(100.0)

    while not rospy.is_shutdown():

    	rate.sleep()
if __name__ == '__main__':
	main()