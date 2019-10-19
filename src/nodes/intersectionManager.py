#!/usr/bin/env python  
import rospy
from aim.srv import *

# Incase we need transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg

# This gets called from the client (car_Manager.py service) with the car's parameters as the req variable.
def handle_car_request(req):
    print "Requested car's info [%s  %s  %s  %s  %s %s %s  %s %s %s]"%(req.car_id, req.lane_id, req.priority, req.t, req.x, req.y, req.heading, req.angular_V, req.vel, req.acc)  
    successfully_scheduled = schedule(req)
    return IntersectionManagerResponse(successfully_scheduled)

# Temporary function names
def schedule(car)

def checkCollision()

def createTrajectory()

def main():
    rospy.init_node('intersection_manager_server')
    rate = rospy.Rate(100.0)

    # To get transform from TF
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    s = rospy.Service('car_request', IntersectionManager, handle_car_request)
    
    while not rospy.is_shutdown():
    
        rate.sleep()

if __name__ == '__main__':
    main()