#!/usr/bin/env python  
import rospy
from aim.srv import *

# Incase we need transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg

# Class for each car's attr
class car:
    def __init__(self, car_id, lane_id, priority = 0, t, x, y, heading, angular_V, vel, acc, length = 4.9784, width = 1.96342, max_V = 249.448, max_A = 43.47826087, min_A = 43.47826087, max_lateral_g = , reservation = False):
        self.car_id = car_id
        self.lane_id = lane_id
        self.priority = priority
        self.t = t
        self.x = x
        self.y = y
        self.heading = heading
        self.angular_V = angular_V
        self.vel = vel
        self.acc = acc
        self.length = length
        self.width = width
        self.max_V = max_V
        self.max_A = max_A
        self.min_A = min_A
        self.max_lateral_g = max_lateral_g
        self.reservation = reservation

    def _update(self):

class carManager:
	def __init__(self, car_list):
		self.car_list = car_list

	def _sendRequests(self):
		for car in self.car_list if car.reservation is not True:
			car.reservation = car_request_client(car.car_id, car.lane_id, car.priority, car.t, 
				car.x, car.y, car.heading, car.angular_V, car.vel, car.acc, car.length, car.width, car.max_V, car.max_A, car.min_A, car.max_lateral_g)

# This is the client's fuction sending a car's info to the server (intersectionManager.py service) as a request to pass
def car_request_client(car_id, lane_id, priority, t, x, y, heading, angular_V, vel, acc, length, width, max_V, max_A, min_A, max_lateral_g):
    rospy.wait_for_service('car_request')
    try:
        car_request = rospy.ServiceProxy('car_request', IntersectionManager)
        resp1 = car_request(car_id, lane_id, priority, t, x, y, heading, angular_V, vel, acc, length, width, max_V, max_A, min_A, max_lateral_g) 
        print ("Request has returned ", resp1, " for car_id: ", car_id, ", lane_id: ", lane_id, ", t: ", t, ", x: ", x, ", y:", y, ", heading: ", heading, 
        	", angular_V: ", angular_V, ", vel: ", vel, ", acc: ", acc)
    	return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

def main():
	rospy.init_node('car_manager')
    rate = rospy.Rate(100.0)
    global t

    while not rospy.is_shutdown():
    	# Spawn car
    	# Send Requests
    	# Clean reserved and write to csv
    	# Update car's and increment t
    	rate.sleep()

if __name__ == '__main__':
	main()