#!/usr/bin/env python  
import rospy
#from aim.srv import *
import numpy as np
import math

# Incase we need transformations
#import tf_conversions
#import tf2_ros
#import geometry_msgs.msg

class IntersectionManager:

	def __init__(self, gsz, isz, policy=0):
		"""
		:param gsz = the size of each grid square (m)
		:param isz = the size of the entire intersection (m)
		"""
		self.__grid_size = gsz
		self.__intersection_size = isz
		self.__grid_length = int(math.ceil(self.__intersection_size/self.__grid_size))
		self.__reservations = np.zeros((1000, self.__grid_length, self.__grid_length))
		self.__policy = policy
		#self.service = rospy.Service('car_request', IntersectionManager, self.handle_car_request)

	#def handle_car_request(self, req):
		#print "Requested car's info [%s  %s  %s  %s  %s %s %s  %s %s %s]"%(req.car_id, req.lane_id, req.priority, req.t, req.x, req.y, req.heading, req.angular_V, req.vel, req.acc)  
		#successfully_scheduled = self.__schedule(req)
		#return IntersectionManagerResponse(successfully_scheduled)

	def __schedule(self, car):
		"""
		:param car = the car we will look at to see if the reservation can be accepted
		:returns: A tuple representing the trajectory. 
				  Success = true or false indicating if the reservation is accepted or not
				  xs = list of x-coordinates over the path
				  ys = list of y-coordinates over the path
				  hs = list of headings over the path
				  vs = list of velocities over the path
				  ts = list of timesteps over the path
		"""
		cur_t = car.t
		time = (cur_t * 10) % 1000
		if time == 0
			time = 1000
		T_old = time - 1
		self.reservations[T_old] = np.zeros((self.grid_length, self.grid_length))

		##########################################################################
		###
		### TODO: Check if the car the first in the lane without a reservation
		###		  Reject the request if it not first
		###
		##########################################################################

		# Check if the car is clear using the correct policy
		if self.policy == 0:
			success, xs, ys, headings, vs, ts = self.__ourPolicy(car)
		elif self.policy == 1:
			success, xs, ys, headings, vs, ts = self.__dresnerStonePolicy(car)
		elif self.policy == 2:
			success, xs, ys, headings, vs, ts = self.__trafficLightPolicy(car)
		elif self.policy == 3:
			success, xs, ys, headings, vs, ts = self.__stopSignPolicy(car)
		else:
			success = false
			xs = []
			ys = []
			headings = []
			vs = []
			ts = []
		

	def __ourPolicy(self, car):
		"""
		:param car = the car we will look at to see if the reservation can be accepted
		:returns: A tuple representing the trajectory. 
				  Success = true or false indicating if the reservation is accepted or not
				  xs = list of x-coordinates over the path
				  ys = list of y-coordinates over the path
				  hs = list of headings over the path
				  vs = list of velocities over the path
				  ts = list of timesteps over the path
		"""
		# Get the trajectory along the path if the car maintains its velocity
		xs, ys, hs, vs, ts = self.__createTrajectory(car, car.vel)
		car_w = car.width
		car_l = car.length

		# Calculate the initial bounding box
		box = np.array([[[xs[0] - car.car_w],
						 [ys[0]],
						 [1]],
						[[xs[0] + car.car_w],
						 [ys[0]],
						 [1]],
						[[xs[0] - car.car_w],
						 [ys[0] - car.car_l],
						 [1]],
						[[xs[0] + car.car_w],
						 [ys[0] - car.car_l],
						 [1]]])
		# Rotate so heading in correct direction
		T = np.array([[1, 0, -xs[0]],
					  [0, 1, -ys[0]],
					  [0, 0, 1]])
		R = np.array([[np.cos(np.radians(-hs[0])), -np.sin(np.radians(-hs[0])), 0],
					  [np.sin(np.radians(-hs[0])), np.cos(np.radians(-hs[0])), 0],
					  [0, 0, 1]])
		R = np.around(R, decimals=10)
		for i in range(4):
			box[i] = np.dot(np.dot(np.dot(np.linalg.inv(T), R) , T) ,box[i])

	def __dresnerStonePolicy(self):
		"""
		:param car = the car we will look at to see if the reservation can be accepted
		:returns: A tuple representing the trajectory. 
				  Success = true or false indicating if the reservation is accepted or not
				  xs = list of x-coordinates over the path
				  ys = list of y-coordinates over the path
				  hs = list of headings over the path
				  vs = list of velocities over the path
				  ts = list of timesteps over the path
		"""
		pass

	def __trafficLightPolicy(self):
		"""
		:param car = the car we will look at to see if the reservation can be accepted
		:returns: A tuple representing the trajectory. 
				  Success = true or false indicating if the reservation is accepted or not
				  xs = list of x-coordinates over the path
				  ys = list of y-coordinates over the path
				  hs = list of headings over the path
				  vs = list of velocities over the path
				  ts = list of timesteps over the path
		"""
		pass

	def __stopSignPolicy(self):
		"""
		:param car = the car we will look at to see if the reservation can be accepted
		:returns: A tuple representing the trajectory. 
				  Success = true or false indicating if the reservation is accepted or not
				  xs = list of x-coordinates over the path
				  ys = list of y-coordinates over the path
				  hs = list of headings over the path
				  vs = list of velocities over the path
				  ts = list of timesteps over the path
		"""
		pass

	def __createTrajectory(self, car, desired_velo):
		pass

	####################### Getters and Setters #######################
	@property
	def grid_size(self):
		return self.__grid_size
	
	@grid_size.setter
	def grid_size(self, value):
		self.__grid_size = value

	@property
	def intersection_size(self):
		return self.__intersection_size
	
	@intersection_size.setter
	def intersection_size(self, value):
		self.__intersection_size = value

	@property
	def grid_length(self):
		return self.__grid_length
	
	@grid_length.setter
	def grid_length(self, value):
		self.__grid_length = value

	@property
	def reservations(self):
		return self.__reservations
	
	@reservations.setter
	def reservations(self, value):
		self.__reservations = value

	@property
	def policy(self):
		return self.__policy

	@policy.setter
	def policy(self, value):
		self.__policy = value
	

def main():
	gsz = 1
	isz = 12
	#rospy.init_node('intersection_manager_server')
	IM = IntersectionManager(gsz, isz)
	#rospy.spin()
	print "Grid Size: %s\nIntersection Size: %s\nGrid Length: %s"%(IM.grid_size, IM.intersection_size, IM.grid_length)

if __name__ == '__main__':
	main()