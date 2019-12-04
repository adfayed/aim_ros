#!/usr/bin/env python  
import rospy
from aim.srv import *
import numpy as np
import math
# import matplotlib.pyplot as plt

# Incase we need transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg


class Car:

	def __init__(self, car_id, lane_id, t, x, y, h, v, desired_v, l, w, max_a, min_a):
		self.car_id = car_id
		self.lane_id = lane_id
		self.t = t
		self.x = x
		self.y = y
		self.heading = h
		self.vel = v
		self.desired_velo = desired_v
		self.max_v = 20
		self.width = w
		self.length = l
		self.max_A = max_a
		self.min_A = min_a


class IntersectionManager:

	def __init__(self, gsz, dMax, dMin, timestep, lane_width = 3.66, policy=0):
		"""
		:param gsz = the size of each grid square (m)
		:param isz = the size of the entire intersection (m)
		:param dMax = distance which cars are starting to request reservations (m)
		:param dMin = distance after intersection when IM stops tracking car (m)
		:param timestep = how much time passes in each timestep
		:param policy = which policy to run. 0 is our method and default
		"""
		self.__grid_size = gsz
		self.__intersection_size = 2 * dMax + 6 * lane_width
		self.__lane_width = lane_width
		self.__num_lanes = 6
		self.__grid_length = int(math.ceil(self.__intersection_size/self.__grid_size))
		self.__reservations = np.full((1000, self.__grid_length, self.__grid_length), -1)
		self.__policy = policy
		self.__dMax = dMax		# The distance from the intersection the car starts making requests
		self.__dMin = dMin		# Distance after the intersection we stop worrying about a car
		self.__timestep = timestep		# The amount of time to pass in one time step
		self.__lane_q = [[], [], [], [], [], [], [], [], [], [], [], []]		# List to indicate which cars are first in the list
		self.__can_go = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]		# indicates what time the car in that lane can go (for stop sign)
		self.__phase = 0		# Used to determine what phase the traffic light policy is in
		self.__time_to_change = 0		# Used to know when it is ok to change to a different phase in the traffic light policy
		self.__conflict = False		# Used to determine if the phase needs to change
		self.service = rospy.Service('car_request', Request, self.handle_car_request)

	def handle_car_request(self, req):
		# print "Received a request with ID: ",req.car_id, " in lane: ",req.lane_id
		# print "Requested car's info [%s  %s  %s  %s  %s %s %s  %s %s %s]"%(req.car_id, req.lane_id, req.priority, req.t, req.x, req.y, req.heading, req.angular_V, req.vel, req.acc)
		successfully_scheduled, xs, ys, hs, vs, ts = self.__schedule(req)
		# return successfully_scheduled, xs, ys, hs, vs, ts
		# print "\rReturning the request with ID: ", req.car_id, " in lane: ",req.lane_id
		# sys.stdout.flush()
		return RequestResponse(successfully_scheduled, xs, ys, hs, vs, ts)

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
		if time == 0:
			time = 1000
		T_old = int(time - 1)
		self.reservations[T_old] = np.full((self.grid_length, self.grid_length), -1)
		# if cur_t < 30:
		# 	return False, [], [], [], [], []

		lane = car.lane_id		# Get the lane of the car
		car_id = car.car_id		# Get the car's id
		# Check if that lane has a list of cars already waiting
		if len(self.lane_q[lane]) != 0:
			# Check if this car is not the first in line
			if self.lane_q[lane][0] != car_id:
				# Check if the car is not in the list
				if car_id not in self.lane_q[lane]:
					# Add the car to the list
					self.lane_q[lane].append(car_id)
				# Ignore the car's request
				return False, [], [], [], [], []

		# Check if the car is clear using the correct policy
		if self.policy == 0:
			min_v = 10.0
			# print "Car's velocity: ",car.vel
			# print "Car's x: ",car.x
			# print "Car's y: ",car.y
			# success, xs, ys, headings, vs, ts = self.__ourPolicy(car, max(min_v,car.vel))
			success, xs, ys, headings, vs, ts = self.__ourPolicy(car, max(min_v, car.desired_vel))
		elif self.policy == 1:
			success, xs, ys, headings, vs, ts = self.__dresnerStonePolicy(car)
		elif self.policy == 2:
			success, xs, ys, headings, vs, ts = self.__trafficLightPolicy(car)
		elif self.policy == 3:
			# print "Car's velocity: ",car.vel
			# print "Car's x: ",car.x
			# print "Car's y: ",car.y
			success, xs, ys, headings, vs, ts = self.__stopSignPolicy(car)
		else:
			success = False
			xs = []
			ys = []
			headings = []
			vs = []
			ts = []

		# If successfully scheduled, remove the car if its in the queue
		if success:
			if len(self.lane_q[lane]) != 0 and car_id in self.lane_q[lane]:
				self.lane_q[lane].remove(car_id)
		# If not successfully scheduled, add the car if it isn't in the queue
		if not success:
			# if len(self.lane_q[lane]) != 0 and car_id not in self.lane_q[lane]:
			if car_id not in self.lane_q[lane]:
				self.lane_q[lane].append(car_id)

		return success, xs, ys, headings, vs, ts

	def __ourPolicy(self, car, desired_velo):
		# First check the path using the desired_velo
		xs, ys, hs, vs, ts = self.__createFullTrajectory(car, desired_velo)
		collision, col_time_index, col_counter, reserved_heading, car_heading, col_temp_res, col_indices = self.__collisionDetection(xs, ys, hs, ts, car)

		# If it works return true
		if not collision:
			# Add the temp grids back to reservations
			for i in range(len(col_indices)):
				self.reservations[col_indices[i]] = col_temp_res[i]
			success = True
			return success, xs, ys, hs, vs, ts
		# else:
		# 	success = False
		# return success, xs, ys, hs, vs, ts

		# If it fails create path based from start to point of collision where car speeds up or slows down
		slow_down = True
		can_speed_up = True

		##############################################################
		###
		### TODO: Determine if the car needs to slow down or speed up
		###
		##############################################################
		if car_heading <= 180:
			offset = car_heading
			if reserved_heading == car_heading:
				from_direction = 'F'
			elif reserved_heading > offset and  reserved_heading < 180 + offset:
				from_direction = 'L'
			else:
				from_direction = 'R'
		else:
			offset = 360 - car_heading
			if reserved_heading == car_heading:
				from_direction = 'F'
			elif reserved_heading < 360 - offset and reserved_heading > 180 - offset:
				from_direction = 'R'
			else:
				from_direction = 'L'

		if from_direction == 'F':
			can_speed_up = False
		elif col_counter[3] > 0:
			slow_down = False
		elif from_direction == 'R':
			if col_counter[1] > col_counter[2]:
				slow_down = True
			elif col_counter[2] > col_counter[0]:
				slow_down = False
			else:
				slow_down = True
		else:
			if col_counter[1] > col_counter[0]:
				slow_down = True
			elif col_counter[0] > col_counter[2]:
				slow_down = False
			else:
				slow_down = True

		######################################################################
		###
		### TODO: Handle if the collision happens with a car in the same lane
		###		  * Maybe do more than just slow down like it will do now
		###
		######################################################################

		tries = 0		# Indicates which attempt this is
		if not can_speed_up:
			tries = 1
			slow_down = False
		while True:
			if tries == 1:
				slow_down = not slow_down
			# Get the new path of the car to the point of collision
			if slow_down:
				# Position and heading stay the same. We want to get to this exact position/heading at a later time
				col_x = xs[col_time_index]
				col_y = ys[col_time_index]
				col_h = hs[col_time_index]
				t = col_time_index + 1		# The index of the time array to check for collision
				# Check the position and heading at a later time until there is no collision
				while collision:
					collision, new_col_time_index, col_counter, reserved_heading, car_heading, col_temp_res, col_indices = self.__collisionDetection([col_x], [col_y], [col_h], [ts[t]], car)
					if collision:
						t += 1
				col_t = ts[t]
			else:
				# Time to check if clear
				col_t = ts[col_time_index]
				i = col_time_index + 1		# The index for the position and heading arrays to check for collisions
				while collision:
					collision, new_col_time_index, col_counter, reserved_heading, car_heading, col_temp_res, col_indices = self.__collisionDetection([xs[i]], [ys[i]], [hs[i]], [col_t], car)
					if collision:
						i += 1
				col_x = xs[i]
				col_y = ys[i]
				col_h = hs[i]
			col_xs, col_ys, col_hs, col_vs, col_ts = self.__createPartTrajectory(car, car.x, car.y, car.heading, car.vel, car.t, None, col_x, col_y, col_h, col_t)
			# if len(col_xs) == 0:
			# 	tries += 1
			# 	continue
			# Check this path
			collision, new_col_time_index, col_counter, reserved_heading, car_heading, col_temp_res, col_indices = self.__collisionDetection(col_xs, col_ys, col_hs, col_ts, car)

			# If this new path works, check path from collision point to finish point using desired velo
			if not collision:
				i = len(col_xs) - 1
				end_xs, end_ys, end_hs, end_vs, end_ts = self.__createPartTrajectory(car, col_xs[i], col_ys[i], col_hs[i], col_vs[i], col_ts[i], desired_velo)
				collision, new_col_time_index, col_counter, reserved_heading, car_heading, end_temp_res, end_indices = self.__collisionDetection(end_xs, end_ys, end_hs, end_ts, car)

				# If this end portion works, return entire path as success
				if not collision:
					# Add the temp grids back to reservations
					for i in range(len(col_indices)):
						self.reservations[col_indices[i]] = col_temp_res[i]
					for i in range(len(end_indices)):
						self.reservations[end_indices[i]] = end_temp_res[i]
					final_xs = col_xs + end_xs[1:]
					final_ys = col_ys + end_ys[1:]
					final_hs = col_hs + end_hs[1:]
					final_vs = col_vs + end_vs[1:]
					final_ts = col_ts + end_ts[1:]
					success = True
					return success, final_xs, final_ys, final_hs, final_vs, final_ts

				# If this end portion fails, try end portion using the final velocity of the portion above
				end_xs, end_ys, end_hs, end_vs, end_ts = self.__createPartTrajectory(car, col_xs[i], col_ys[i], col_hs[i], col_vs[i], col_ts[i], col_vs[i])
				collision, new_col_time_index, col_counter, reserved_heading, car_heading, end_temp_res, end_indices = self.__collisionDetection(end_xs, end_ys, end_hs, end_ts, car)

				# If this end portion works, return entire path
				if not collision:
					# Add the temp grids back to reservations
					for i in range(len(col_indices)):
						self.reservations[col_indices[i]] = col_temp_res[i]
					for i in range(len(end_indices)):
						self.reservations[end_indices[i]] = end_temp_res[i]
					final_xs = col_xs + end_xs[1:]
					final_ys = col_ys + end_ys[1:]
					final_hs = col_hs + end_hs[1:]
					final_vs = col_vs + end_vs[1:]
					final_ts = col_ts + end_ts[1:]
					success = True
					return success, final_xs, final_ys, final_hs, final_vs, final_ts

				# If fails try first portion using opposite speed up/slow down
				if tries == 0:
					tries = 1
					continue
				else:
					return False, [], [], [], [], []
			else:
				if tries == 0:
					tries = 1
					continue
				else:
					return False, [], [], [], [], []

	def __dresnerStonePolicy(self, car):
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
		min_v = 10
		# Check the car going at max velocity
		xs, ys, hs, vs, ts = self.__createFullTrajectory(car, car.max_V)
		collision, col_time_index, col_counter, reserved_heading, car_heading, col_temp_res, col_indices = self.__collisionDetection(xs, ys, hs, ts, car)
		# If there is a collision, check the car going at its current velocity. Limit it to go at least the min velocity
		if collision:
			xs, ys, hs, vs, ts = self.__createFullTrajectory(car, max(min_v, car.vel))
			collision, col_time_index, col_counter, reserved_heading, car_heading, col_temp_res, col_indices = self.__collisionDetection(xs, ys, hs, ts, car)
		if not collision:
			# Add the temp grids back to reservations
			for i in range(len(col_indices)):
				self.reservations[col_indices[i]] = col_temp_res[i]
			success = True
		else:
			success = False
		return success, xs, ys, hs, vs, ts

	def __trafficLightPolicy(self, car):
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
		success = False
		xs = []
		ys = []
		hs = []
		vs = []
		ts = []
		if self.phase == 0:
			lanes = [2, 8]
			min_green_time = 2 * max(len(self.lane_q[2]), len(self.lane_q[8]))
			max_green_time = 30
		elif self.phase == 1:
			lanes = [0, 1, 6, 7]
			min_green_time = 5 + 2 * max(len(self.lane_q[0]), len(self.lane_q[1]), len(self.lane_q[6]), len(self.lane_q[7]))
			max_green_time = 50
		elif self.phase == 2:
			lanes = [5, 11]
			min_green_time = 2 * max(len(self.lane_q[5]), len(self.lane_q[11]))
			max_green_time = 30
		elif self.phase == 3:
			lanes = [3, 4, 9, 10]
			min_green_time = 5 + max(len(self.lane_q[3]), len(self.lane_q[4]), len(self.lane_q[9]), len(self.lane_q[10]))
			max_green_time = 50
		else:
			lanes = []
			min_green_time = 0
			max_green_time = 0
		# When car requests:
		if car.lane_id in lanes:		# Car is in the lane that is green
			if not self.conflict:		# No conflict so check the request
				xs, ys, hs, vs, ts = self.__createFullTrajectory(car, car.desired_vel)
				collision, col_time_index, col_counter, reserved_heading, car_heading, col_temp_res, col_indices = self.__collisionDetection(xs, ys, hs, ts, car)
				self.time_to_change = max(self.time_to_change, self.__getExitTime(car))
				if not collision:
					exit_time = self.__getExitTime(car)
					self.time_to_change = max(self.time_to_change, exit_time)
					# Add the temp grids back to reservations
					for i in range(len(col_indices)):
						self.reservations[col_indices[i]] = col_temp_res[i]
					success = True
				else:
					success = False
				return success, xs, ys, hs, vs, ts
			else:		# Conflict so see if the car can still make it through
				exit_time = self.__getExitTime(car)
				if exit_time > self.time_to_change:		# The car cannot make it through the intersection
					return success, xs, ys, hs, vs, ts
				else:
					xs, ys, hs, vs, ts = self.__createFullTrajectory(car, car.desired_vel)
					collision, col_time_index, col_counter, reserved_heading, car_heading, col_temp_res, col_indices = self.__collisionDetection(xs, ys, hs, ts, car)
					if not collision:
						# Add the temp grids back to reservations
						for i in range(len(col_indices)):
							self.reservations[col_indices[i]] = col_temp_res[i]
						success = True
					else:
						success = False
					return success, xs, ys, hs, vs, ts
		else:		# Car is not in the lane that is green
			if car.t >= self.time_to_change:		# It is past the time to change the light
				# Change the phase to the first phase that has a car waiting/coming
				while True:
					self.phase = (self.phase + 1) % 4
					if self.phase == 0:
						lanes = [2, 8]
						min_green_time = 2 * max(len(self.lane_q[2]), len(self.lane_q[8]))
						max_green_time = 30
					elif self.phase == 1:
						lanes = [0, 1, 6, 7]
						min_green_time = 5 + 2 * max(len(self.lane_q[0]), len(self.lane_q[1]), len(self.lane_q[6]),
													 len(self.lane_q[7]))
						max_green_time = 50
					elif self.phase == 2:
						lanes = [5, 11]
						min_green_time = 2 * max(len(self.lane_q[5]), len(self.lane_q[11]))
						max_green_time = 30
					elif self.phase == 3:
						lanes = [3, 4, 9, 10]
						min_green_time = 5 + max(len(self.lane_q[3]), len(self.lane_q[4]), len(self.lane_q[9]),
												 len(self.lane_q[10]))
						max_green_time = 50
					else:
						lanes = []
						min_green_time = 0
						max_green_time = 0
					# If there are cars waiting then min_green_time will not be 0 so set to this phase
					if min_green_time != 0 or car.lane_id in lanes:
						break
				self.time_to_change = car.t + min(min_green_time, max_green_time)
				if car.lane_id in lanes:		# The car is in the lane that is green
					xs, ys, hs, vs, ts = self.__createFullTrajectory(car, car.desired_vel)
					collision, col_time_index, col_counter, reserved_heading, car_heading, col_temp_res, col_indices = self.__collisionDetection(xs, ys, hs, ts, car)
					self.conflict = False
					self.time_to_change = max(self.time_to_change, self.__getExitTime(car))
					if not collision:
						exit_time = self.__getExitTime(car)
						self.time_to_change = max(self.time_to_change, exit_time)
						# Add the temp grids back to reservations
						for i in range(len(col_indices)):
							self.reservations[col_indices[i]] = col_temp_res[i]
						success = True
					else:
						success = False
					return success, xs, ys, hs, vs, ts
				else:		# The car is not in the lane that is green so set conflict flag and reject request
					self.conflict = True
					return success, xs, ys, hs, vs, ts
			else:
				self.conflict = True
				return success, xs, ys, hs, vs, ts

	def __getExitTime(self, car):
		lane = car.lane_id % 3
		# Calculate how far the car needs to travel
		travel_distance = car.length
		big_radius = 3.5 * self.lane_width
		small_radius = 0.5 * self.lane_width
		if lane == 0:
			travel_distance += 0.5 * np.pi * small_radius
		elif lane == 1:
			travel_distance += self.num_lanes * self.lane_width
		elif lane == 2:
			travel_distance += 0.5 * np.pi * big_radius
		if car.heading == 0:
			travel_distance += self.dMax - car.y
		elif car.heading == 90:
			travel_distance += self.dMax - car.x
		elif car.heading == 180:
			travel_distance += self.dMax - (self.intersection_size - car.y)
		elif car.heading == 270:
			travel_distance += self.dMax - (self.intersection_size - car.x)

		# Calculate how long it will take to travel
		time = (travel_distance * 2) / (car.desired_vel + car.vel)
		exit_time = car.t + time
		return exit_time

	def __stopSignPolicy(self, car):
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
		xs = []
		ys = []
		hs = []
		vs = []
		ts = []
		
		# Make sure the car comes to a complete stop first
		if car.vel != 0:
			success = False
			return success, xs, ys, hs, vs, ts
		
		# Make sure the car is stopped at the intersection and not before
		if car.heading == 0:
			stop_x = car.x
			stop_y = self.dMax - car.length / 2
		elif car.heading == 90:
			stop_x = self.dMax - car.length / 2
			stop_y = car.y
		elif car.heading == 180:
			stop_x = car.x
			stop_y = self.intersection_size - self.dMax + car.length / 2
		elif car.heading == 270:
			stop_x = self.intersection_size - self.dMax + car.length / 2
			stop_y = car.y
		else:
			stop_x = -1
			stop_y = -1
		if abs(car.x - stop_x) >= stop_x * 0.01 or abs(car.y - stop_y) >= stop_y * 0.01:
		# if car.x != stop_x or car.y != stop_y:
			success = False
			return success, xs, ys, hs, vs, ts
		if self.can_go[car.lane_id] == round(car.t, 3):
			self.can_go[car.lane_id] = -1
			# Check if the car is clear
			xs, ys, hs, vs, ts = self.__createFullTrajectory(car, car.desired_vel)
			collision, col_time_index, col_counter, reserved_heading, car_heading, col_temp_res, col_indices = self.__collisionDetection(xs, ys, hs, ts, car)
			if not collision:
				# Add the temp grids back to reservations
				for i in range(len(col_indices)):
					self.reservations[col_indices[i]] = col_temp_res[i]
				success = True
			else:
				success = False
			return success, xs, ys, hs, vs, ts
		else:
			if self.can_go[car.lane_id] == -1:
				self.can_go[car.lane_id] = round(car.t + 1, 3)
			return False, xs, ys, hs, vs, ts

	def __collisionDetection(self, xs, ys, hs, ts, car):
		if len(xs) == 0:
			return True, -1, [0, 0, 0, 0], -1, car.heading, [], []
		car_w = car.width
		car_l = car.length
		dSafe = 1

		# Calculate the initial bounding box
		box = np.array([[[xs[0] - (car_w / 2) - (dSafe / 2)],
						 [ys[0] + (car_l / 2) + (dSafe / 2)],
						 [1]],
						[[xs[0] + (car_w / 2) + (dSafe / 2)],
						 [ys[0] + (car_l / 2) + (dSafe / 2)],
						 [1]],
						[[xs[0] - (car_w / 2) - (dSafe / 2)],
						 [ys[0] - (car_l / 2) - (dSafe / 2)],
						 [1]],
						[[xs[0] + (car_w / 2) + (dSafe / 2)],
						 [ys[0] - (car_l / 2) - (dSafe / 2)],
						 [1]]])
		# Rotate so heading in correct direction
		T = np.array([[1, 0, -xs[0]],
					  [0, 1, -ys[0]],
					  [0, 0, 1]])
		R = np.array([[np.cos(np.radians(-hs[0])), -np.sin(np.radians(-hs[0])), 0],
					  [np.sin(np.radians(-hs[0])), np.cos(np.radians(-hs[0])), 0],
					  [0, 0, 1]])
		R = np.around(R, decimals=10)
		for b in range(4):
			box[b] = np.dot(np.dot(np.dot(np.linalg.inv(T), R), T), box[b])

		temp_res = np.full((len(ts), self.grid_length, self.grid_length), -1)  # Hold the temp grids
		indices = [i for i in range(len(ts))]  # Hold the index of reservations the grid in temp_res came from
		collision = False
		reserved_heading = -1		# Holds the heading of the car that already has a reservation
		car_heading = car.heading

		# Check for collisions at each time
		for time in range(len(ts)):
			# Get the index of the grid in reservations
			res_index = int((ts[time] * 10) % 1000)
			temp_res[time] = self.reservations[res_index]  # Copy the grid
			indices[time] = res_index  # Save the index

			# Update the position of the bounding box
			if time > 0:
				delta_x = xs[time] - xs[time - 1]
				delta_y = ys[time] - ys[time - 1]
				delta_h = hs[time] - hs[time - 1]
				T = np.array([[1, 0, delta_x],
							  [0, 1, delta_y],
							  [0, 0, 1]])
				# Translate the box to the new position
				for b in range(4):
					box[b] = T.dot(box[b])
				# Rotate the box if needed
				if delta_h != 0:
					T = np.array([[1, 0, -xs[time]],
								  [0, 1, -ys[time]],
								  [0, 0, 1]])
					R = np.array([[np.cos(np.radians(-delta_h)), -np.sin(np.radians(-delta_h)), 0],
								  [np.sin(np.radians(-delta_h)), np.cos(np.radians(-delta_h)), 0],
								  [0, 0, 1]])
					R = np.around(R, decimals=10)
					for b in range(4):
						box[b] = np.dot(np.dot(np.dot(np.linalg.inv(T), R), T), box[b])

			mode = 0
			if box.min() < 0:
				mode = 1
			points = [0, 1, 2, 3]
			col_counter = [0, 0, 0, 0]
			c_pos = [1, 3, 0, 2]
			if hs[time] >= 0 and hs[time] < 90:
				points = [2, 0, 3, 1]
				c_pos = [0, 2, 3, 1]
			elif hs[time] >= 90 and hs[time] < 180:
				points = [3, 2, 1, 0]
				c_pos = [3, 1, 2, 0]
			elif hs[time] >= 180 and hs[time] < 270:
				points = [1, 3, 0, 2]
				c_pos = [2, 0, 1, 3]

			line = 0
			# Draw positive sloped lines left-right/bottom-top
			if mode == 0:
				# Positive sloped lines
				for j in [0, 2]:
					# Get the starting and ending x, y coords
					start_x = np.around(box[points[j]][0][0], decimals=10)
					start_y = np.around(box[points[j]][1][0], decimals=10)
					end_x = np.around(box[points[j + 1]][0][0], decimals=10)
					end_y = np.around(box[points[j + 1]][1][0], decimals=10)
					# Get the starting grid coords
					grid_x = int(math.floor(start_x / self.grid_size))
					grid_y = int(math.floor(start_y / self.grid_size))
					# Check that the grid position is valid
					if grid_x < 0 or grid_x >= self.grid_length:
						continue
					elif grid_y < 0 or grid_y >= self.grid_length:
						continue
					# Check if it is already occupied
					if self.reservations[res_index][grid_y][grid_x] != -1:
						collision = True
						col_counter[c_pos[line]] += 1
						reserved_heading = self.reservations[res_index][grid_y][grid_x]
						car_heading = hs[time]
					# Set the grid square to occupied
					temp_res[time][grid_y][grid_x] = hs[time]
					# Get the next reference point
					check_x = (grid_x + 1) * self.grid_size
					check_y = (grid_y + 1) * self.grid_size
					on_left_line = start_x == (check_x - self.grid_size)
					on_bottom_line = start_y == (check_y - self.grid_size)
					# Check if the line starts on an edge/corner of a grid square
					if on_left_line and grid_x > 0:
						# Check the grid to the left
						if self.reservations[res_index][grid_y][grid_x - 1] != -1:
							collision = True
							col_counter[c_pos[line]] += 1
							reserved_heading = self.reservations[res_index][grid_y][grid_x - 1]
							car_heading = hs[time]
						temp_res[time][grid_y][grid_x - 1] = hs[time]
					if on_bottom_line and grid_y > 0:
						# Check the grid bellow
						if self.reservations[res_index][grid_y - 1][grid_x] != -1:
							collision = True
							col_counter[c_pos[line]] += 1
							reserved_heading = self.reservations[res_index][grid_y - 1][grid_x]
							car_heading = hs[time]
						temp_res[time][grid_y - 1][grid_x] = hs[time]
					if on_left_line and on_left_line and grid_x > 0 and grid_y > 0:
						# Check the grid bellow and to the left
						if self.reservations[res_index][grid_y - 1][grid_x - 1] != -1:
							collision = True
							col_counter[c_pos[line]] += 1
							reserved_heading = self.reservations[res_index][grid_y - 1][grid_x - 1]
							car_heading = hs[time]
						temp_res[time][grid_y - 1][grid_x - 1] = hs[time]
					# Get the slope of the line
					delta_x = np.around(end_x - start_x, decimals=7)
					delta_y = np.around(end_y - start_y, decimals=7)
					if delta_x == 0:
						m = float("inf")
					else:
						m = np.around(delta_y / delta_x, decimals=10)
					while check_x <= end_x or check_y <= end_y:
						# Get the slope to the reference point
						delta_x = np.around(check_x - start_x, decimals=7)
						delta_y = np.around(check_y - start_y, decimals=7)
						if delta_x == 0:
							temp_m = float("inf")
						else:
							temp_m = np.around(delta_y / delta_x, decimals=10)
						# Update the grid coords
						if temp_m > m:  # Go right
							grid_x += 1
						elif temp_m < m:  # Go up
							grid_y += 1
							# Check the grid to the left if the line is on the grid line
							if m == float("inf") and start_x == check_x - self.grid_size and grid_x > 0:
								# Make sure we are not trying to mark a grid off the intersection
								if grid_x >= self.grid_length or grid_y >= self.grid_length:
									break
								if self.reservations[res_index][grid_y][grid_x - 1] != -1:
									collision = True
									col_counter[c_pos[line]] += 1
									reserved_heading = self.reservations[res_index][grid_y][grid_x - 1]
									car_heading = hs[time]
								temp_res[time][grid_y][grid_x - 1] = hs[time]
						else:  # Go up and right
							grid_x += 1
							grid_y += 1
							# Make sure we are not trying to mark a grid off the intersection
							if grid_x >= self.grid_length or grid_y >= self.grid_length:
								break
							if self.reservations[res_index][grid_y - 1][grid_x] != -1:
								collision = True
								col_counter[c_pos[line]] += 1
								reserved_heading = self.reservations[res_index][grid_y - 1][grid_x]
								car_heading = hs[time]
							if self.reservations[res_index][grid_y][grid_x - 1] != -1:
								collision = True
								col_counter[c_pos[line]] += 1
								reserved_heading = self.reservations[res_index][grid_y][grid_x - 1]
								car_heading = hs[time]
							temp_res[time][grid_y - 1][grid_x] = hs[time]
							temp_res[time][grid_y][grid_x - 1] = hs[time]
						# Make sure we are not trying to mark a grid off the intersection
						if grid_x >= self.grid_length or grid_y >= self.grid_length:
							break
						# Check if it is already occupied
						if self.reservations[res_index][grid_y][grid_x] != -1:
							collision = True
							col_counter[c_pos[line]] += 1
							reserved_heading = self.reservations[res_index][grid_y][grid_x]
							car_heading = hs[time]
						# Set the grid square to occupied
						temp_res[time][grid_y][grid_x] = hs[time]
						# Get the next reference point
						check_x = (grid_x + 1) * self.grid_size
						check_y = (grid_y + 1) * self.grid_size
					# if collision:
					# 	break
					line += 1
				# Stop if there was a collision
				# if collision:
				# 	break

				# Negative sloped lines
				for j in [0, 1]:
					# Get the starting and ending x, y coords
					start_x = np.around(box[points[j]][0][0], decimals=10)
					start_y = np.around(box[points[j]][1][0], decimals=10)
					end_x = np.around(box[points[j + 2]][0][0], decimals=10)
					end_y = np.around(box[points[j + 2]][1][0], decimals=10)
					# Get the starting grid coords
					grid_x = int(math.floor(start_x / self.grid_size))
					grid_y = int(math.floor(start_y / self.grid_size))
					# Check if the grid position is valid
					if grid_x < 0 or grid_x >= self.grid_length:
						continue
					if grid_y < 0 or grid_y >= self.grid_length:
						continue
					# Check if it is already occupied
					if self.reservations[res_index][grid_y][grid_x] != -1:
						collision = True
						col_counter[c_pos[line]] += 1
						reserved_heading = self.reservations[res_index][grid_y][grid_x]
						car_heading = hs[time]
					# Set the grid square to occupied
					temp_res[time][grid_y][grid_x] = hs[time]
					check_x = (grid_x + 1) * self.grid_size
					check_y = grid_y * self.grid_size
					on_left_line = start_x == (check_x - self.grid_size)
					on_bottom_line = start_y == check_y
					# Check if the line starts on an edge/corner of a grid square
					if on_left_line and grid_x > 0:
						# Check the grid to the left
						if self.reservations[res_index][grid_y][grid_x - 1] != -1:
							collision = True
							col_counter[c_pos[line]] += 1
							reserved_heading = self.reservations[res_index][grid_y][grid_x - 1]
							car_heading = hs[time]
						temp_res[time][grid_y][grid_x - 1] = hs[time]
					if on_bottom_line and grid_y > 0:
						# Check the grid below
						if self.reservations[res_index][grid_y - 1][grid_x] != -1:
							collision = True
							col_counter[c_pos[line]] += 1
							reserved_heading = self.reservations[res_index][grid_y - 1][grid_x]
							car_heading = hs[time]
						temp_res[time][grid_y - 1][grid_x] = hs[time]
					# Get the slope of the line
					delta_x = np.around(end_x - start_x, decimals=7)
					delta_y = np.around(end_y - start_y, decimals=7)
					if delta_x == 0:
						m = -float("inf")
					else:
						m = np.around(delta_y / delta_x, decimals=10)
					while check_x <= end_x or check_y >= end_y:
						# Get the slope to the reference point
						delta_x = np.around(check_x - start_x, decimals=7)
						delta_y = np.around(check_y - start_y, decimals=7)
						if delta_x == 0:
							temp_m = float("inf")
						else:
							temp_m = np.around(delta_y / delta_x, decimals=10)
						# Update grid coords
						if temp_m < m:  # Go right
							grid_x += 1
							# Check the grid square above, if the line is on the grid line
							if m == 0 and start_y == check_y + self.grid_size:
								# Make sure not to mark a grid off the intersection
								if grid_x >= self.grid_length or grid_y >= self.grid_length:
									break
								if self.reservations[res_index][grid_y + 1][grid_x] != -1:
									collision = True
									col_counter[c_pos[line]] += 1
									reserved_heading = self.reservations[res_index][grid_y + 1][grid_x]
									car_heading = hs[time]
								temp_res[time][grid_y + 1][grid_x] = hs[time]
						elif temp_m > m:  # Go down
							grid_y -= 1
						else:  # Go down and right
							grid_x += 1
							grid_y -= 1
							# Make sure we are not trying to mark a grid off the intersection
							if grid_x >= self.grid_length or grid_y < 0:
								break
							# Check the grid squares next to this one
							if self.reservations[res_index][grid_y + 1][grid_x] != -1:
								collision = True
								col_counter[c_pos[line]] += 1
								reserved_heading = self.reservations[res_index][grid_y + 1][grid_x]
								car_heading = hs[time]
							if self.reservations[res_index][grid_y][grid_x - 1] != -1:
								collision = True
								col_counter[c_pos[line]] += 1
								reserved_heading = self.reservations[res_index][grid_y][grid_x - 1]
								car_heading = hs[time]
							temp_res[time][grid_y + 1][grid_x] = hs[time]
							temp_res[time][grid_y][grid_x - 1] = hs[time]
						# Make sure we are not trying to mark a grid off the intersection
						if grid_x >= self.grid_length or grid_y < 0:
							break
						# Check if it is already occupied
						if self.reservations[res_index][grid_y][grid_x] != -1:
							collision = True
							col_counter[c_pos[line]] += 1
							reserved_heading = self.reservations[res_index][grid_y][grid_x]
							car_heading = hs[time]
						# Set the grid space to occupied
						temp_res[time][grid_y][grid_x] = hs[time]
						# Get the next reference point
						check_x = (grid_x + 1) * self.grid_size
						check_y = grid_y * self.grid_size
					# if collision:
					# 	break
					line += 1
				# Stop if there was a collision
				if collision:
					break
			# Draw positive sloped lines right-left/top-bottom
			else:
				# Vertical lines
				for j in [1, 3]:
					# Get the starting and ending x, y coords
					start_x = np.around(box[points[j]][0][0], decimals=10)
					start_y = np.around(box[points[j]][1][0], decimals=10)
					end_x = np.around(box[points[j - 1]][0][0], decimals=10)
					end_y = np.around(box[points[j - 1]][1][0], decimals=10)
					# Get the starting grid coords
					grid_x = int(math.floor(start_x / self.grid_size))
					grid_y = int(math.floor(start_y / self.grid_size))
					# Check that the grid position is valid
					if grid_x < 0 or grid_x >= self.grid_length:
						continue
					if grid_y < 0 or grid_y >= self.grid_length:
						continue
					# Check if it is already occupied
					if self.reservations[res_index][grid_y][grid_x] != -1:
						collision = True
						col_counter[c_pos[line]] += 1
						reserved_heading = self.reservations[res_index][grid_y][grid_x]
						car_heading = hs[time]
					# Set the grid square to occupied
					temp_res[time][grid_y][grid_x] = hs[time]
					# Get the reference point
					check_x = grid_x * self.grid_size
					check_y = grid_y * self.grid_size
					# Check if the line is on the edge/corner of a grid square
					if start_x == check_x and grid_x > 0:
						# Check the grid to the left
						if self.reservations[res_index][grid_y][grid_x - 1] != -1:
							collision = True
							col_counter[c_pos[line]] += 1
							reserved_heading = self.reservations[res_index][grid_y][grid_x - 1]
							car_heading = hs[time]
						temp_res[time][grid_y][grid_x - 1] = hs[time]
					while check_y >= end_y:
						grid_y -= 1
						# Make sure we are not trying to mark a grid off the intersection
						if grid_y < 0:
							break
						# Check if the line is on the edge of a grid square
						if start_x == check_x and grid_x > 0:
							# Check the grid to the left
							if self.reservations[res_index][grid_y][grid_x - 1] != -1:
								collision = True
								col_counter[c_pos[line]] += 1
								reserved_heading = self.reservations[res_index][grid_y][grid_x - 1]
								car_heading = hs[time]
							temp_res[time][grid_y][grid_x - 1] = hs[time]
						# Check if the current grid is already occupied
						if self.reservations[res_index][grid_y][grid_x] != -1:
							collision = True
							col_counter[c_pos[line]] += 1
							reserved_heading = self.reservations[res_index][grid_y][grid_x]
							car_heading = hs[time]
						# Set the grid square to occupied
						temp_res[time][grid_y][grid_x] = hs[time]
						# Get the next reference point
						check_x = grid_x * self.grid_size
						check_y = grid_y * self.grid_size
					# if collision:
					# 	break
					line += 1
				# Stop if there was a collision
				# if collision:
				# 	break

				# Horizontal lines
				for j in [2, 3]:
					# Get the starting and ending x, y coords
					start_x = np.around(box[points[j]][0][0], decimals=10)
					start_y = np.around(box[points[j]][1][0], decimals=10)
					end_x = np.around(box[points[j - 2]][0][0], decimals=10)
					end_y = np.around(box[points[j - 2]][1][0], decimals=10)
					# Get the starting grid coords
					grid_x = int(math.floor(start_x / self.grid_size))
					grid_y = int(math.floor(start_y / self.grid_size))
					# Check that the grid position is valid
					if grid_x < 0 or grid_x >= self.grid_length:
						continue
					if grid_y < 0 or grid_y >= self.grid_length:
						continue
					# Check if it is already occupied
					if self.reservations[res_index][grid_y][grid_x] != -1:
						collision = True
						col_counter[c_pos[line]] += 1
						reserved_heading = self.reservations[res_index][grid_y][grid_x]
						car_heading = hs[time]
					# Set the grid square to occupied
					temp_res[time][grid_y][grid_x] = hs[time]
					# Get the next reference point
					check_x = grid_x * self.grid_size
					check_y = grid_y * self.grid_size
					# Check if the line is on the edge of a grid square
					if start_y == check_y and grid_y > 0:
						# Check the grid below
						if self.reservations[res_index][grid_y - 1][grid_x] != -1:
							collision = True
							col_counter[c_pos[line]] += 1
							reserved_heading = self.reservations[res_index][grid_y - 1][grid_x]
							car_heading = hs[time]
						temp_res[time][grid_y - 1][grid_x] = hs[time]
					while check_x >= end_x:
						grid_x -= 1
						# Make sure we are not trying to mark a grid off the intersection
						if grid_x < 0:
							break
						# Check if the line is on the edge of a grid square
						if start_y == check_y and grid_y > 0:
							# Check the grid below
							if self.reservations[res_index][grid_y - 1][grid_x] != -1:
								collision = True
								col_counter[c_pos[line]] += 1
								reserved_heading = self.reservations[res_index][grid_y - 1][grid_x]
								car_heading = hs[time]
							temp_res[time][grid_y - 1][grid_x] = hs[time]
						# Check if it is already occupied
						if self.reservations[res_index][grid_y][grid_x] != -1:
							collision = True
							col_counter[c_pos[line]] += 1
							reserved_heading = self.reservations[res_index][grid_y][grid_x]
							car_heading = hs[time]
						# Set the grid square to occupied
						temp_res[time][grid_y][grid_x] = hs[time]
						# Get the next reference point
						check_x = grid_x * self.grid_size
						check_y = grid_y * self.grid_size
					# if collision:
					# 	break
					line += 1
				# Stop if there was a collision
				if collision:
					break
		return collision, time, col_counter, reserved_heading, car_heading, temp_res[0:time], indices[0:time]

	def __createFullTrajectory(self, car, desired_velo):
		"""
		:param car = the car we will look at to see if the reservation can be accepted
		:param desired_velo = the desired velocity for the car by the end of the path
		:returns: A tuple representing the trajectory.
				  xs = list of x-coordinates over the path
				  ys = list of y-coordinates over the path
				  hs = list of headings over the path
				  vs = list of velocities over the path
				  ts = list of timesteps over the path
		"""
		# print "\rDesired velocity = ",desired_velo,"       "
		# sys.stdout.flush()
		# Initialize our lists
		xs = [car.x]
		ys = [car.y]
		heading = car.heading
		hs = [heading]
		velo = car.vel
		vs = [velo]
		t = car.t
		ts = [t]

		# Determine what the acceleration should be for the vehicle
		delta_v = desired_velo - velo
		if delta_v >= 0:
			accel = min(car.max_A, delta_v / 5)
			speeding_up = True
		else:
			accel = max(car.min_A, delta_v / 5)
			speeding_up = False

		####################### Section 1: Straight path from dMax to intersetion ##########################
		if heading == 0:
			delta_x = 0		# Indicates how x values change
			delta_y = 1		# Indicates how y values change
			end_x = car.x		# The goal x value
			end_y = self.dMax		# The goal y value
			increasing = True		# Indicates if the value we will check is increasing
		elif heading == 90:
			delta_x = 1
			delta_y = 0
			end_x = self.dMax
			end_y = car.y
			increasing = True
		elif heading == 180:
			delta_x = 0
			delta_y = -1
			end_x = car.x
			end_y = self.intersection_size - self.dMax
			increasing = False
		elif heading == 270:
			delta_x = -1
			delta_y = 0
			end_x = self.intersection_size - self.dMax
			end_y = car.y
			increasing = False
		else:
			delta_x = 0
			delta_y = 0
			end_x = -1
			end_y = -1
			increasing = False

		new_x = car.x		# Get the current x of the car
		new_y = car.y		# Get the current y of the car
		time_left = 0		# Indicates how much time is left if the car finishes the section without using the whole timestep
		if car.x == end_x and car.y == end_y:
			time_left = self.timestep
		while time_left == 0:
			# Get the new x and y values based on the velocity and acceleration of the car
			new_x, new_y = self.__nextStraightPoint(new_x, new_y, velo, accel, delta_x, delta_y, self.timestep)
			# Check if the car has passed its goal in the x direction
			if (increasing and new_x > end_x) or (not increasing and new_x < end_x):
				dist_traveled = abs(xs[len(xs) - 1] - end_x)		# The distance from the last point to the goal
				final_v = np.sqrt(velo**2 + 2 * accel * dist_traveled)
				time_left = self.timestep - ((dist_traveled * 2) / (final_v + velo))		# How much longer to finish this timestep in the next section
				new_x = end_x		# Make sure the car is at its goal position
			# Check if the car has passed its goal in the y direction
			if (increasing and new_y > end_y) or (not increasing and new_y < end_y):
				dist_traveled = abs(ys[len(ys) - 1] - end_y)		# The distance from the last point to the goal
				final_v = np.sqrt(velo**2 + 2 * accel * dist_traveled)
				time_left = self.timestep - ((dist_traveled * 2) / (final_v + velo))
				new_y = end_y		# Make sure the car is at its goal position
			# Update the velocity of the car
			velo += accel * (self.timestep - time_left)
			# Set acceleration to 0 if the car is at its desired velocity
			if (speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo):
				accel = 0
			# We can add the new points if the car finished the timestep
			if time_left == 0:
				xs.append(new_x)
				ys.append(new_y)
				hs.append(heading)
				vs.append(velo)
				t += self.timestep
				t = np.around(t, decimals=10)
				ts.append(t)

		####################### Section 2: Intersection path (Turn Left, Straight, Turn Right ##########################
		lane = car.lane_id % 3
		if lane == 0:		# Turning right
			if heading == 0:
				center_x = self.intersection_size - self.dMax		# X value of the circle path the turn follows
				center_y = self.dMax								# Y value of the circle path the turn follows
				end_x = self.intersection_size - self.dMax			# Goal's x value
				end_y = self.dMax + (self.lane_width / 2)			# Goal's y value
				end_h = 90											# Goal's heading
			elif heading == 90:
				center_x = self.dMax
				center_y = self.dMax
				end_x = self.dMax + (self.lane_width / 2)
				end_y = self.dMax
				end_h = 180
			elif heading == 180:
				center_x = self.dMax
				center_y = self.intersection_size - self.dMax
				end_x = self.dMax
				end_y = self.intersection_size - self.dMax - (self.lane_width / 2)
				end_h = 270
			elif heading == 270:
				center_x = self.intersection_size - self.dMax
				center_y = self.intersection_size - self.dMax
				end_x = self.intersection_size - self.dMax - (self.lane_width / 2)
				end_y = self.intersection_size - self.dMax
				end_h = 360		# Use 360 instead of 0 for mathematical purposes, Updates to 0 later
			else:
				center_x = 0
				center_y = 0
				end_x = -1
				end_y = -1
				end_h = 0

			# Account for the leftover time from the last section
			new_x, new_y, new_h = self.__nextRightPoint(heading, velo, accel, time_left, center_x, center_y)
			velo += accel * time_left
			if (speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo):
				accel = 0
			xs.append(new_x)
			ys.append(new_y)
			hs.append(new_h)
			vs.append(velo)
			t += self.timestep
			t = np.around(t, decimals=10)
			ts.append(t)
			time_left = 0

			while time_left == 0:
				new_x, new_y, new_h = self.__nextRightPoint(new_h, velo, accel, self.timestep, center_x, center_y)
				# Check if the car passes its goal point
				if new_h > end_h:
					radius = self.lane_width		# Radius of circle path
					delta_h = end_h - hs[len(hs) - 1]
					dist_traveled = (delta_h / 360) * (2 * np.pi * radius)
					final_v = np.sqrt(velo**2 + 2 * accel * dist_traveled)
					time_left = self.timestep - ((dist_traveled * 2) / (final_v + velo))
					new_x = end_x
					new_y = end_y
					if end_h == 360:
						new_h = 0
					else:
						new_h = end_h
					heading = new_h
				velo += accel * (self.timestep - time_left)
				if (speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo):
					accel = 0
				if time_left == 0:
					xs.append(new_x)
					ys.append(new_y)
					hs.append(new_h)
					vs.append(velo)
					t += self.timestep
					t = np.around(t, decimals=10)
					ts.append(t)

		elif lane == 1:		# Going straight
			if heading == 0:
				delta_x = 0
				delta_y = 1
				end_x = new_x
				end_y = self.intersection_size - self.dMax
				increasing = True
			elif heading == 90:
				delta_x = 1
				delta_y = 0
				end_x = self.intersection_size - self.dMax
				end_y = new_y
				increasing = True
			elif heading == 180:
				delta_x = 0
				delta_y = -1
				end_x = new_x
				end_y = self.dMax
				increasing = False
			elif heading == 270:
				delta_x = -1
				delta_y = 0
				end_x = self.dMax
				end_y = new_y
				increasing = False
			else:
				delta_x = 0
				delta_y = 0
				end_x = -1
				end_y = -1
				increasing = False

			# Account for the leftover time from the last section
			new_x, new_y = self.__nextStraightPoint(new_x, new_y, velo, accel, delta_x, delta_y, time_left)
			velo += accel * time_left
			if (speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo):
				accel = 0
			xs.append(new_x)
			ys.append(new_y)
			hs.append(heading)
			vs.append(velo)
			t += self.timestep
			t = np.around(t, decimals=10)
			ts.append(t)
			time_left = 0

			while time_left == 0:
				new_x, new_y = self.__nextStraightPoint(new_x, new_y, velo, accel, delta_x, delta_y, self.timestep)
				if (increasing and new_x > end_x) or (not increasing and new_x < end_x):
					dist_traveled = abs(xs[len(xs) - 1] - end_x)
					final_v = np.sqrt(velo**2 + 2 * accel * dist_traveled)
					time_left = self.timestep - ((dist_traveled * 2) / (final_v + velo))
					new_x = end_x
				if (increasing and new_y > end_y) or (not increasing and new_y < end_y):
					dist_traveled = abs(ys[len(ys) - 1] - end_y)
					final_v = np.sqrt(velo**2 + 2 * accel * dist_traveled)
					time_left = self.timestep - ((dist_traveled * 2) / (final_v + velo))
					new_y = end_y
				velo += accel * (self.timestep - time_left)
				if (speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo):
					accel = 0
				if time_left == 0:
					xs.append(new_x)
					ys.append(new_y)
					hs.append(heading)
					vs.append(velo)
					t += self.timestep
					t = np.around(t, decimals=10)
					ts.append(t)

		elif lane == 2:		# Turning Left
			if heading == 0:
				center_x = self.dMax
				center_y = self.dMax
				end_x = self.dMax
				end_y = self.intersection_size - self.dMax - (self.lane_width * 2.5)
				end_h = 270
			elif heading == 90:
				center_x = self.dMax
				center_y = self.intersection_size - self.dMax
				end_x = self.intersection_size - self.dMax - (self.lane_width * 2.5)
				end_y = self.intersection_size - self.dMax
				end_h = 0
			elif heading == 180:
				center_x = self.intersection_size - self.dMax
				center_y = self.intersection_size - self.dMax
				end_x = self.intersection_size - self.dMax
				end_y = self.dMax + (self.lane_width * 2.5)
				end_h = 90
			elif heading == 270:
				center_x = self.intersection_size - self.dMax
				center_y = self.dMax
				end_x = self.dMax + (self.lane_width * 2.5)
				end_y = self.dMax
				end_h = 180
			else:
				center_x = 0
				center_y = 0
				end_x = -1
				end_y = -1
				end_h = 0

			# Account for the leftover time from the last section
			new_x, new_y, new_h = self.__nextLeftPoint(heading, velo, accel, time_left, center_x, center_y)
			velo += accel * time_left
			if (speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo):
				accel = 0
			xs.append(new_x)
			ys.append(new_y)
			hs.append(new_h)
			vs.append(velo)
			t += self.timestep
			t = np.around(t, decimals=10)
			ts.append(t)
			time_left = 0

			while time_left == 0:
				new_x, new_y, new_h = self.__nextLeftPoint(new_h, velo, accel, self.timestep, center_x, center_y)
				if new_h < end_h:
					radius = self.lane_width * 3.5
					delta_h = hs[len(hs) - 1] - end_h
					dist_traveled = (delta_h / 360) * (2 * np.pi * radius)
					final_v = np.sqrt(velo**2 + 2 * accel * dist_traveled)
					time_left = self.timestep - ((dist_traveled * 2) / (final_v + velo))
					new_x = end_x
					new_y = end_y
					new_h = end_h
					heading = new_h
				velo += accel * (self.timestep - time_left)
				if (speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo):
					accel = 0
				if time_left == 0:
					xs.append(new_x)
					ys.append(new_y)
					hs.append(new_h)
					vs.append(velo)
					t += self.timestep
					t = np.around(t, decimals=10)
					ts.append(t)

		####################### Section 3: Go straight until dMin ##########################
		if heading == 0:
			delta_x = 0
			delta_y = 1
			end_x = new_x
			end_y = self.intersection_size - self.dMax + self.dMin
			increasing = True
		elif heading == 90:
			delta_x = 1
			delta_y = 0
			end_x = self.intersection_size - self.dMax + self.dMin
			end_y = new_y
			increasing = True
		elif heading == 180:
			delta_x = 0
			delta_y = -1
			end_x = new_x
			end_y = self.dMax - self.dMin
			increasing = False
		elif heading == 270:
			delta_x = -1
			delta_y = 0
			end_x = self.dMax - self.dMin
			end_y = new_y
			increasing = False
		else:
			delta_x = 0
			delta_y = 0
			end_x = -1
			end_y = -1
			increasing = False

		# Account for the leftover time from the last section
		new_x, new_y = self.__nextStraightPoint(new_x, new_y, velo, accel, delta_x, delta_y, time_left)
		velo += accel * time_left
		if (speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo):
			accel = 0
		xs.append(new_x)
		ys.append(new_y)
		hs.append(heading)
		vs.append(velo)
		t += self.timestep
		t = np.around(t, decimals=10)
		ts.append(t)

		# Continue until the car reaches a point at the goal
		while True:
			new_x, new_y = self.__nextStraightPoint(new_x, new_y, velo, accel, delta_x, delta_y, self.timestep)
			velo += accel * self.timestep
			if (speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo):
				accel = 0
			xs.append(new_x)
			ys.append(new_y)
			hs.append(heading)
			vs.append(velo)
			t += self.timestep
			t = np.around(t, decimals=10)
			ts.append(t)
			if (increasing and new_x > end_x) or (not increasing and new_x < end_x):
				break
			if (increasing and new_y > end_y) or (not increasing and new_y < end_y):
				break

		# Return the trajectory
		return xs, ys, hs, vs, ts

	def __createPartTrajectory(self, car, start_x, start_y, start_h, start_v, start_t, desired_velo, final_x = -1, final_y = -1, final_h = -1, final_t = -1):
		# Initialize our lists
		xs = [start_x]
		ys = [start_y]
		heading = start_h
		hs = [heading]
		velo = start_v
		vs = [velo]
		t = start_t
		ts = [t]

		# Determine what the acceleration should be for the vehicle
		######################################################
		###
		### TODO: Figure out best way to calculate accel here
		###
		######################################################
		if desired_velo != None:
			delta_v = desired_velo - velo
			if delta_v >= 0:
				accel = min(car.max_A, delta_v / 5)
				speeding_up = True
			else:
				accel = max(car.min_A, delta_v / 5)
				speeding_up = False
		else:
			lane = car.lane_id % 3
			distance = 0
			if car.heading == 0:
				# Add the first straight portion if there is any
				if start_y < self.dMax:
					if final_y <= self.dMax:
						distance += abs(start_y - final_y)
					else:
						distance += abs(start_y - self.dMax)
				# Add the middle and final portion if there is any
				if lane == 0:		# Turning right
					arc_measure = abs(final_h - start_h)
					distance += (arc_measure * 2 * np.pi * (self.lane_width / 2)) / 360.0
					# Add the final straigh portion
					if final_x > self.intersection_size - self.dMax:
						if start_x > self.intersection_size - self.dMax:
							distance += abs(final_x - start_x)
						else:
							distance += abs(final_x - (self.intersection_size - self.dMax))
				elif lane == 1:		# Going straight
					if start_y >= self.dMax:
						distance += abs(final_y - start_y)
					elif final_y >= self.dMax:
						distance += abs(final_y - self.dMax)
				elif lane == 2:		# Turning left
					arc_measure = abs(final_h - start_h)
					distance += (arc_measure * 2 * np.pi * (self.lane_width * 3.5)) / 360.0
					# Add the final straigh portion
					if final_x < self.dMax:
						if start_x < self.dMax:
							distance += abs(start_x - final_x)
						else:
							distance += abs(self.dMax - final_x)
			elif car.heading == 90:
				# Add the first straight portion if there is any
				if start_x  < self.dMax:
					if final_x < self.dMax:
						distance += abs(final_x - start_x)
					else:
						distance += abs(self.dMax - start_x)
				# Add the middle and final portion if there is any
				if lane == 0:		# Turning right
					arc_measure = abs(final_h - start_h)
					distance += (arc_measure * 2 * np.pi * (self.lane_width / 2)) / 360.0
					# Add the final straigh portion
					if final_y < self.dMax:
						if start_y < self.dMax:
							distance += abs(start_y - final_y)
						else:
							distance += abs(self.dMax - final_y)
				elif lane == 1:		# Going straight
					if start_x >= self.dMax:
						distance += abs(final_x - start_x)
					elif final_x >= self.dMax:
						distance += abs(final_x - self.dMax)
				elif lane == 2:		# Turning left
					arc_measure = abs(final_h - start_h)
					distance += (arc_measure * 2 * np.pi * (self.lane_width * 3.5)) / 360.0
					# Add the final straigh portion
					if final_y > self.intersection_size - self.dMax:
						if start_y > self.intersection_size - self.dMax:
							distance += abs(final_y - start_y)
						else:
							distance += abs(final_y - (self.intersection_size - self.dMax))
			elif car.heading == 180:
				# Add the first straight portion if there is any
				if start_y > self.intersection_size - self.dMax:
					if final_y > self.intersection_size - self.dMax:
						distance += abs(start_y - final_y)
					else:
						distance += abs(start_y - self.intersection_size - self.dMax)
				# Add the middle and final portion if there is any
				if lane == 0:		# Turning right
					arc_measure = abs(final_h - start_h)
					distance += (arc_measure * 2 * np.pi * (self.lane_width / 2)) / 360.0
					# Add the final straigh portion
					if final_x < self.dMax:
						if start_x < self.dMax:
							distance += abs(start_x - final_x)
						else:
							distance += abs(self.dMax - final_x)
				elif lane == 1:		# Going straight
					if start_y <= self.intersection_size - self.dMax:
						distance += abs(start_y - final_y)
					elif final_y <= self.intersection_size - self.dMax:
						distance += abs(self.intersection_size - self.dMax - final_y)
				elif lane == 2:		# Turning left
					arc_measure = abs(final_h - start_h)
					distance += (arc_measure * 2 * np.pi * (self.lane_width * 3.5)) / 360.0
					# Add the final straigh portion
					if final_x > self.intersection_size - self.dMax:
						if start_x >= self.intersection_size - self.dMax:
							distance += abs(final_x - start_x)
						else:
							distance += abs(final_x - (self.intersection_size - self.dMax))
			elif car.heading == 270:
				# Add the first straight portion if there is any
				if start_x > self.intersection_size - self.dMax:
					if final_x > self.intersection_size - self.dMax:
						distance += abs(start_x - final_x)
					else:
						distance += abs(start_x - (self.intersection_size - self.dMax))
				# Add the middle and final portion if there is any
				if lane == 0:		# Turning right
					arc_measure = abs(final_h - start_h)
					distance += (arc_measure * 2 * np.pi * (self.lane_width / 2)) / 360.0
					# Add the final straigh portion
					if final_y > self.intersection_size - self.dMax:
						if start_y > self.intersection_size - self.dMax:
							distance += abs(final_y - start_y)
						else:
							distance += abs(final_y - self.intersection_size - self.dMax)
				elif lane == 1:		# Going straight
					if start_x <= self.intersection_size - self.dMax:
						distance += abs(start_x - final_x)
					elif final_x <= self.intersection_size - self.dMax:
						distance += abs(self.intersection_size - self.dMax - final_x)
				elif lane == 2:		# Turning left
					arc_measure = abs(final_h - start_h)
					distance += (arc_measure * 2 * np.pi * (self.lane_width * 3.5)) / 360.0
					# Add the final straigh portion
					if final_y < self.dMax:
						if start_y <= self.dMax:
							distance += abs(final_y - start_y)
						else:
							distance += abs(self.dMax - final_y)
			delta_t = round(final_t - start_t, 2)
			if delta_t == 0:
				print "final_t = ", final_t, "\tstart_t = ", start_t
				print("Lane: %d  X: %f  Y: %f  V: %f" %(car.lane_id, car.x, car.y, car.v))
				raw_input("Continue with car %d" %(car.car_id))
			# 	delta_t = 1	
			v = ((2 * distance) / delta_t) - start_v
			accel = (v - start_v) / delta_t
			# Make sure the car is not trying to travel faster than it can
			if v > car.max_V or v <= 0 or accel > car.max_A or accel < car.min_A:
				return [], [], [], [], []

		####################### Section 1: Straight path from dMax to intersetion ##########################
		if heading == 0:
			delta_x = 0		# Indicates how x values change
			delta_y = 1		# Indicates how y values change
			end_x = car.x		# The goal x value
			end_y = self.dMax		# The goal y value
			increasing = True		# Indicates if the value we will check is increasing
		elif heading == 90:
			delta_x = 1
			delta_y = 0
			end_x = self.dMax
			end_y = car.y
			increasing = True
		elif heading == 180:
			delta_x = 0
			delta_y = -1
			end_x = car.x
			end_y = self.intersection_size - self.dMax
			increasing = False
		elif heading == 270:
			delta_x = -1
			delta_y = 0
			end_x = self.intersection_size - self.dMax
			end_y = car.y
			increasing = False
		else:
			delta_x = 0
			delta_y = 0
			end_x = -1
			end_y = -1
			increasing = True

		new_x = start_x		# Get the current x of the car
		new_y = start_y		# Get the current y of the car
		time_left = 0		# Indicates how much time is left if the car finishes the section without using the whole timestep
		if (increasing and new_x > end_x) or (not increasing and new_x < end_x) or (increasing and new_y > end_y) or (not increasing and new_y < end_y):
			time_left = self.timestep
		while time_left == 0:
			# Get the new x and y values based on the velocity and acceleration of the car
			new_x, new_y = self.__nextStraightPoint(new_x, new_y, velo, accel, delta_x, delta_y, self.timestep)
			# Check if the car has passed its goal in the x direction
			if (increasing and new_x > end_x) or (not increasing and new_x < end_x):
				dist_traveled = abs(xs[len(xs) - 1] - end_x)		# The distance from the last point to the goal
				final_v = np.sqrt(velo**2 + 2 * accel * dist_traveled)
				time_left = self.timestep - ((dist_traveled * 2) / (final_v + velo))		# How much longer to finish this timestep in the next section
				new_x = end_x		# Make sure the car is at its goal position
			# Check if the car has passed its goal in the y direction
			if (increasing and new_y > end_y) or (not increasing and new_y < end_y):
				dist_traveled = abs(ys[len(ys) - 1] - end_y)		# The distance from the last point to the goal
				final_v = np.sqrt(velo**2 + 2 * accel * dist_traveled)
				time_left = self.timestep - ((dist_traveled * 2) / (final_v + velo))
				new_y = end_y		# Make sure the car is at its goal position
			# Update the velocity of the car
			velo += accel * (self.timestep - time_left)
			# Set acceleration to 0 if the car is at its desired velocity
			if desired_velo != None and ((speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo)):
				accel = 0
			# We can add the new points if the car finished the timestep
			if time_left == 0:
				xs.append(new_x)
				ys.append(new_y)
				hs.append(heading)
				vs.append(velo)
				t += self.timestep
				t = np.around(t, decimals=10)
				ts.append(t)

			# Check if the car has reached the final point of the trajectory ..... Do we need to check if there is still time remaining?
			if delta_x != 0 and final_x != -1 and ((increasing and new_x >= final_x) or (not increasing and new_x <= final_x)):
				return xs, ys, hs, vs, ts
			if delta_y != 0 and final_y != -1 and ((increasing and new_y >= final_y) or (not increasing and new_y <= final_y)):
				return xs, ys, hs, vs, ts

		####################### Section 2: Intersection path (Turn Left, Straight, Turn Right ##########################
		lane = car.lane_id % 3
		if lane == 0 and start_h != final_h:		# Turning right
			if heading >= 0 and heading < 90:
				center_x = self.intersection_size - self.dMax		# X value of the circle path the turn follows
				center_y = self.dMax								# Y value of the circle path the turn follows
				end_x = self.intersection_size - self.dMax			# Goal's x value
				end_y = self.dMax + (self.lane_width / 2)			# Goal's y value
				end_h = 90											# Goal's heading
			elif heading >= 90 and heading < 180:
				center_x = self.dMax
				center_y = self.dMax
				end_x = self.dMax + (self.lane_width / 2)
				end_y = self.dMax
				end_h = 180
			elif heading >= 180 and heading < 270:
				center_x = self.dMax
				center_y = self.intersection_size - self.dMax
				end_x = self.dMax
				end_y = self.intersection_size - self.dMax - (self.lane_width / 2)
				end_h = 270
			elif heading >= 270 and heading < 360:
				center_x = self.intersection_size - self.dMax
				center_y = self.intersection_size - self.dMax
				end_x = self.intersection_size - self.dMax - (self.lane_width / 2)
				end_y = self.intersection_size - self.dMax
				end_h = 360		# Use 360 instead of 0 for mathematical purposes, Updates to 0 later
			else:
				center_x = 0
				center_y = 0
				end_x = -1
				end_y = -1
				end_h = 0

			####################################################
			###
			### TODO: Fix this leftover so it doesn't overshoot
			###
			####################################################

			# Account for the leftover time from the last section
			new_x, new_y, new_h = self.__nextRightPoint(heading, velo, accel, time_left, center_x, center_y)
			velo += accel * time_left
			if desired_velo != None and ((speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo)):
				accel = 0
			xs.append(new_x)
			ys.append(new_y)
			hs.append(new_h)
			vs.append(velo)
			t += self.timestep
			t = np.around(t, decimals=10)
			ts.append(t)
			time_left = 0

			# Return if reached final position
			if final_h == 0:
				temp_heading = 360
			else:
				temp_heading = final_h
			if final_h != -1 and (new_h >= temp_heading or (final_h == 0 and new_h == final_h)):
				return xs, ys, hs, vs, ts

			while time_left == 0:
				new_x, new_y, new_h = self.__nextRightPoint(new_h, velo, accel, self.timestep, center_x, center_y)
				# Check if the car passes its goal point
				if new_h > end_h:
					radius = self.lane_width		# Radius of circle path
					delta_h = end_h - hs[len(hs) - 1]
					dist_traveled = (delta_h / 360) * (2 * np.pi * radius)
					final_v = np.sqrt(velo**2 + 2 * accel * dist_traveled)
					time_left = self.timestep - ((dist_traveled * 2) / (final_v + velo))
					new_x = end_x
					new_y = end_y
					if end_h == 360:
						new_h = 0
					else:
						new_h = end_h
					heading = new_h
				velo += accel * (self.timestep - time_left)
				if desired_velo != None and ((speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo)):
					accel = 0
				if time_left == 0:
					xs.append(new_x)
					ys.append(new_y)
					hs.append(new_h)
					vs.append(velo)
					t += self.timestep
					t = np.around(t, decimals=10)
					ts.append(t)

				# Return if reached final position
				if final_h != -1 and (new_h >= temp_heading or (final_h == 0 and new_h == final_h)):
					return xs, ys, hs, vs, ts

		elif lane == 1:		# Going straight
			if heading == 0:
				delta_x = 0
				delta_y = 1
				end_x = new_x
				end_y = self.intersection_size - self.dMax
				increasing = True
			elif heading == 90:
				delta_x = 1
				delta_y = 0
				end_x = self.intersection_size - self.dMax
				end_y = new_y
				increasing = True
			elif heading == 180:
				delta_x = 0
				delta_y = -1
				end_x = new_x
				end_y = self.dMax
				increasing = False
			elif heading == 270:
				delta_x = -1
				delta_y = 0
				end_x = self.dMax
				end_y = new_y
				increasing = False
			else:
				delta_x = 0
				delta_y = 0
				end_x = -1
				end_y = -1
				increasing = False

			if (increasing and start_x > end_x) or (not increasing and start_x < end_x) or (increasing and start_y > end_y) or (not increasing and start_y < end_y):
				time_left = self.timestep
			else:
				# Account for the leftover time from the last section
				new_x, new_y = self.__nextStraightPoint(new_x, new_y, velo, accel, delta_x, delta_y, time_left)
				velo += accel * time_left
				if desired_velo != None and ((speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo)):
					accel = 0
				xs.append(new_x)
				ys.append(new_y)
				hs.append(heading)
				vs.append(velo)
				t += self.timestep
				t = np.around(t, decimals=10)
				ts.append(t)
				time_left = 0

			# Check if the car has reached the final point of the trajectory
			if delta_x != 0 and final_x != -1 and ((increasing and new_x >= final_x) or (not increasing and new_x <= final_x)):
				return xs, ys, hs, vs, ts
			if delta_y != 0 and final_y != -1 and ((increasing and new_y >= final_y) or (not increasing and new_y <= final_y)):
				return xs, ys, hs, vs, ts

			while time_left == 0:
				new_x, new_y = self.__nextStraightPoint(new_x, new_y, velo, accel, delta_x, delta_y, self.timestep)
				if (increasing and new_x > end_x) or (not increasing and new_x < end_x):
					dist_traveled = abs(xs[len(xs) - 1] - end_x)
					final_v = np.sqrt(velo**2 + 2 * accel * dist_traveled)
					time_left = self.timestep - ((dist_traveled * 2) / (final_v + velo))
					new_x = end_x
				if (increasing and new_y > end_y) or (not increasing and new_y < end_y):
					dist_traveled = abs(ys[len(ys) - 1] - end_y)
					final_v = np.sqrt(velo**2 + 2 * accel * dist_traveled)
					time_left = self.timestep - ((dist_traveled * 2) / (final_v + velo))
					new_y = end_y
				velo += accel * (self.timestep - time_left)
				if desired_velo != None and ((speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo)):
					accel = 0
				if time_left == 0:
					xs.append(new_x)
					ys.append(new_y)
					hs.append(heading)
					vs.append(velo)
					t += self.timestep
					t = np.around(t, decimals=10)
					ts.append(t)

				# Check if the car has reached the final point of the trajectory ..... Do we need to check if there is still time remaining?
				if delta_x != 0 and final_x != -1 and ((increasing and new_x >= final_x) or (not increasing and new_x <= final_x)):
					return xs, ys, hs, vs, ts
				if delta_y != 0 and final_y != -1 and ((increasing and new_y >= final_y) or (not increasing and new_y <= final_y)):
					return xs, ys, hs, vs, ts

		elif lane == 2 and start_h != final_h:		# Turning Left
			if heading == 0 or (heading > 270 and heading < 360):
				center_x = self.dMax
				center_y = self.dMax
				end_x = self.dMax
				end_y = self.intersection_size - self.dMax - (self.lane_width * 2.5)
				end_h = 270
			elif heading <= 90 and heading > 0:
				center_x = self.dMax
				center_y = self.intersection_size - self.dMax
				end_x = self.intersection_size - self.dMax - (self.lane_width * 2.5)
				end_y = self.intersection_size - self.dMax
				end_h = 0
			elif heading <= 180 and heading > 90:
				center_x = self.intersection_size - self.dMax
				center_y = self.intersection_size - self.dMax
				end_x = self.intersection_size - self.dMax
				end_y = self.dMax + (self.lane_width * 2.5)
				end_h = 90
			elif heading <= 270 and heading > 180:
				center_x = self.intersection_size - self.dMax
				center_y = self.dMax
				end_x = self.dMax + (self.lane_width * 2.5)
				end_y = self.dMax
				end_h = 180
			else:
				center_x = 0
				center_y = 0
				end_x = -1
				end_y = -1
				end_h = 0

			# Account for the leftover time from the last section
			new_x, new_y, new_h = self.__nextLeftPoint(heading, velo, accel, time_left, center_x, center_y)
			velo += accel * time_left
			if desired_velo != None and ((speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo)):
				accel = 0
			xs.append(new_x)
			ys.append(new_y)
			hs.append(new_h)
			vs.append(velo)
			t += self.timestep
			t = np.around(t, decimals=10)
			ts.append(t)
			time_left = 0

			# Return if reached final position
			if final_h != -1 and new_h <= final_h:
				return xs, ys, hs, vs, ts

			while time_left == 0:
				new_x, new_y, new_h = self.__nextLeftPoint(new_h, velo, accel, self.timestep, center_x, center_y)
				if new_h < end_h:
					radius = self.lane_width * 3.5
					delta_h = hs[len(hs) - 1] - end_h
					dist_traveled = (delta_h / 360) * (2 * np.pi * radius)
					final_v = np.sqrt(velo**2 + 2 * accel * dist_traveled)
					time_left = self.timestep - ((dist_traveled * 2) / (final_v + velo))
					new_x = end_x
					new_y = end_y
					new_h = end_h
					heading = new_h
				velo += accel * (self.timestep - time_left)
				if desired_velo != None and ((speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo)):
					accel = 0
				if time_left == 0:
					xs.append(new_x)
					ys.append(new_y)
					hs.append(new_h)
					vs.append(velo)
					t += self.timestep
					t = np.around(t, decimals=10)
					ts.append(t)

				# Return if reached final position
				if final_h != -1 and new_h <= final_h:
					return xs, ys, hs, vs, ts

		####################### Section 3: Go straight until dMin ##########################
		if heading == 0:
			delta_x = 0
			delta_y = 1
			end_x = new_x
			end_y = self.intersection_size - self.dMax + self.dMin
			increasing = True
		elif heading == 90:
			delta_x = 1
			delta_y = 0
			end_x = self.intersection_size - self.dMax + self.dMin
			end_y = new_y
			increasing = True
		elif heading == 180:
			delta_x = 0
			delta_y = -1
			end_x = new_x
			end_y = self.dMax - self.dMin
			increasing = False
		elif heading == 270:
			delta_x = -1
			delta_y = 0
			end_x = self.dMax - self.dMin
			end_y = new_y
			increasing = False
		else:
			delta_x = 0
			delta_y = 0
			end_x = -1
			end_y = -1
			increasing = False

		# Account for the leftover time from the last section
		new_x, new_y = self.__nextStraightPoint(new_x, new_y, velo, accel, delta_x, delta_y, time_left)
		velo += accel * time_left
		if desired_velo != None and ((speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo)):
			accel = 0
		xs.append(new_x)
		ys.append(new_y)
		hs.append(heading)
		vs.append(velo)
		t += self.timestep
		t = np.around(t, decimals=10)
		ts.append(t)

		# Check if the car has reached the final point of the trajectory
		if delta_x != 0 and final_x != -1 and ((increasing and new_x >= final_x) or (not increasing and new_x <= final_x)):
			return xs, ys, hs, vs, ts
		if delta_y != 0 and final_y != -1 and ((increasing and new_y >= final_y) or (not increasing and new_y <= final_y)):
			return xs, ys, hs, vs, ts

		# Continue until the car reaches a point at the goal
		while True:
			new_x, new_y = self.__nextStraightPoint(new_x, new_y, velo, accel, delta_x, delta_y, self.timestep)
			velo += accel * self.timestep
			if desired_velo != None and ((speeding_up and velo >= desired_velo) or (not speeding_up and velo <= desired_velo)):
				accel = 0
			xs.append(new_x)
			ys.append(new_y)
			hs.append(heading)
			vs.append(velo)
			t += self.timestep
			t = np.around(t, decimals=10)
			ts.append(t)
			if (increasing and new_x > end_x) or (not increasing and new_x < end_x):
				break
			if (increasing and new_y > end_y) or (not increasing and new_y < end_y):
				break
			# Check if the car has reached the final point of the trajectory
			if delta_x != 0 and final_x != -1 and ((increasing and new_x >= final_x) or (not increasing and new_x <= final_x)):
				return xs, ys, hs, vs, ts
			if delta_y != 0 and final_y != -1 and ((increasing and new_y >= final_y) or (not increasing and new_y <= final_y)):
				return xs, ys, hs, vs, ts

		# Return the trajectory
		return xs, ys, hs, vs, ts

	def __nextStraightPoint(self, cur_x, cur_y, cur_v, accel, delta_x, delta_y, timestep):
		distance = (0.5 * accel * (timestep**2) + (cur_v * timestep))
		new_x = distance * delta_x + cur_x
		new_y = distance * delta_y + cur_y
		return new_x, new_y

	def __nextRightPoint(self, cur_h, cur_v, accel, timestep, center_x, center_y):
		distance = (0.5 * accel * (timestep**2) + (cur_v * timestep))		# Arc length
		radius = self.lane_width / 2
		arc_measure = (distance * 360) / (2 * np.pi * radius)
		angle = 180 - cur_h - arc_measure
		new_x = radius * np.cos(np.deg2rad(angle)) + center_x
		new_y = radius * np.sin(np.deg2rad(angle)) + center_y
		new_h = cur_h + arc_measure
		return new_x, new_y, new_h

	def __nextLeftPoint(self, cur_h, cur_v, accel, timestep, center_x, center_y):
		if cur_h == 0:
			cur_h = 360
		distance = (0.5 * accel * (timestep ** 2) + (cur_v * timestep))  # Arc length
		radius = self.lane_width * 3.5
		arc_measure = (distance * 360) / (2 * np.pi * radius)
		angle = 360 - cur_h + arc_measure
		new_x = radius * np.cos(np.deg2rad(angle)) + center_x
		new_y = radius * np.sin(np.deg2rad(angle)) + center_y
		new_h = cur_h - arc_measure
		return new_x, new_y, new_h

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
		return int(self.__grid_length)
	
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

	@property
	def dMax(self):
		return self.__dMax

	@dMax.setter
	def dMax(self, value):
		self.__dMax = value

	@property
	def dMin(self):
		return self.__dMin

	@dMin.setter
	def dMin(self, value):
		self.__dMin = value

	@property
	def timestep(self):
		return self.__timestep

	@timestep.setter
	def timestep(self, value):
		self.__timestep = value

	@property
	def lane_width(self):
		return self.__lane_width

	@lane_width.setter
	def lane_width(self, value):
		self.__lane_width = value

	@property
	def num_lanes(self):
		return self.__num_lanes

	@num_lanes.setter
	def num_lanes(self, value):
		self.__num_lanes = value

	@property
	def lane_q(self):
		return self.__lane_q

	@lane_q.setter
	def lane_q(self, value):
		self.__lane_q = value

	@property
	def phase(self):
		return self.__phase
	
	@phase.setter
	def phase(self, value):
		self.__phase = value

	@property
	def time_to_change(self):
		return self.__time_to_change

	@time_to_change.setter
	def time_to_change(self, value):
		self.__time_to_change = value

	@property
	def conflict(self):
		return self.__conflict

	@conflict.setter
	def conflict(self, value):
		self.__conflict = value

	@property
	def can_go(self):
		return self.__can_go
	
	@can_go.setter
	def can_go(self, value):
		self.__can_go = value
	

def main():
	rospy.init_node('intersection_manager_server')
	# rate = rospy.Rate(100.0)
	gsz = 0.75
	lane_width = 3.66
	dMax = 148
	dMin = 50
	timestep = 0.1
	policy = 0
	IM = IntersectionManager(gsz, dMax, dMin, timestep, lane_width, policy)
	rospy.spin()
	# while not rospy.is_shutdown():
 		# rate.sleep()


if __name__ == '__main__':
	main()
