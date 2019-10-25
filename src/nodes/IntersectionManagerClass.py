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
		for b in box:
			b = np.dot(np.dot(np.dot(np.linalg.inv(T), R) , T) ,b)

		temp_res = np.zeros((len(ts), self.grid_length, self.grid_length))	# Hold the temp grids
		indices = [i for i in range(len(ts))]	# Hold the index of reservations the grid in temp_res came from
		collision = False

		# Check for collisions at each time
		for time in range(len(ts)):
			# Get the index of the grid in reservations
			res_index = (ts[time] * 10) % 1000
			if res_index == 0:
				res_index = 1000
			temp_res[time] = self.reservations[res_index]	# Copy the grid
			indices[time] = res_index	# Save the index

			# Update the position of the bounding box
			if time > 0:
				delta_x = xs[time] - xs[time - 1]
				delta_y = ys[time] - ys[time - 1]
				delta_h = hs[time] - hs[time - 1]
				T = np.array([[1, 0, delta_x],
							  [0, 1, delta_y],
							  [0, 0, 1]])
				# Translate the box to the new position
				for b in box:
					b = T.dot(b)
				# Rotate the box if needed
				if delta_h != 0:
					T = np.array([[1, 0, -xs[time]],
								  [0, 1, -ys[time]],
								  [0, 0, 1]])
					R = np.array([[np.cos(np.radians(-delta_h)), -np.sin(np.radians(-delta_h)), 0],
								  [np.sin(np.radians(-delta_h)), np.cos(np.radians(-delta_h)), 0],
								  [0, 0, 1]])
					R = np.around(R, decimals=10)
					for b in box:
						b = np.dot(np.dot(np.dot(np.linalg.inv(T), R) , T) ,b)

			mode = 0
			if box.min() < 0:
				mode = 1
			points = [0, 1, 2, 3]
			if hs[time] >= 0 and hs[time] < 90:
				points = [2, 0, 3, 1]
			elif hs[time] >= 90 and hs[time] < 180:
				points = [3, 2, 1, 0]
			elif hs[time] >= 180 and hs[time] < 270:
				points = [1, 3, 0, 2]

			# Draw positive sloped lines left-right/bottom-top
			if mode == 0:
				# Positive sloped lines
				for j in [0, 2]:
					# Get the starting and ending x, y coords
					start_x = np.around(box[points[j]][0][0], decimals=10)
					start_y = np.around(box[points[j]][1][0], decimals=10)
					end_x = np.around(box[points[j+1]][0][0], decimals=10)
					end_y = np.around(box[points[j+1]][1][0], decimals=10)
					# Get the starting grid coords
					grid_x = math.floor(start_x/self.grid_size)
					grid_y = math.floor(start_y/self.grid_size)
					# Check that the grid position is valid
					if grid_x < 0 or grid_x >= self.grid_length:
						continue
					elif grid_y < 0 or grid_y >= self.grid_length:
						continue
					# Check if it is already occupied
					if self.reservations[res_index][grid_y][grid_x] == 1:
						collision = True
						break
					# Set the grid square to occupied
					temp_res[time][grid_y][grid_x] = 1
					# Get the next reference point
					check_x = (grid_x + 1) * self.grid_size
					check_y = (grid_y + 1) * self.grid_size
					on_left_line = start_x == (check_x - self.grid_size)
					on_bottom_line = start_y == (check_y - self.grid_size)
					# Check if the line starts on an edge/corner of a grid square
					if on_left_line and grid_x > 0:
						# Check the grid to the left
						if self.reservations[res_index][grid_y][grid_x - 1] == 1:
							collision = True
							break
						temp_res[time][grid_y][grid_x - 1] = 1
					if on_bottom_line and grid > 0:
						# Check the grid bellow
						if self.reservations[res_index][grid_y - 1][grid_x] == 1:
							collision = True
							break
						temp_res[time][grid_y - 1][grid_x] = 1
					if on_left_line and on_left_line and grid_x > 0 and grid_y > 0:
						# Check the grid bellow and to the left
						if self.reservations[res_index][grid_y - 1][grid_x - 1] == 1:
							collision = True
							break
						temp_res[time][grid_y - 1][grid_x - 1] = 1
					# Get the slope of the line
					delta_x = end_x - start_x
					delta_y = end_y - start_y
					if delta_x == 0:
						m = float("inf")
					else:
						m = np.around(delta_y / delta_x, decimals=10)
					while check_x <= end_x or check_y <= end_y:
						# Get the slope to the reference point
						delta_x = check_x - start_x
						delta_y = check_y - start_y
						if delta_x ==  0:
							temp_m = float("inf")
						else:
							temp_m = np.around(delta_y / delta_x, decimals=10)
						# Update the grid coords
						if temp_m > m:	# Go right
							grid_x += 1
						elif temp_m < m:	# Go up
							grid_y += 1
							# Check the grid to the left if the line is on the grid line
							if m == float("inf") and start_x == check_x - self.grid_size and grid_x > 0:
								# Make sure we are not trying to mark a grid off the intersection
								if grid_x >= self.grid_length or grid_y >= self.grid_length:
									break
								if self.reservations[res_index][grid_y][grid_x - 1] == 1:
									collision = True
									break
								temp_res[time][grid_y][grid_x - 1] = 1
						else:	# Go up and right
							grid_x += 1
							grid_y += 1
							# Make sure we are not trying to mark a grid off the intersection
							if grid_x >= self.grid_length or grid_y >= self.grid_length:
								break
							if self.reservations[res_index][grid_y - 1][grid_x] == 1:
								collision = True
								break
							if self.reservations[res_index][grid_y][grid_x - 1] == 1:
								collision = True
								break
							temp_res[time][grid_y - 1][grid_x] = 1
							temp_res[time][grid_y][grid_x - 1] = 1
						# Make sure we are not trying to mark a grid off the intersection
						if grid_x >= self.grid_length or grid_y >= self.grid_length:
							break
						# Check if it is already occupied
						if self.reservations[res_index][grid_y][grid_x] == 1:
							collision = True
							break
						# Set the grid square to occupied
						temp_res[time][grid_y][grid_x] = 1
						# Get the next reference point
						check_x = (grid_x + 1) * self.grid_size
						check_y = (grid_y + 1) * self.grid_size
					if collision:
						break
				# Stop if there was a collision
				if collision:
					break

				# Negative sloped lines
				for j in [0, 1]:
					# Get the starting and ending x, y coords
					start_x = np.around(box[points[j]][0][0], decimals=10)
					start_y = np.around(box[points[j]][1][0], decimals=10)
					end_x = np.around(box[points[j+2]][0][0], decimals=10)
					end_y = np.around(box[points[j+2]][1][0], decimals=10)
					# Get the starting grid coords
					grid_x = math.floor(start_x / self.grid_size)
					grid_y = math.floor(start_y / self.grid_size)
					# Check if the grid position is valid
					if grid_x < 0 or grid_x >= self.grid_length:
						continue
					if grid_y < 0 or grid_y >= self.grid_length:
						continue
					# Check if it is already occupied
					if self.reservations[res_index][grid_y][grid_x] == 1:
						collision = True
						break
					# Set the grid square to occupied
					temp_res[time][grid_y][grid_x] = 1
					check_x = (grid_x + 1) * self.grid_size
					check_y = grid_y * self.grid_size
					on_left_line = start_x == (check_x - self.grid_size)
					on_bottom_line = start_y == check_y
					# Check if the line starts on an edge/corner of a grid square
					if on_left_line and grid_x > 0:
						# Check the grid to the left
						if self.reservations[res_index][grid_y][grid_x - 1] == 1:
							collision = True
							break
						temp_res[time][grid_y][grid_x - 1] = 1
					if on_bottom_line and grid_y > 0:
						# Check the grid below
						if self.reservations[res_index][grid_y - 1][grid_x] == 1:
							collision = True
							break
						temp_res[time][grid_y - 1][grid_x] = 1
					# Get the slope of the line
					delta_x = end_x - start_x
					delta_y = end_y - start_y
					if delta_x == 0:
						m = -float("inf")
					else:
						m = np.around(delta_y / delta_x, decimals=10)
					while check_x <= end_x or check_y >= end_y:
						# Get the slope to the reference point
						delta_x = check_x - start_x
						delta_y = check_y - start_y
						if delta_x == 0:
							temp_m = float("inf")
						else:
							temp_m = np.around(delta_y / delta_x, decimals=10)
						# Update grid coords
						if temp_m < m:	# Go right
							grid_x += 1
							# Check the grid square above, if the line is on the grid line
							if m == 0 and start_y == check_y + self.grid_size:
								# Make sure not to mark a grid off the intersection
								if grid_x >= self.grid_length or grid_y >= self.grid_length:
									break
								if self.reservations[res_index][grid_y + 1][grid_x] == 1:
									collision = True
									break
								temp_res[time][grid_y + 1][grid_x] = 1
						elif temp_m > m:	# Go down
							grid_y -= 1
						else:	# Go down and right
							grid_x += 1
							grid_y -= 1
							# Make sure we are not trying to mark a grid off the intersection
							if grid_x >= self.grid_length or grid_y < 0:
								break
							# Check the grid squares next to this one
							if self.reservations[res_index][grid_y + 1][grid_x] == 1:
								collision = True
								break
							if self.reservations[res_index][grid_y][grid_x - 1] == 1:
								collision = True
								break
							temp_res[time][grid_y + 1][grid_x] = 1
							temp_res[time][grid_y][grid_x - 1] = 1
						# Make sure we are not trying to mark a grid off the intersection
						if grid_x >= self.grid_length or grid_y < 0:
							break
						# Check if it is already occupied
						if self.reservations[res_index][grid_y][grid_x] == 1:
							collision = True
							break
						# Set the grid space to occupied
						temp_res[time][grid_y][grid_x] = 1
						# Get the next reference point
						check_x = (grid_x + 1) * self.grid_size
						check_y = grid_y * self.grid_size
					if collision:
						break
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
					grid_x = math.floor(start_x / self.grid_size)
					grid_y = math.floor(start_y / self.grid_size)
					# Check that ht egrid position is valid
					if grid_x < 0 or grid_x >= self.grid_length:
						continue
					if grid_y < 0 or grid_y >= self.grid_length:
						continue
					# Check if it is already occupied
					if self.reservations[res_index][grid_y][grid_x] == 1:
						collision = True
						break
					# Set the grid square to occupied
					temp_res[time][grid_y][grid_x] = 1
					# Get the reference point
					check_x = grid_x * self.grid_size
					check_y = grid_y * self.grid_size
					# Check if the line is on the edge/corner of a grid square
					if start_x == check_x and grid > 0:
						# Check the grid to the left
						if self.reservations[res_index][grid_y][grid_x - 1] == 1:
							collision = True
							break
						temp_res[time][grid_y][grid_x - 1] = 1
					while check_y >= end_y:
						grid_y -= 1
						# Make sure we are not trying to mark a grid off the intersection
						if grid_y < 0:
							break
						# Check if the line is on the edge of a grid square
						if start_x == check_x and grid_x > 0:
							# Check the grid to the left
							if self.reservations[res_index][grid_y][grid_x - 1] == 1:
								collision = True
								break
							temp_res[time][grid_y][grid_x - 1] = 1
						# Check if the current grid is already occupied
						if self.reservations[res_index][grid_y][grid_x] == 1:
							collision = True
							break
						# Set the grid square to occupied
						temp_res[time][grid_y][grid_x] = 1
						# Get the next reference point
						check_x = grid_x * self.grid_size
						check_y = grid_y * self.grid_size
					if collision:
						break
				# Stop if there was a collision
				if collision:
					break

				# Horizontal lines
				for j in [2, 3]:
					# Get the starting and ending x, y coords
					start_x = np.around(box[points[j]][0][0], decimals=10)
					start_y = np.around(box[points[j]][1][0], decimals=10)
					end_x = np.around(box[points[j - 2]][0][0], decimals=10)
					end_y = np.around(box[points[j - 2]][1][0], decimals=10)
					# Get the starting grid coords
					grid_x = math.floor(start_x / self.grid_size)
					grid_y = math.floor(start_y / self.grid_size)
					# Check that the grid position is valid
					if grid_x < 0 or grid_x >= self.grid_length:
						continue
					if grid_y < 0 or grid_y >= self.grid_length:
						continue
					# Check if it is already occupied
					if self.reservations[res_index][grid_y][grid_x] == 1:
						collision = True
						break
					# Set the grid square to occupied
					temp_res[time][grid_y][grid_x] = 1
					# Get the next reference point
					check_x = grid_x * self.grid_size
					check_y = grid_y * self.grid_size
					# Check if the line is on the edge of a grid square
					if start_y == check_y and grid_y > 0:
						# Check to the left
						if self.reservations[res_index][grid_y - 1][grid_x] == 1:
							collision = True
							break
						temp_res[time][grid_y][grid_x] = 1
					while check_x >= end_x:
						grid_x -= 1
						# Make sure we are not trying to mark a grid off the intersection
						if grid_x < 0:
							break
						# Check if the line is on the edge of a grid square
						if start_y == check_y and grid_y > 0:
							# Check the grid below
							if self.reservations[res_index][grid_y - 1][grid_x] == 1:
								collision = True
								break
							temp_res[time][grid_y][grid_x] = 1
						# Check if it is already occupied
						if self.reservations[res_index][grid_y][grid_x] == 1:
							collision = True
							break
						# Set the grid square to occupied
						temp_res[time][grid_y][grid_x] = 1
						# Get the next reference point
						check_x = grid_x * self.grid_size
						check_y = grid_y * self.grid_size
					if collision:
						break
				# Stop if there was a collision
				if collision:
					break

		# There was no collision
		if not collision:
			# Add the temp grids back to reservations
			for i in range(len(indices)):
				self.reservations[indices[i]] = temp_res[i]
			success = True
		# Collision occured
		else:
			success = False

		return success, xs, ys, hs, vs, ts

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