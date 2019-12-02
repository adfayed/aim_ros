#!/usr/bin/env python 
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from aim.srv import *
import numpy as np
import math
import pdb
import tf
import time
import visualizeSim
import warnings
import traceback, sys, code

# Class for each car's attr
class car:
	def __init__(self, car_id, lane_id, t, x, y, heading, angular_V, vel, acc, following = False, reservation = False, priority = 0,
		length = 4.9784, width = 1.96342, max_V = 69.29, max_A = 43.47826087, min_A = -43.47826087, 
		max_lateral_g = 1.2):
		self.following = following # For visualization purposes
		self.car_id = car_id
		self.lane_id = lane_id
		self.priority = priority
		self.t = t
		self.x = x
		self.y = y
		self.heading = heading
		self.angular_V = angular_V
		self.vel = vel
		self.desired_vel = vel[0]
		self.acc = acc
		self.length = length
		self.width = width
		self.max_V = max_V
		self.max_A = max_A
		self.min_A = min_A
		''' 
		Formula:
		g = 4.019028871 x R (meters)
		'''
		self.max_lateral_g = max_lateral_g
		self.reservation = reservation

	# This is a function to check if car has cleared its lane (rearmost edge)
	def _check_for_exit(self, t_index):
		if self.lane_id == 6 or self.lane_id == 7 or self.lane_id == 8:
			# South lanes
			return True if ((self.y[t_index] - self.length/2) > dMax) else False # - self.length/2

		elif self.lane_id == 3 or self.lane_id == 4 or self.lane_id == 5:
			# East lanes
			return True if ((self.x[t_index] + self.length/2) < (dMax+6*lane_width)) else False # + self.length/2

		elif self.lane_id == 0 or self.lane_id == 1 or self.lane_id == 2:
			# North lanes
			return True if ((self.y[t_index] + self.length/2) < (dMax+6*lane_width)) else False # + self.length/2

		elif self.lane_id == 9 or self.lane_id == 10 or self.lane_id == 11:
			# West lanes
			return True if ((self.x[t_index] - self.length/2) > dMax) else False # - self.length/2

		#returns True if exited False otherwise

	def _expected_vel(self, t_index, stopping_distance):
		nominal_dist = dMax
		nominal_decel = (-self.vel[0] / (2*dMax/self.vel[0]))
		for i in range(0,t_index+1):
			expected_vel = self.vel[0] + (nominal_decel*timestep_size * i)
			nominal_dist = nominal_dist - expected_vel
			if nominal_dist <= stopping_distance:
				break
		return expected_vel

	def _update(self, t_index, follow_car = None , f_t_index = None):
		self.heading[t_index] = self.heading[t_index - 1]
		if self.lane_id == 6 or self.lane_id == 7 or self.lane_id == 8:
			# Position calculations for South lanes
			self.y[t_index] = self.y[t_index - 1] - (0.5*self.acc[t_index - 1]*(timestep_size**2)) + self.vel[t_index - 1]*timestep_size
			self.x[t_index] = self.x[t_index - 1]
			stopping_distance = (dMax - self.y[t_index] - self.length/2)
		elif self.lane_id == 3 or self.lane_id == 4 or self.lane_id == 5:
			# Position calculations for East lanes
			self.y[t_index] = self.y[t_index - 1]
			self.x[t_index] = self.x[t_index - 1] + (0.5*self.acc[t_index - 1]*(timestep_size**2)) - self.vel[t_index - 1]*timestep_size
			stopping_distance = (self.x[t_index] - (dMax+6*lane_width) - self.length/2)
		elif self.lane_id == 0 or self.lane_id == 1 or self.lane_id == 2:
			# Position calculations for North lanes
			self.x[t_index] = self.x[t_index - 1]
			self.y[t_index] = self.y[t_index - 1] + (0.5*self.acc[t_index - 1]*(timestep_size**2)) - self.vel[t_index - 1]*timestep_size
			stopping_distance = (self.y[t_index] - (dMax+6*lane_width) - self.length/2)
		elif self.lane_id == 9 or self.lane_id == 10 or self.lane_id == 11:
			# Position calculations for West lanes
			self.x[t_index] = self.x[t_index - 1] - (0.5*self.acc[t_index - 1]*(timestep_size**2)) + self.vel[t_index - 1]*timestep_size
			self.y[t_index] = self.y[t_index - 1]
			stopping_distance = (dMax - self.x[t_index] - self.length/2)
		self.vel[t_index] = self.vel[t_index - 1] + self.acc[t_index - 1]*timestep_size
		nominal_decel = (-self.vel[0] / (2*dMax/self.vel[0]))
		if follow_car is None:
			# Slow down to the line or speed up to the line
			if stopping_distance <= 1:
				# DONE
				self.acc[t_index] = 0
				self.vel[t_index] = 0
			else:
				if self.vel[t_index] > 0:
					# Slowdown to line
					line_slowdown_acc = (-self.vel[t_index] / (2*stopping_distance/self.vel[t_index]))
					if line_slowdown_acc <= nominal_decel:
						self.acc[t_index] = line_slowdown_acc
					else:
						# Speed up to line
						expected_vel = self._expected_vel(t_index, stopping_distance)
						# Attempt to speed up to expected_vel in half the stopping distance left
						line_speedup_acc = (expected_vel)
						self.acc[t_index] = line_speedup_acc
				else:
					expected_vel = self._expected_vel(t_index, stopping_distance)
					# Attempt to speed up to expected_vel in half the stopping distance left
					line_speedup_acc = (expected_vel)
					self.acc[t_index] = line_speedup_acc
		else:
			# FOLLOW CAR EXISTS
			if self.lane_id == 6 or self.lane_id == 7 or self.lane_id == 8: # South lanes
				car_gap = (follow_car.y[f_t_index] - follow_car.length/2) - (self.y[t_index] + self.length/2)
			elif self.lane_id == 3 or self.lane_id == 4 or self.lane_id == 5: # East lanes
				car_gap = (self.x[t_index] - self.length/2 ) - (follow_car.x[f_t_index] + follow_car.length/2)
			elif self.lane_id == 0 or self.lane_id == 1 or self.lane_id == 2: # North lanes
				car_gap = (self.y[t_index] - self.length/2) - (follow_car.y[f_t_index] + follow_car.length/2)
			elif self.lane_id == 9 or self.lane_id == 10 or self.lane_id == 11: # West lanes
				car_gap = (follow_car.x[f_t_index] - follow_car.length/2) - (self.x[t_index] + self.length/2)
			if stopping_distance <= 1:
				# DONE
				self.acc[t_index] = 0
				self.vel[t_index] = 0	
				print("Shouldn't have a follow car.")
			else:
				if self.vel[t_index] > 0:
					if car_gap - dSafe >= 1:
						follow_acc = (follow_car.vel[f_t_index] - self.vel[t_index])/(2*(car_gap - dSafe)/self.vel[t_index])
					else:
						follow_acc = follow_car.acc[f_t_index]
						follow_vel = follow_car.vel[f_t_index]
					line_slowdown_acc = (-self.vel[t_index] / (2*stopping_distance/self.vel[t_index]))
					if follow_acc <= nominal_decel or line_slowdown_acc <= nominal_decel:
						# Slow Down neccessary
						if follow_acc <= line_slowdown_acc:
							# Slow to follow car
							self.acc[t_index] = follow_acc
							if car_gap - dSafe < 1:
								self.vel[t_index] = follow_vel
							self.following[t_index] = True
						else:
							# Slow to line
							self.acc[t_index] = line_slowdown_acc
					else:
						# Speed up
						expected_vel = self._expected_vel(t_index, stopping_distance)
						# Attempt to speed up to expected_vel in half the stopping distance left
						line_speedup_acc = (expected_vel)
						if follow_acc <= line_speedup_acc:
							self.acc[t_index] = follow_acc
							if car_gap - dSafe < 1:
								self.vel[t_index] = follow_vel
							self.following[t_index] = True
						else:
							self.acc[t_index] = line_speedup_acc						
				else:
					# Take follow_car's acc and vel if they are less than line's
					expected_vel = self._expected_vel(t_index, stopping_distance)
					# Attempt to speed up to expected_vel in half the stopping distance left
					line_speedup_acc = (expected_vel)
					if car_gap - dSafe >= 1:
						follow_acc = follow_car.vel[f_t_index]/(2*(car_gap - dSafe)/(car_gap - dSafe))
					else:
						follow_acc = follow_car.acc[f_t_index]
						follow_vel = follow_car.vel[f_t_index]
					if follow_acc <= line_speedup_acc:
						self.acc[t_index] = follow_acc
						if car_gap - dSafe < 1:
							self.vel[t_index] = follow_vel
						self.following[t_index] = True
					else:
						self.acc[t_index] = line_speedup_acc

		# if self.vel[t_index] <= 2:
		# 	self.vel[t_index] = 0
		# 	self.acc[t_index] = 0


class carManager:
	def __init__(self, car_list = []):
		self.car_list = car_list

	def update(self, time):
		time = round(time, 3)
		for i in range(0,len(self.car_list)):
			if self.car_list[i].reservation.any(): 
				continue
			else:
				curr_t_index = None
				f_next_t_index = None
				if round(self.car_list[i].t[0],3) <= time:
					for q in range(0, len(self.car_list[i].t)):
						if time == round(self.car_list[i].t[q],3):
							curr_t_index = q
							break
					response = car_request_client(self.car_list[i].car_id, self.car_list[i].lane_id, self.car_list[i].t[curr_t_index], 
						self.car_list[i].x[curr_t_index], self.car_list[i].y[curr_t_index], self.car_list[i].heading[curr_t_index], 
						self.car_list[i].angular_V, self.car_list[i].vel[curr_t_index], self.car_list[i].desired_vel,
						self.car_list[i].acc[curr_t_index],	self.car_list[i].priority, self.car_list[i].length, self.car_list[i].width,
						self.car_list[i].max_V,	self.car_list[i].max_A, self.car_list[i].min_A, self.car_list[i].max_lateral_g)
					if response[0]: 
						print("Request accepted for Car: ",self.car_list[i].car_id," At time: ",time)
						self.car_list[i].reservation = np.append(self.car_list[i].reservation[0:curr_t_index], np.ones(len(response[1]),dtype=bool))
						self.car_list[i].x = np.append(self.car_list[i].x[0:curr_t_index], response[1])
						self.car_list[i].y = np.append(self.car_list[i].y[0:curr_t_index], response[2])
						self.car_list[i].heading = np.append(self.car_list[i].heading[0:curr_t_index], response[3])
						self.car_list[i].vel = np.append(self.car_list[i].vel[0:curr_t_index], response[4])
						self.car_list[i].t = np.append(self.car_list[i].t[0:curr_t_index], response[5])
					else:
						print("Request rejected for Car: ",self.car_list[i].car_id," At time: ",time)
						if curr_t_index+1 < len(self.car_list[i].t):
							next_t_index = curr_t_index + 1
							follow_car = None
							for j in range(i-1, -1, -1):
								if self.car_list[j].lane_id == self.car_list[i].lane_id and round(self.car_list[j].t[-1],3) >= time+1: # DEBATE placing "and self.car_list[j].reservation == False 
									follow_car = self.car_list[j]
									for u in range(0, len(follow_car.t)):
										if time == round(follow_car.t[u],3):
											f_next_t_index = u + 1
											break
									if follow_car._check_for_exit(f_next_t_index):
										follow_car = None
									break
							self.car_list[i]._update(next_t_index, follow_car, f_next_t_index)
				else:
					break


# This is the client's fuction sending a car's info to the server (intersectionManager.py service) as a request to pass
def car_request_client(car_id, lane_id, t, x, y, heading, angular_V, vel, desired_vel, acc, priority, length, width, max_V, max_A, min_A, max_lateral_g):
	rospy.wait_for_service('car_request')
	# print(car_id, lane_id, t, x, y, heading, angular_V, vel, acc, priority, length, width, max_V, max_A, min_A, max_lateral_g)
	try:
		car_request = rospy.ServiceProxy('car_request', Request)
		response = car_request(car_id, int(lane_id), t, x, y, heading, angular_V, vel, desired_vel, acc, priority, length, width, max_V, max_A, min_A, max_lateral_g) 
		resp = (response.success, response.x, response.y, response.heading, response.vel, response.t)
		return resp
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		return False, [], [], [], [], []

#-------------------- Functions for generating cars ---------------------------------
def match_spawn_count(cars_spawned, linearly = True):
	car_id = 1
	spawn_vel_range = (17.88, 20.11) # All speeds in meters/sec (effectively (40, 45) mph)
	if not linearly:
		pass
	else:
		spawn_car_every = time_to_complete/num_cars
		for t in np.arange(0, time_to_complete + timestep_size, timestep_size):
			spawn_count = int(t/spawn_car_every)
			new_spawns = spawn_count - len(cars_spawned)
			if new_spawns is not 0:
				free_lanes = check_voided_lanes(t, cars_spawned)
				if not free_lanes: # Checks if free_lanes isempty
					continue
				if new_spawns > len(free_lanes):
					new_spawns = len(free_lanes)
					print ("Not enough free lanes, the rest of the needed cars will attempt to spawn next timestep.")
				rand_car_lanes = np.random.choice(free_lanes, new_spawns, replace=False)
				for lane in rand_car_lanes:
					x, y, heading, vel, acc, foll, reserv = init_state(t, lane)
					vel[0] = (spawn_vel_range[1] - spawn_vel_range[0])*np.random.random_sample() + spawn_vel_range[0]
					acc[0] = -vel[0] / (2*dMax/vel[0])
					cars_spawned.append(car(car_id, lane, np.arange(t, end_time + timestep_size, timestep_size), x, y, heading, 0, vel, acc, foll, reserv))
					car_id = car_id + 1
	return cars_spawned

def check_voided_lanes(t, cars_spawned):
# Check if cars in the same lane arent within a certain time tSafe of each other before adding them
	lane_set = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}
	free_lanes = lane_set
	for i in range(1, len(cars_spawned)+1):
		if cars_spawned[-i].t[0] >= t-tSafe:
			free_lanes = free_lanes - {cars_spawned[-i].lane_id}
		else:
			break
	return list(free_lanes) 

def init_state(t, lane):
	x = np.arange(t, end_time + timestep_size, timestep_size)
	y = np.arange(t, end_time + timestep_size, timestep_size)
	h = np.arange(t, end_time + timestep_size, timestep_size)
	vel = np.arange(t, end_time + timestep_size, timestep_size)
	acc = np.arange(t, end_time + timestep_size, timestep_size)
	following = np.zeros(len(x), dtype=bool)
	reservation = np.zeros(len(x), dtype=bool)
	x[0] = x_states[lane]
	y[0] = y_states[lane]
	h[0] = h_states[lane]
	return x, y, h, vel, acc, following, reservation

def main():
	#warnings.filterwarnings("ignore")
	# ----------------------------- Sim configurations ------------------------------
	np.random.seed(1)
	global x_states, y_states, h_states, timestep_size, num_cars, tSafe, time_to_complete, end_time, dMax, dSafe, lane_width
	timestep_size = 0.1 # Must be a float
	num_cars = 40.0 # Must be a float
	tSafe = 0.5 # Must be a float
	time_to_complete = 50.0 # Must be a float
	end_time = 100.0 # Must be a float
	cars_spawned = []
	dMax = 148 
	dSafe = 2
	lane_width = 3.66 
	x_states = {0: dMax+0.5*lane_width,
	 1:dMax+1.5*lane_width,
	 2:dMax+2.5*lane_width,
	 3:2*dMax+6*lane_width,
	 4:2*dMax+6*lane_width,
	 5:2*dMax+6*lane_width,
	 6:dMax+5.5*lane_width,
	 7:dMax+4.5*lane_width,
	 8:dMax+3.5*lane_width,
	 9:0,
	 10:0,
	 11:0
	 }
	y_states = {0:2*dMax+6*lane_width,
	 1:2*dMax+6*lane_width,
	 2:2*dMax+6*lane_width,
	 3:dMax+5.5*lane_width,
	 4:dMax+4.5*lane_width,
	 5:dMax+3.5*lane_width,
	 6:0,
	 7:0,
	 8:0,
	 9:dMax+0.5*lane_width,
	 10:dMax+1.5*lane_width,
	 11:dMax+2.5*lane_width
	 }
	h_states = {0:180,
	 1:180,
	 2:180,
	 3:270,
	 4:270,
	 5:270,
	 6:0,
	 7:0,
	 8:0,
	 9:90,
	 10:90,
	 11:90
	 }

	rospy.init_node('car_manager')
	rate = rospy.Rate(100.0)

	print("Generating Cars...")
	cars_spawned = match_spawn_count(cars_spawned)
	print("Generated Cars.\n Running Simulation, this may take a while...")
	cm = carManager(cars_spawned)
	start_time = time.time()
	sim_time = 0
	#pdb.set_trace()
	while not rospy.is_shutdown():
		cm.update(sim_time)
		sim_time = sim_time + timestep_size
		if round(sim_time,3) >= end_time + timestep_size:
			#pdb.set_trace()
			completion_time = time.time() - start_time
			print "Simulation Complete \n Execution Time: ", round(completion_time,2), " seconds"
			# print "Calculating Average Delay..."
			# print ("Average Delay is: ", cm.calculate_delay)
			print("Initiating Visualization. Please run Rviz")
			raw_input('Press ENTER to start Visualization')
			visualizeSim.main(cm.car_list, dMax, lane_width, timestep_size, end_time)
			print("Visualization complete and shutdown successful!")
			rospy.signal_shutdown("Simulation Complete")
		else:
			rate.sleep()
		
if __name__ == '__main__':
	main()