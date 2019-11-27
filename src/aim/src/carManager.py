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
	def __init__(self, car_id, lane_id, t, x, y, heading, angular_V, vel, acc, priority = 0,
		length = 4.9784, width = 1.96342, max_V = 249.448, max_A = 43.47826087, min_A = 43.47826087, 
		max_lateral_g = 1.2 , reservation = False):
		self.follow_car = None # For visualization purposes
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
		''' 
		Formula:
		g = 4.019028871 x R (meters)
		'''
		self.max_lateral_g = max_lateral_g
		self.reservation = reservation


	def _update(self, curr_t_index, follow_car = None , f_curr_t_index = None):
		if self.reservation is True:
			# Follow x, y, heading, vel given by IM
			pass
		else:
			self.heading[curr_t_index] = self.heading[curr_t_index - 1]
			if self.lane_id == 6 or self.lane_id == 7 or self.lane_id == 8:
				# Position calculations for South lanes
				self.y[curr_t_index] = self.y[curr_t_index - 1] - (0.5*self.acc[curr_t_index - 1]*(timestep_size**2)) + self.vel[curr_t_index - 1]*timestep_size
				self.x[curr_t_index] = self.x[curr_t_index - 1]
				stopping_distance = (dMax - self.y[curr_t_index] - self.length/2)
			elif self.lane_id == 3 or self.lane_id == 4 or self.lane_id == 5:
				# Position calculations for East lanes
				self.y[curr_t_index] = self.y[curr_t_index - 1]
				self.x[curr_t_index] = self.x[curr_t_index - 1] + (0.5*self.acc[curr_t_index - 1]*(timestep_size**2)) - self.vel[curr_t_index - 1]*timestep_size
				stopping_distance = (self.x[curr_t_index] - (dMax+6*lane_width) - self.length/2)
			elif self.lane_id == 0 or self.lane_id == 1 or self.lane_id == 2:
				# Position calculations for North lanes
				self.x[curr_t_index] = self.x[curr_t_index - 1]
				self.y[curr_t_index] = self.y[curr_t_index - 1] + (0.5*self.acc[curr_t_index - 1]*(timestep_size**2)) - self.vel[curr_t_index - 1]*timestep_size
				stopping_distance = (self.y[curr_t_index] - (dMax+6*lane_width) - self.length/2)
			elif self.lane_id == 9 or self.lane_id == 10 or self.lane_id == 11:
				# Position calculations for West lanes
				self.x[curr_t_index] = self.x[curr_t_index - 1] - (0.5*self.acc[curr_t_index - 1]*(timestep_size**2)) + self.vel[curr_t_index - 1]*timestep_size
				self.y[curr_t_index] = self.y[curr_t_index - 1]
				stopping_distance = (dMax - self.x[curr_t_index] - self.length/2)
			self.vel[curr_t_index] = self.vel[curr_t_index - 1] + self.acc[curr_t_index - 1]*timestep_size
			if follow_car is None:
				# if stopping_distance <= dSafe:
				#     self.acc[curr_t_index] = -10 
				# else:
				self.acc[curr_t_index] = (-self.vel[curr_t_index - 1] / (2*stopping_distance/self.vel[curr_t_index - 1]))
			else:
				if self.lane_id == 6 or self.lane_id == 7 or self.lane_id == 8: # South lanes
					car_gap = (follow_car.y[f_curr_t_index] - follow_car.length/2) - (self.y[curr_t_index] + self.length/2)
				elif self.lane_id == 3 or self.lane_id == 4 or self.lane_id == 5: # East lanes
					car_gap = (self.x[curr_t_index] - self.length/2 ) - (follow_car.x[f_curr_t_index] + follow_car.length/2)
				elif self.lane_id == 0 or self.lane_id == 1 or self.lane_id == 2: # North lanes
					car_gap = (self.y[curr_t_index] - self.length/2) - (follow_car.y[f_curr_t_index] + follow_car.length/2)
				elif self.lane_id == 9 or self.lane_id == 10 or self.lane_id == 11: # West lanes
					car_gap = (follow_car.x[f_curr_t_index] - follow_car.length/2) - (self.x[curr_t_index] + self.length/2)
				self.acc[curr_t_index] = (follow_car.vel[f_curr_t_index - 1] - self.vel[curr_t_index - 1])/(2*(car_gap - dSafe)/self.vel[curr_t_index - 1])
				if car_gap <= 1.5*dSafe:
					self.acc[curr_t_index] = self.acc[curr_t_index] - math.exp(1.5*dSafe - car_gap)
					# try:
					# 	self.acc[curr_t_index] = self.acc[curr_t_index] - math.exp(0.5*dSafe - car_gap)
					# except OverflowError as err:
					# 	print "Overflow Error: Something went wrong"
					# 	self.acc[curr_t_index] = 0
					# except:
					# 	type, value, tb = sys.exc_info()
					# 	traceback.print_exc()
					# 	last_frame = lambda tb=tb: last_frame(tb.tb_next) if tb.tb_next else tb
					# 	frame = last_frame().tb_frame
					# 	ns = dict(frame.f_globals)
					# 	ns.update(frame.f_locals)
					# 	code.interact(local=ns)
				if follow_car.reservation is True:
					self.acc[curr_t_index] = min((follow_car.vel[f_curr_t_index - 1] - self.vel[curr_t_index - 1])/(2*(car_gap - dSafe)/self.vel[curr_t_index - 1]),
						(-self.vel[curr_t_index - 1] / (2*stopping_distance/self.vel[curr_t_index - 1])))
			if self.vel[curr_t_index] <= 2:
				self.vel[curr_t_index] = 0
				self.acc[curr_t_index] = 0
		#return curr_t_index


class carManager:
	def __init__(self, car_list = []):
		self.car_list = car_list

	def update(self, time):
		for i in range(0,len(self.car_list)):
			if self.car_list[i].reservation: 
				continue
			else:
				curr_t_index = None
				f_curr_t_index = None
				if round(self.car_list[i].t[0],3) <= time:
					for q in range(0, len(self.car_list[i].t)):
						if time == round(self.car_list[i].t[q],3):
							curr_t_index = q
							break
					response = car_request_client(self.car_list[i].car_id, self.car_list[i].lane_id, self.car_list[i].t[curr_t_index], 
						self.car_list[i].x[curr_t_index], self.car_list[i].y[curr_t_index], self.car_list[i].heading[curr_t_index], 
						self.car_list[i].angular_V, self.car_list[i].vel[curr_t_index], self.car_list[i].acc[curr_t_index], 
						self.car_list[i].priority, self.car_list[i].length, self.car_list[i].width, self.car_list[i].max_V, 
						self.car_list[i].max_A, self.car_list[i].min_A, self.car_list[i].max_lateral_g)
					if response[5]:
						self.car_list[i].x = self.car_list[i].x[0:curr_t_index].append(response[0])
						self.car_list[i].y = self.car_list[i].y[0:curr_t_index].append(response[1])
						self.car_list[i].heading = self.car_list[i].heading[0:curr_t_index].append(response[2])
						self.car_list[i].vel = self.car_list[i].vel[0:curr_t_index].append(response[3])
						#self.car_list[i].t
					else:
						if round(self.car_list[i].t[1],3) <= time:
							follow_car = None
							for j in range(i-1, -1, -1):
								if self.car_list[j].lane_id == self.car_list[i].lane_id: # DEBATE placing "and self.car_list[j].reservation == False 
									follow_car = self.car_list[j]
									for u in range(0, len(follow_car.t)):
										if time == round(follow_car.t[u],3):
											f_curr_t_index = u
											break
									self.car_list[i].follow_car = self.car_list[j] # For visualization purposes
									break
							self.car_list[i]._update(curr_t_index+1, follow_car, f_curr_t_index)
				else:
					break


# This is the client's fuction sending a car's info to the server (intersectionManager.py service) as a request to pass
def car_request_client(car_id, lane_id, t, x, y, heading, angular_V, vel, acc, priority, length, width, max_V, max_A, min_A, max_lateral_g):
	rospy.wait_for_service('car_request')
	try:
		car_request = rospy.ServiceProxy('car_request', Request)
		resp_x, resp_y, resp_heading, resp_vel, resp_t, resp_success = car_request(car_id, lane_id, priority, t, x, y, heading, angular_V, vel, acc, length, width, max_V, max_A, min_A, max_lateral_g) 
		print ("Request has returned ", resp1, " for car_id: ", car_id, ", lane_id: ", lane_id, ", t: ", t, ", x: ", x, ", y:", y, ", heading: ", heading, 
			", angular_V: ", angular_V, ", vel: ", vel, ", acc: ", acc)
		resp = (resp_x, resp_y, resp_heading, resp_vel, resp_t, resp_success)
		return resp
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		return False

#-------------------- Functions for generating cars ---------------------------------
def match_spawn_count(cars_spawned, linearly = True):
	car_id = 1
	spawn_vel_range = (17.88, 26.83) # All speeds in meters/sec (effectively (40, 60) mph)
	if not linearly:
		pass
		# spawn_car_every = 0
		# for t in np.arange(0, time_to_complete + timestep_size, timestep_size):
		# 	spawn_car_every = 8*time_to_complete/num_cars
		# 	if t > 1/8*time_to_complete:
		# 		spawn_car_every = 4*time_to_complete/num_cars
		# 	elif t > 1/4*time_to_complete:
		# 		spawn_car_every = 2*time_to_complete/num_cars
		# 	elif: t> 1/2*time_to_complete:
		# 		spawn_car_every = time_to_complete/num_cars
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
					x, y, heading, vel, acc = init_state(t, lane)
					vel[0] = (spawn_vel_range[1] - spawn_vel_range[0])*np.random.random_sample() + spawn_vel_range[0]
					acc[0] = -vel[0] / (2*dMax/vel[0])
					cars_spawned.append(car(car_id, lane, np.arange(t, end_time + timestep_size, timestep_size), x, y, heading, 0, vel, acc))
					car_id = car_id + 1
	return cars_spawned

def check_voided_lanes(t, cars_spawned):
#check if cars in the same lane arent within a certain time tSafe of each other before adding them
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
	x[0] = x_states[lane]
	y[0] = y_states[lane]
	h[0] = h_states[lane]
	return x, y, h, vel, acc

def main():
	warnings.filterwarnings("ignore")
	# ----------------------------- Sim configurations ------------------------------
	np.random.seed(1)
	global x_states, y_states, h_states, timestep_size, num_cars, tSafe, time_to_complete, end_time, dMax, dSafe, lane_width
	timestep_size = 0.1 # Must be a float
	num_cars = 40.0 # Must be a float
	tSafe = 0.2 # Must be a float
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
	# print("Generating Cars...")
	# cars_spawned = match_spawn_count(cars_spawned)
	# print("Generated Cars.\n Running Simulation, this may take a while...")
	# cm = carManager(cars_spawned)
	# start_time = time.time()

	# for sim_time in np.arange(0, end_time + timestep_size, timestep_size):
	# 	sim_time = round(sim_time,3)
	# 	# if round(sim_time,1).is_integer():
	# 	# 	print("Simulation time:", round(sim_time,1))
	# 	cm.update(sim_time)

	# completion_time = time.time() - start_time
	# print "Simulation Complete \n Execution Time: ", round(completion_time,2), " seconds"
	# print("Initiating Visualization. Please run Rviz")

	# visualizeSim.main(cm.car_list, dMax, lane_width, timestep_size, end_time)
	# print("Visualization complete and shutdown successful!")




	rospy.init_node('car_manager')
	rate = rospy.Rate(100.0)

	print("Generating Cars...")
	cars_spawned = match_spawn_count(cars_spawned)
	print("Generated Cars.\n Running Simulation, this may take a while...")
	cm = carManager(cars_spawned)
	start_time = time.time()
	sim_time = 0
	while not rospy.is_shutdown():
		cm.update(sim_time)
		sim_time = sim_time + timestep_size
		if sim_time == end_time + timestep_size:
			completion_time = time.time() - start_time
			print "Simulation Complete \n Execution Time: ", round(completion_time,2), " seconds"
			print("Initiating Visualization. Please run Rviz")

			visualizeSim.main(cm.car_list, dMax, lane_width, timestep_size, end_time)
			print("Visualization complete and shutdown successful!")
			rospy.signal_shutdown("Simulation Complete")
		else:
			rate.sleep()
		
if __name__ == '__main__':
	main()