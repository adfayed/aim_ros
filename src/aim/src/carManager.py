#!/usr/bin/env python  
import rospy
import numpy as np
#from aim.srv import *

# Class for each car's attr
class car:
	def __init__(self, car_id, lane_id, t, x, y, heading, angular_V, vel, acc, priority = 0,
		length = 4.9784, width = 1.96342, max_V = 249.448, max_A = 43.47826087, min_A = 43.47826087, 
		max_lateral_g = 1.2 , reservation = False):
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


	def _update(self, time, follow_car = None):
		# if self.lane_id == 0:
		# elif self.lane_id == 1:
		# elif self.lane_id == 2:
		# elif self.lane_id == 3:
		# elif self.lane_id == 4:
		# elif self.lane_id == 5:
		# elif self.lane_id == 6:
		# elif self.lane_id == 7:
		# elif self.lane_id == 8:
		# elif self.lane_id == 9:
		# elif self.lane_id == 10:
		# elif self.lane_id == 11:
		if follow_car is None:
			
		else:

		return self


class carManager:
	def __init__(self, car_list = []):
		self.car_list = car_list
		self.reserved_cars

	def sendRequests(self, time):
		for car in self.car_list:
			if car.reservation is not True and car.t[0] <= time:
				car.reservation = car_request_client(car.car_id, car.lane_id, car.t, 
					car.x, car.y, car.heading, car.angular_V, car.vel, car.acc, car.priority, car.length, car.width, car.max_V, car.max_A, car.min_A, car.max_lateral_g)

	def update(self, time, timestep_size):
		for i in len(self.car_list):
			follow_car = None
			if self.car_list[i].t[0] <= time+timestep_size:
				for j in range(i-1, 0, -1):
					if self.car_list[j].lane_id is self.car_list[i].lane_id:
						follow_car = self.car_list[j]
			self.car_list[i] = self.car_list[i]._update(time, follow_car)


# This is the client's fuction sending a car's info to the server (intersectionManager.py service) as a request to pass
def car_request_client(car_id, lane_id, t, x, y, heading, angular_V, vel, acc, priority, length, width, max_V, max_A, min_A, max_lateral_g):
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

#-------------------- Functions for generating cars ---------------------------------
def match_spawn_count(num_cars, timestep_size, tSafe, time_to_complete, end_time, cars_spawned, linearly = True):
	car_id = 1
	spawn_vel_range = (17.88, 26.83) # All speeds in meters/sec (effectively (40, 60) mph)
	spawn_car_every = time_to_complete/num_cars
	for t in np.arange(0, time_to_complete + timestep_size, timestep_size):
		spawn_count = int(t/spawn_car_every)
		new_spawns = spawn_count - len(cars_spawned)
		if new_spawns is not 0:
			free_lanes = check_voided_lanes(t, tSafe, cars_spawned)
			if not free_lanes: # Checks if free_lanes isempty
				continue
			if new_spawns > len(free_lanes):
				new_spawns = len(free_lanes)
				print ("Not enough free lanes, the rest of the needed cars will attempt to spawn next timestep.")
			rand_car_lanes = np.random.choice(free_lanes, new_spawns, replace=False)
			for lane in rand_car_lanes:
				x, y, heading = init_state(t, end_time, lane)
				cars_spawned.append(car(car_id, lane, np.arange(t, end_time + timestep_size, timestep_size), x, y, heading, 0, (spawn_vel_range[1] - spawn_vel_range[0])*np.random.random_sample() + spawn_vel_range[0], 0))
				car_id = car_id + 1
	return cars_spawned

def check_voided_lanes(t, tSafe, cars_spawned):
#check if cars in the same lane arent within a certain time tSafe of each other before adding them
	lane_set = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}
	free_lanes = lane_set
	for i in range(1, len(cars_spawned)+1):
		if cars_spawned[-i].t[0] >= t-tSafe:
			free_lanes = free_lanes - {cars_spawned[-i].lane_id}
		else:
			break
	return list(free_lanes) 

def init_state(t, end_time, lane):
	x = np.arange(t, end_time + timestep_size, timestep_size)
	y = np.arange(t, end_time + timestep_size, timestep_size)
	h = np.arange(t, end_time + timestep_size, timestep_size)
	x[0] = x_states[lane]
	y[0] = y_states[lane]
	h[0] = h_states[lane]
	return x, y, h



def main():
	# ----------------------------- Sim configurations ------------------------------
	np.random.seed(0)
	timestep_size = 0.1 # Must be a float
	num_cars = 500.0 # Must be a float
	tSafe = 1.0 # Must be a float
	time_to_complete = 500.0 # Must be a float
	end_time = 1000.0 # Must be a float
	cars_spawned = []
	dMax = 148 
	lane_width = 3.66 
	global x_states, y_states, h_states
	x_states = {0: dMax+3.5*lane_width,
	 1:dMax+4.5*lane_width,
	 2:dMax+5.5*lane_width,
	 3:2*dMax+6*lane_width,
	 4:2*dMax+6*lane_width,
	 5:2*dMax+6*lane_width,
	 6:dMax+2.5*lane_width,
	 7:dMax+1.5*lane_width,
	 8:dMax+0.5*lane_width,
	 9:0,
	 10:0,
	 11:0
	 }
	y_states = {0:0,
	 1:0,
	 2:0,
	 3:dMax+3.5*lane_width,
	 4:dMax+4.5*lane_width,
	 5:dMax+5.5*lane_width,
	 6:2*dMax+6*lane_width,
	 7:2*dMax+6*lane_width,
	 8:2*dMax+6*lane_width,
	 9:dMax+2.5*lane_width,
	 10:dMax+1.5*lane_width,
	 11:dMax+0.5*lane_width
	 }
	h_states = {0:0,
	 1:0,
	 2:0,
	 3:270,
	 4:270,
	 5:270,
	 6:180,
	 7:180,
	 8:180,
	 9:90,
	 10:90,
	 11:90
	 }


	cars_spawned = match_spawn_count(num_cars, timestep_size, tSafe, time_to_complete, end_time, cars_spawned)
	cm = carManager(cars_spawned)
	rospy.init_node('car_manager')
	rate = rospy.Rate(100.0)

	while not rospy.is_shutdown():
		global time
		print ("Car Manager is initializing...")
		cm.initialize()
		print ("Starting simulation.")
		for time in np.arange(0, end_time + timestep_size, timestep_size):
			cm.update(time)
			response = cm.sendRequests(time)
			
			#if response is not None:
			#	cm.update(time)
	# 	   # Clean reserved and write to csv
	# 	   # Update car's and increment t
	 	rate.sleep()

if __name__ == '__main__':
	main()