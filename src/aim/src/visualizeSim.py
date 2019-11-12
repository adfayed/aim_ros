#!/usr/bin/env python 
import rospy
from std_msgs.msg import String
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import numpy as np
import math
import pdb
import tf

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
        for i in range(0, len(self.t)):
            if time == round(self.t[i],3):
                curr_t_index = i
                break
        if self.reservation is True:
            # Follow x, y, heading, vel
            pass
        else:
            self.heading[curr_t_index] = self.heading[curr_t_index - 1]
            #braking_slope = self.vel[curr_t_index - 1]/((dMax - self.x[curr_t_index - 1])/self.vel[curr_t_index - 1])
            if self.lane_id == 0 or self.lane_id == 1 or self.lane_id == 2:
                # Position calculations for South lanes
                self.y[curr_t_index] = self.y[curr_t_index - 1] - (0.5*self.acc[curr_t_index - 1]*(timestep_size**2)) + self.vel[curr_t_index - 1]*timestep_size
                self.x[curr_t_index] = self.x[curr_t_index - 1]
            elif self.lane_id == 3 or self.lane_id == 4 or self.lane_id == 5:
                # Position calculations for East lanes
                self.y[curr_t_index] = self.y[curr_t_index - 1]
                self.x[curr_t_index] = self.x[curr_t_index - 1] + (0.5*self.acc[curr_t_index - 1]*(timestep_size**2)) - self.vel[curr_t_index - 1]*timestep_size
            elif self.lane_id == 6 or self.lane_id == 7 or self.lane_id == 8:
                # Position calculations for North lanes
                self.x[curr_t_index] = self.x[curr_t_index - 1]
                self.y[curr_t_index] = self.y[curr_t_index - 1] + (0.5*self.acc[curr_t_index - 1]*(timestep_size**2)) - self.vel[curr_t_index - 1]*timestep_size
            elif self.lane_id == 9 or self.lane_id == 10 or self.lane_id == 11:
                # Position calculations for West lanes
                self.x[curr_t_index] = self.x[curr_t_index - 1] - (0.5*self.acc[curr_t_index - 1]*(timestep_size**2)) + self.vel[curr_t_index - 1]*timestep_size
                self.y[curr_t_index] = self.y[curr_t_index - 1]
            new_speed_lead = self.vel[curr_t_index - 1] + self.acc[curr_t_index - 1]*timestep_size
            if follow_car is None:
                new_speed = new_speed_lead
                self.acc[curr_t_index] = (-self.vel[0] / (2*dMax/self.vel[0]))
            else:
                for i in range(0, len(follow_car.t)):
                    if time == round(follow_car.t[i],3):
                        f_curr_t_index = i
                        break
                car_gap = (follow_car.x[f_curr_t_index] - self.x[curr_t_index]) + (follow_car.y[f_curr_t_index] - self.y[curr_t_index])
                #new_speed_follow = follow_car.vel[f_curr_t_index] + ((car_gap - dSafe)*timestep_size)
                #self.acc[curr_t_index] = (follow_car.vel[f_curr_t_index - 1]**2 - self.vel[curr_t_index - 1]**2)/2*(car_gap - dSafe)
                self.acc[curr_t_index] = (follow_car.vel[f_curr_t_index - 1] - self.vel[curr_t_index - 1])/(2*(car_gap - dSafe)/self.vel[curr_t_index - 1])
                new_speed_follow = self.vel[curr_t_index - 1] + self.acc[curr_t_index - 1]*timestep_size
                if follow_car.reservation is True:
                    # Take Minimums 
                    new_speed = min(new_speed_follow, new_speed_lead)
                    #self.acc[curr_t_index] = min(self.acc[curr_t_index], follow_car.acc[f_curr_t_index])
                else:
                    # Stop behind follow_car by dSafe
                    new_speed = new_speed_follow
                    #self.acc[curr_t_index] = min(self.acc[curr_t_index], follow_car.acc[f_curr_t_index])
            self.vel[curr_t_index] = new_speed
            if new_speed <= 3:
                self.vel[curr_t_index] = 0
                self.acc[curr_t_index] = 0
        return self


class carManager:
    def __init__(self, car_list = []):
        self.car_list = car_list

    def sendRequests(self, time):
        for car in self.car_list:
            if car.reservation is not True and car.t[0] <= time:
                car.reservation = car_request_client(car.car_id, car.lane_id, car.t, 
                    car.x, car.y, car.heading, car.angular_V, car.vel, car.acc, car.priority, car.length, car.width, car.max_V, car.max_A, car.min_A, car.max_lateral_g)

    def update(self, time):
        for i in range(0,len(self.car_list)):
            if round(self.car_list[i].t[1],3) <= time:
                follow_car = None
                for j in range(i-1, -1, -1):
                    if self.car_list[j].lane_id == self.car_list[i].lane_id:
                        follow_car = self.car_list[j]
                        break
                self.car_list[i] = self.car_list[i]._update(time, follow_car)
            else:
                break


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
def match_spawn_count(cars_spawned, linearly = True):
    car_id = 1
    spawn_vel_range = (17.88, 26.83) # All speeds in meters/sec (effectively (40, 60) mph)
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
    # ----------------------------- Sim configurations ------------------------------
    np.random.seed(0)
    global x_states, y_states, h_states, timestep_size, num_cars, tSafe, time_to_complete, end_time, dMax, dSafe, lane_width
    timestep_size = 0.1 # Must be a float
    num_cars = 40.0 # Must be a float
    tSafe = 0.2 # Must be a float
    time_to_complete = 10.0 # Must be a float
    end_time = 100.0 # Must be a float
    cars_spawned = []
    dMax = 148 
    dSafe = 2
    lane_width = 3.66 
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

    cars_spawned = match_spawn_count(cars_spawned)
    cm = carManager(cars_spawned)
    
    for time in np.arange(0, end_time + timestep_size, timestep_size):
        time = round(time,3)
        cm.update(time)

    rospy.init_node('marker_publisher')
    pub = rospy.Publisher('aim_marker_array', MarkerArray, queue_size=100)
    rate = rospy.Rate(10.0)

    marker_array = MarkerArray()
    south_left_bound_p1 = Point(dMax, 0, 0)
    south_left_bound_p2 = Point(dMax, dMax, 0)
    south_right_bound_p1 = Point(dMax+6*lane_width, 0, 0)
    south_right_bound_p2 = Point(dMax+6*lane_width, dMax, 0)

    west_left_bound_p1 = Point(0, dMax+6*lane_width, 0)
    west_left_bound_p2 = Point(dMax, dMax+6*lane_width, 0)
    west_right_bound_p1 = Point(0, dMax, 0)
    west_right_bound_p2 = Point(dMax, dMax, 0)

    east_left_bound_p1 = Point((2*dMax)+6*lane_width, dMax, 0)
    east_left_bound_p2 = Point(dMax+6*lane_width, dMax, 0)
    east_right_bound_p1 = Point((2*dMax)+6*lane_width, dMax+6*lane_width, 0)
    east_right_bound_p2 = Point(dMax+6*lane_width, dMax+6*lane_width, 0)

    north_left_bound_p1 = Point(dMax+6*lane_width, (2*dMax)+6*lane_width, 0)
    north_left_bound_p2 = Point(dMax+6*lane_width, dMax+6*lane_width, 0)
    north_right_bound_p1 = Point(dMax, (2*dMax)+6*lane_width, 0)
    north_right_bound_p2 = Point(dMax, dMax+6*lane_width, 0)

    south_middle_line_p1 = Point(dMax + 3*lane_width, 0, 0)
    south_middle_line_p2 = Point(dMax + 3*lane_width, dMax, 0)

    west_middle_line_p1 = Point(0, dMax+3*lane_width, 0)
    west_middle_line_p2 = Point(dMax, dMax+3*lane_width, 0)

    east_middle_line_p1 = Point(2*dMax+6*lane_width, dMax+3*lane_width, 0)
    east_middle_line_p2 = Point(dMax+6*lane_width, dMax+3*lane_width, 0)

    north_middle_line_p1 = Point(dMax + 3*lane_width, 2*dMax+6*lane_width, 0)
    north_middle_line_p2 = Point(dMax + 3*lane_width, dMax+6*lane_width, 0)

    north_text = Marker()
    north_text.header.frame_id = "map"
    north_text.header.stamp = rospy.Time(0)
    north_text.id = 99996
    north_text.ns = "aim"
    north_text.type = Marker.TEXT_VIEW_FACING
    north_text.action = Marker.ADD
    north_text.scale.z = 10
    north_text.color.a = 1.0
    north_text.color.r = 0.0
    north_text.color.g = 0.0
    north_text.color.b = 0.0
    north_text.pose.position.x = 0.85*dMax
    north_text.pose.position.y = 1.85*dMax+6*lane_width
    north_text.pose.position.z = 0
    north_text.text = "NORTH"
    east_text = Marker()
    east_text.header.frame_id = "map"
    east_text.header.stamp = rospy.Time(0)
    east_text.id = 99995
    east_text.ns = "aim"
    east_text.type = Marker.TEXT_VIEW_FACING
    east_text.action = Marker.ADD
    east_text.scale.z = 10
    east_text.color.a = 1.0
    east_text.color.r = 0.0
    east_text.color.g = 0.0
    east_text.color.b = 0.0
    east_text.pose.position.x = 1.85*dMax+6*lane_width
    east_text.pose.position.y = 1.15*dMax+6*lane_width
    east_text.pose.position.z = 0
    east_text.text = "EAST"
    west_text = Marker()
    west_text.header.frame_id = "map"
    west_text.header.stamp = rospy.Time(0)
    west_text.id = 99994
    west_text.ns = "aim"
    west_text.type = Marker.TEXT_VIEW_FACING
    west_text.action = Marker.ADD
    west_text.scale.z = 10
    west_text.color.a = 1.0
    west_text.color.r = 0.0
    west_text.color.g = 0.0
    west_text.color.b = 0.0
    west_text.pose.position.x = 0.15*dMax
    west_text.pose.position.y = 1.15*dMax+6*lane_width
    west_text.pose.position.z = 0
    west_text.text = "WEST"
    south_text = Marker()
    south_text.header.frame_id = "map"
    south_text.header.stamp = rospy.Time(0)
    south_text.id = 99993
    south_text.ns = "aim"
    south_text.type = Marker.TEXT_VIEW_FACING
    south_text.action = Marker.ADD
    south_text.scale.z = 10
    south_text.color.a = 1.0
    south_text.color.r = 0.0
    south_text.color.g = 0.0
    south_text.color.b = 0.0
    south_text.pose.position.x = 0.85*dMax
    south_text.pose.position.y = 0.15*dMax
    south_text.pose.position.z = 0
    south_text.text = "SOUTH"
    marker_array.markers.append(north_text)
    marker_array.markers.append(east_text)
    marker_array.markers.append(west_text)
    marker_array.markers.append(south_text)

    middle_lines = Marker()
    middle_lines.header.frame_id = "map"
    middle_lines.header.stamp = rospy.Time(0)
    middle_lines.id = 99998
    middle_lines.ns = "aim"
    middle_lines.type = Marker.LINE_LIST
    middle_lines.action = Marker.ADD
    middle_lines.scale.x = 0.6
    middle_lines.color.a = 1.0
    middle_lines.color.r = 1.0
    middle_lines.color.g = 1.0
    middle_lines.color.b = 1.0
    middle_lines.points.append(south_middle_line_p1)
    middle_lines.points.append(south_middle_line_p2)
    middle_lines.points.append(west_middle_line_p1)
    middle_lines.points.append(west_middle_line_p2)
    middle_lines.points.append(east_middle_line_p1)
    middle_lines.points.append(east_middle_line_p2)
    middle_lines.points.append(north_middle_line_p1)
    middle_lines.points.append(north_middle_line_p2)
    marker_array.markers.append(middle_lines)

    road_bounds = Marker()
    road_bounds.header.frame_id = "map"
    road_bounds.header.stamp = rospy.Time(0)
    road_bounds.id = 99999
    road_bounds.ns = "aim"
    road_bounds.type = Marker.LINE_LIST
    road_bounds.action = Marker.ADD
    road_bounds.scale.x = 1
    road_bounds.color.a = 1.0
    road_bounds.color.r = 0.0
    road_bounds.color.g = 0.0
    road_bounds.color.b = 0.0
    road_bounds.points.append(south_left_bound_p1)
    road_bounds.points.append(south_left_bound_p2)
    road_bounds.points.append(south_right_bound_p1)
    road_bounds.points.append(south_right_bound_p2)
    road_bounds.points.append(west_left_bound_p1)
    road_bounds.points.append(west_left_bound_p2)
    road_bounds.points.append(west_right_bound_p1)
    road_bounds.points.append(west_right_bound_p2)
    road_bounds.points.append(east_left_bound_p1)
    road_bounds.points.append(east_left_bound_p2)
    road_bounds.points.append(east_right_bound_p1)
    road_bounds.points.append(east_right_bound_p2)
    road_bounds.points.append(north_left_bound_p1)
    road_bounds.points.append(north_left_bound_p2)
    road_bounds.points.append(north_right_bound_p1)
    road_bounds.points.append(north_right_bound_p2)
    marker_array.markers.append(road_bounds)

    for i in cm.car_list:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time(0)
        marker.id = i.car_id
        marker.ns = "aim"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 2
        marker.scale.y = 4
        marker.scale.z = 1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, float(math.radians(i.heading[0]))))
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.mesh_resource = "package://aim/meshes/complete.stl"
        marker_array.markers.append(marker)
    time = 0
    while not rospy.is_shutdown():
        for k in range(len(cm.car_list)):
            marker_temp = marker_array.markers.pop(0)
            marker_temp.header.stamp = rospy.Time(0)
            for i in range(len(cm.car_list)):
                if marker_temp.id == cm.car_list[i].car_id and cm.car_list[i].t[0] <= time*timestep_size:
                    for j in range(len(cm.car_list[i].t)):
                        if time*timestep_size == round(cm.car_list[i].t[j],2):
                            marker_temp.pose.position.x = cm.car_list[i].x[j]
                            marker_temp.pose.position.y = cm.car_list[i].y[j]
                            marker_temp.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, float(math.radians(cm.car_list[i].heading[j]))))
            marker_array.markers.append(marker_temp)
        pub.publish(marker_array)
        time = time + 1
        rate.sleep()

if __name__ == '__main__':
    main()