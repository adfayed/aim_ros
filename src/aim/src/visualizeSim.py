#!/usr/bin/env python 
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import numpy as np
import math
import tf

def main(car_list, dMax, lane_width, timestep_size, end_time):
    dMax = dMax
    lane_width = lane_width
    car_list = car_list
    end_time = end_time

    #rospy.init_node('marker_publisher', disable_signals = True)
    pub = rospy.Publisher('aim_marker_array', MarkerArray, queue_size=100)
    rate = rospy.Rate(1/timestep_size)

    marker_array = MarkerArray()

    dotted_line_length = 3
    dotted_line = Marker()
    dotted_line.header.frame_id = "map"
    dotted_line.header.stamp = rospy.Time(0)
    dotted_line.id = 99998
    dotted_line.ns = "aim"
    dotted_line.type = Marker.LINE_LIST
    dotted_line.action = Marker.ADD
    dotted_line.scale.x = 0.4
    dotted_line.color.a = 1.0
    dotted_line.color.r = 1.0
    dotted_line.color.g = 1.0
    dotted_line.color.b = 0.0
    for l in range(0, int(2*dMax+6*lane_width), 2*dotted_line_length):
        if l + dotted_line_length < dMax or l > dMax+6*lane_width:
            dotted_line.points.append(Point(dMax + lane_width, l, 0)) # Lane N3
            dotted_line.points.append(Point(dMax + lane_width, l + dotted_line_length, 0)) 

            dotted_line.points.append(Point(dMax + 2*lane_width, l, 0)) # Lane N2
            dotted_line.points.append(Point(dMax + 2*lane_width, l + dotted_line_length, 0))
            
            dotted_line.points.append(Point(dMax + 4*lane_width, l, 0)) # Lane S1
            dotted_line.points.append(Point(dMax + 4*lane_width, l + dotted_line_length, 0))
            
            dotted_line.points.append(Point(dMax + 5*lane_width, l, 0)) # Lane S2
            dotted_line.points.append(Point(dMax + 5*lane_width, l + dotted_line_length, 0))  

            dotted_line.points.append(Point(l, dMax + lane_width, 0)) # Lane W3
            dotted_line.points.append(Point(l + dotted_line_length, dMax + lane_width, 0))

            dotted_line.points.append(Point(l, dMax + 2*lane_width, 0)) # Lane W2
            dotted_line.points.append(Point(l + dotted_line_length, dMax + 2*lane_width, 0))

            dotted_line.points.append(Point(l, dMax + 4*lane_width, 0)) # Lane E1
            dotted_line.points.append(Point(l + dotted_line_length, dMax + 4*lane_width, 0))

            dotted_line.points.append(Point(l, dMax + 5*lane_width, 0)) # Lane E2
            dotted_line.points.append(Point(l + dotted_line_length, dMax + 5*lane_width, 0))

    marker_array.markers.append(dotted_line)

    north_text = Marker()
    north_text.header.frame_id = "map"
    north_text.header.stamp = rospy.Time(0)
    north_text.id = 999996
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
    east_text.id = 999995
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
    west_text.id = 999994
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
    south_text.id = 999993
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

    south_stop_p1 = Point(dMax+6*lane_width, dMax, 0)
    south_stop_p2 = Point(dMax+3*lane_width, dMax, 0)

    west_stop_p1 = Point(dMax, dMax, 0)
    west_stop_p2 = Point(dMax, dMax+3*lane_width, 0)

    north_stop_p1 = Point(dMax, dMax+6*lane_width, 0)
    north_stop_p2 = Point(dMax+3*lane_width, dMax+6*lane_width, 0)

    east_stop_p1 = Point(dMax+6*lane_width, dMax+6*lane_width, 0)
    east_stop_p2 = Point(dMax+6*lane_width, dMax+3*lane_width, 0)

    stop_lines = Marker()
    stop_lines.header.frame_id = "map"
    stop_lines.header.stamp = rospy.Time(0)
    stop_lines.id = 999992
    stop_lines.ns = "aim"
    stop_lines.type = Marker.LINE_LIST
    stop_lines.action = Marker.ADD
    stop_lines.scale.x = 0.6
    stop_lines.color.a = 1.0
    stop_lines.color.r = 1.0
    stop_lines.color.g = 1.0
    stop_lines.color.b = 1.0
    stop_lines.points.append(south_stop_p1)
    stop_lines.points.append(south_stop_p2)
    stop_lines.points.append(west_stop_p1)
    stop_lines.points.append(west_stop_p2)
    stop_lines.points.append(north_stop_p1)
    stop_lines.points.append(north_stop_p2)
    stop_lines.points.append(east_stop_p1)
    stop_lines.points.append(east_stop_p2)
    marker_array.markers.append(stop_lines)

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

    middle_lines = Marker()
    middle_lines.header.frame_id = "map"
    middle_lines.header.stamp = rospy.Time(0)
    middle_lines.id = 999998
    middle_lines.ns = "aim"
    middle_lines.type = Marker.LINE_LIST
    middle_lines.action = Marker.ADD
    middle_lines.scale.x = 0.6
    middle_lines.color.a = 1.0
    middle_lines.color.r = 1.0
    middle_lines.color.g = 1.0
    middle_lines.color.b = 0.0
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
    road_bounds.id = 999999
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

    for i in car_list:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time(0)
        marker.id = i.car_id
        marker.ns = "aim"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = i.width
        marker.scale.y = i.length
        marker.scale.z = 1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, float(math.radians(i.heading[0]))))
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.mesh_resource = "package://aim/meshes/complete.stl"
        marker_array.markers.append(marker)
    viz_time = 0
    while not rospy.is_shutdown():
        for k in range(len(car_list)):
            marker_temp = marker_array.markers.pop(0)
            marker_temp.header.stamp = rospy.Time(0)
            for i in range(len(car_list)):
                if marker_temp.id == car_list[i].car_id and car_list[i].t[0] <= viz_time*timestep_size:
                    for j in range(len(car_list[i].t)):
                        if viz_time*timestep_size == round(car_list[i].t[j],2):
                            marker_temp.pose.position.x = car_list[i].x[j]
                            marker_temp.pose.position.y = car_list[i].y[j]
                            marker_temp.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, float(math.radians(car_list[i].heading[j]))))
                            if car_list[i].follow_car is not None:
                                marker_temp.color.r = 0.0 
                                marker_temp.color.b = 1.0
                        elif viz_time*timestep_size > round(car_list[i].t[-1],2):
                            marker_temp.pose.position.x = 0
                            marker_temp.pose.position.y = 0
            marker_array.markers.append(marker_temp)
        pub.publish(marker_array)
        viz_time = viz_time + 1
        if viz_time*timestep_size >= end_time:
            rospy.signal_shutdown("Visulization Complete")
        else:
            rate.sleep()

if __name__ == '__main__':
    main()