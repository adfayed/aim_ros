import numpy as np
import math

"""
Used testing the IntersectionManagerClass

main():
gsz = 1
isz = 74
dMax = 25
dMin = 12
timestep = 1
policy = 0

car1 = Car(1, 0, 0, 27, 74, 180, 6, 4, 2, 1, -1)
# car2 = Car(2, 0.1, 15, 2, 3)
# car3 = Car(3, 0.2, 15, 2, 3)
success1, xs1, ys1, hs1, vs1, ts1 = IM.handle_car_request(car1)
# success2 = IM.handle_car_request(car2)
# success3 = IM.handle_car_request(car3)
print(success1)
print(xs1)
print(ys1)
print(hs1)
print(vs1)
print(ts1)
# print "Car1: %s\nCar2: %s\nCar3: %s" % (success1, success2, success3)

calculateTrajectories():
if car.car_id == 1:
	xs = [6, 7, 8]
	ys = [7, 7, 7]
	hs = [90, 90, 90]
	vs = [desired_velo, desired_velo, desired_velo]
	ts = [car.t, car.t + 0.1, car.t + 0.2]
elif car.car_id == 2:
	xs = [9, 9, 9]
	ys = [5, 6, 7]
	hs = [0, 0, 0]
	vs = [desired_velo, desired_velo, desired_velo]
	ts = [car.t, car.t + 0.1, car.t + 0.2]
else:
	xs = [2, 3, 4]
	ys = [7, 7, 7]
	hs = [90, 90, 90]
	vs = [desired_velo, desired_velo, desired_velo]
	ts = [car.t, car.t + 0.1, car.t + 0.2]
return xs, ys, hs, vs, ts
"""

"""
Testing with matplotlib visualization
car1 = Car(1, 1, 0.0, dMax + 6, isz, 180, 5.0, 5.0, 4, 2, 1, -1)
	car2 = Car(2, 10, 0.8, 0, dMax + 6, 90, 5.0, 5.0, 4, 2, 1, -1)
	success1, xs1, ys1, hs1, vs1, ts1 = IM.handle_car_request(car1)
	success2, xs2, ys2, hs2, vs2, ts2 = IM.handle_car_request(car2)
	time_2_stop1 = (dMax * 2) / car1.vel
	accel1 = -car1.vel / time_2_stop1
	time_2_stop2 = (dMax * 2) / car2.vel
	accel2 = -car2.vel / time_2_stop2
	actual_x1 = []
	actual_y1 = []
	actual_h1 = []
	actual_v1 = []
	actual_t1 = []
	actual_x2 = []
	actual_y2 = []
	actual_h2 = []
	actual_v2 = []
	actual_t2 = []
	while True:
		if not success1:
			actual_x1.append(car1.x)
			actual_y1.append(car1.y)
			actual_h1.append(car1.heading)
			actual_v1.append(car1.vel)
			actual_t1.append(car1.t)
			car1.y = car1.y - ((0.5 * accel1 * timestep**2) + (car1.vel * timestep))
			car1.vel = car1.vel + (accel1 * timestep)
			if car1.vel <= 0:
				car1.vel = 0
				accel1 = 0
				car1.y = isz - dMax
			car1.t = np.around(car1.t + timestep, decimals=10)
			#print "Timestep: %s\nvelo: %s"%(car1.t, car1.vel)
			success1, xs1, ys1, hs1, vs1, ts1 = IM.handle_car_request(car1)
		if not success2:
			#visualize(isz, dMax, dMin, [car2.x], [car2.y], [car2.heading], [car2], 1, False)
			actual_x2.append(car2.x)
			actual_y2.append(car2.y)
			actual_h2.append(car2.heading)
			actual_v2.append(car2.vel)
			actual_t2.append(car2.t)
			car2.x = car2.x + ((0.5 * accel2 * timestep**2) + (car2.vel * timestep))
			car2.vel = car2.vel + (accel2 * timestep)
			if car2.vel <= 0:
				car2.vel = 0
				accel2 = 0
				car2.x = dMax
			car2.t = np.around(car2.t + timestep, decimals=10)
			#print "Timestep: %s\nvelo: %s"%(car1.t, car1.vel)
			success2, xs2, ys2, hs2, vs2, ts2 = IM.handle_car_request(car2)
		if success1 and success2:
			break

	for i in range(len(xs1)):
		actual_x1.append(xs1[i])
		actual_y1.append(ys1[i])
		actual_h1.append(hs1[i])
		actual_v1.append(vs1[i])
		actual_t1.append(ts1[i])
	for i in range(len(xs2)):
		actual_x2.append(xs2[i])
		actual_y2.append(ys2[i])
		actual_h2.append(hs2[i])
		actual_v2.append(vs2[i])
		actual_t2.append(ts2[i])
	for t in range(len(actual_t1)):
		actual_t1[t] = np.around(actual_t1[t], decimals=5)
	for t in range(len(actual_t2)):
		actual_t2[t] = np.around(actual_t2[t], decimals=5)

	time = 0.0
	car1pointer = 0
	car2pointer = 0
	while True:
		x2print = []
		y2print = []
		h2print = []
		v2print = []
		car2print = []
		colors = []
		if car1pointer < len(actual_t1) and actual_t1[car1pointer] == time:
			x2print.append(actual_x1[car1pointer])
			y2print.append(actual_y1[car1pointer])
			h2print.append(actual_h1[car1pointer])
			v2print.append(actual_v1[car1pointer])
			car2print.append(car1)
			colors.append('-r')
			car1pointer += 1
		if car2pointer < len(actual_t2) and actual_t2[car2pointer] == time:
			x2print.append(actual_x2[car2pointer])
			y2print.append(actual_y2[car2pointer])
			h2print.append(actual_h2[car2pointer])
			v2print.append(actual_v2[car2pointer])
			car2print.append(car2)
			colors.append('-b')
			car2pointer += 1
		visualize(isz, dMax, dMin, x2print, y2print, h2print, v2print, car2print, colors, 1, False)
		time += 0.1
		time = np.around(time, decimals=5)
		if car1pointer >= len(actual_t1) and car2pointer >= len(actual_t2):
			break

	# x2print = []
	# y2print = []
	# h2print = []
	# for i in range(len(actual_x1)):
	# 	x2print.append(actual_x1[i])
	# 	y2print.append(actual_y1[i])
	# 	h2print.append(actual_h1[i])
	# for i in range(len(actual_x2)):
	# 	x2print.append(actual_x2[i])
	# 	y2print.append(actual_y2[i])
	# 	h2print.append(actual_h2[i])
	# visualize(isz, dMax, dMin, x2print, y2print, h2print, [car1, car2], [], 1, True)
	# visualize(isz, dMax, dMin, actual_x2, actual_y2, car2, 1, True)


def visualize(isz, dMax, dMin, x, y, h, v, car, colors, fignum=1, whole_path=True):
	fig = plt.figure(fignum)
	plt.cla()
	# Plot outside boarders
	plt.plot((0, isz), (0, 0), 'k')
	plt.plot((isz, isz), (0, isz), 'k')
	plt.plot((0, isz), (isz, isz), 'k')
	plt.plot((0, 0), (0, isz), 'k')

	# Plot the boarders for the lanes
	# Horizontal lines
	plt.plot((0, dMax), (dMax, dMax), 'k')
	plt.plot((0, dMax), (isz - dMax, isz - dMax), 'k')
	plt.plot((isz - dMax, isz), (dMax, dMax), 'k')
	plt.plot((isz - dMax, isz), (isz - dMax, isz - dMax), 'k')
	# Vertical lines
	plt.plot((dMax, dMax), (0, dMax), 'k')
	plt.plot((isz - dMax, isz - dMax), (0, dMax), 'k')
	plt.plot((dMax, dMax), (isz - dMax, isz), 'k')
	plt.plot((isz - dMax, isz - dMax), (isz - dMax, isz), 'k')
	# Crosswalk lines
	plt.plot((dMax, dMax), (dMax, isz - dMax), 'y')
	plt.plot((dMax, isz - dMax), (dMax, dMax), 'y')
	plt.plot((dMax, isz - dMax), (isz - dMax, isz - dMax), 'y')
	plt.plot((isz - dMax, isz - dMax), (isz - dMax, dMax), 'y')

	# Lane Seperators
	# Vertical lines
	plt.plot((dMax + 4, dMax + 4), (0, dMax), '--y')
	plt.plot((dMax + 8, dMax + 8), (0, dMax), '--y')
	plt.plot((dMax + 12, dMax + 12), (0, dMax), '-y')
	plt.plot((dMax + 16, dMax + 16), (0, dMax), '--y')
	plt.plot((dMax + 20, dMax + 20), (0, dMax), '--y')
	plt.plot((dMax + 4, dMax + 4), (isz, isz - dMax), '--y')
	plt.plot((dMax + 8, dMax + 8), (isz, isz - dMax), '--y')
	plt.plot((dMax + 12, dMax + 12), (isz, isz - dMax), '-y')
	plt.plot((dMax + 16, dMax + 16), (isz, isz - dMax), '--y')
	plt.plot((dMax + 20, dMax + 20), (isz, isz - dMax), '--y')
	# Horizontal lines
	plt.plot((0, dMax), (dMax + 4, dMax + 4), '--y')
	plt.plot((0, dMax), (dMax + 8, dMax + 8),  '--y')
	plt.plot((0, dMax), (dMax + 12, dMax + 12), '-y')
	plt.plot((0, dMax), (dMax + 16, dMax + 16), '--y')
	plt.plot((0, dMax), (dMax + 20, dMax + 20), '--y')
	plt.plot((isz, isz - dMax), (dMax + 4, dMax + 4), '--y')
	plt.plot((isz, isz - dMax), (dMax + 8, dMax + 8), '--y')
	plt.plot((isz, isz - dMax), (dMax + 12, dMax + 12), '-y')
	plt.plot((isz, isz - dMax), (dMax + 16, dMax + 16), '--y')
	plt.plot((isz, isz - dMax), (dMax + 20, dMax + 20), '--y')

	# Plot the points
	if not whole_path:
		boxes = []
		for i in range(len(car)):
			boxes.append(calculateBox(car[i], [x[i]], [y[i]], [h[i]]))
			plt.annotate(v[i], xy=(x[i], y[i]), xytext=(x[i] - 15, y[i] + 15),
						 arrowprops=dict(facecolor='black', shrink=0.01))
		for b in range(len(boxes)):
			plt.plot((boxes[b][0][0][0], boxes[b][1][0][0], boxes[b][3][0][0], boxes[b][2][0][0], boxes[b][0][0][0]),
					 (boxes[b][0][1][0], boxes[b][1][1][0], boxes[b][3][1][0], boxes[b][2][1][0], boxes[b][0][1][0]),
					 colors[b])
		plt.pause(0.01)
	else:
		for i in range(len(x)):
			plt.plot(x[i], y[i], '-r')
		plt.show()


def calculateBox(car, xs, ys, hs):
	# Calculate the initial bounding box
		box = np.array([[[xs[0] - (car.width / 2)],
						 [ys[0]],
						 [1]],
						[[xs[0] + (car.width / 2)],
						 [ys[0]],
						 [1]],
						[[xs[0] - (car.width / 2)],
						 [ys[0] - car.length],
						 [1]],
						[[xs[0] + (car.width / 2)],
						 [ys[0] - car.length],
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
		return box


"""