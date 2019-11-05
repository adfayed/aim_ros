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