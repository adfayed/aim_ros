import numpy as np
import math

xs = [10, 10, 10]
ys = [10, 10, 10]
hs = [90, 90, 90]
car_l = 5
car_w = 2

# Calculate the initial bounding box
box = np.array([[[xs[0] - car_w],
				 [ys[0]],
				 [1]],
				[[xs[0] + car_w],
				 [ys[0]],
				 [1]],
				[[xs[0] - car_w],
				 [ys[0] - car_l],
				 [1]],
				[[xs[0] + car_w],
				 [ys[0] - car_l],
				 [1]]])
print box
print "\n\n"

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

print box
