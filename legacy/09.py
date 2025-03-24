import numpy as np
from math import pi, sin, cos, floor
from common import *
import random
import matplotlib.pyplot as plt


road_map, vehicles = gen_map(4, 25, 10, 10)

disp_road_matrix(road_map, vehicles, False)

count_tuple_list = []
for vehicle in vehicles:
	count_tuple_list += [(vehicle, generate_count_matrix(road_map, vehicle))]
print("count matrices generated")
perm = np.random.permutation(range(len(count_tuple_list)))

x_arr = []
y_arr = []

for n in range(1, len(count_tuple_list)+1):
	print(f"n={n}")
	filtered_perm = perm[0:n]
	filtered = []
	for index in filtered_perm:
		filtered+=[count_tuple_list[index]]

	merged = merge_count_matrices(filtered, vehicles, MERGE_MODE_COUNT)
	c_m = count_metric(road_map, merged, vehicles)
	print(c_m)
	# disp_prob_matrix(merged, False)

	x_arr+=[n]
	y_arr+=[c_m]

plt.plot(x_arr, y_arr)
plt.show()
