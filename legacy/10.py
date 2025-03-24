import numpy as np
from math import pi, sin, cos, floor
from common import *
import random
import matplotlib.pyplot as plt

random.seed(1)
np.random.seed(2)

NUM_TRIES = 10
STEP = 4

x_arr = {}
y_arr = {}
y_arr_min = {}
y_arr_av = {}
for t in range(NUM_TRIES):
	print(f"t={t}")
	road_map, vehicles, objects = gen_map(4, 25, 10, 10)

	# disp_road_matrix(road_map, vehicles, False)

	count_tuple_list = []
	for vehicle in vehicles:
		count_tuple_list += [(vehicle, generate_count_matrix(road_map, vehicle))]
	print("count matrices generated")
	perm = np.random.permutation(range(len(count_tuple_list)))

	x_arr[t] = []
	y_arr[t] = []
	y_arr_min[t] = []
	y_arr_av[t] = []

	# for n in range(1, 10):
	for n in range(1, len(count_tuple_list)+1, STEP):
		print(f"n={n}")
		filtered_perm = perm[0:n]
		filtered = []
		for index in filtered_perm:
			filtered+=[count_tuple_list[index]]

		merged = merge_count_matrices(filtered, vehicles, MERGE_MODE_COUNT)
		c_m = count_metric(road_map, merged, vehicles)
		o_m_min, o_m_avg = object_metrics(merged, objects)
		print(c_m)
		print(o_m_min, o_m_avg)
		# disp_prob_matrix(merged, False)

		x_arr[t]+=[n]
		y_arr[t]+=[c_m]
		y_arr_min[t]+=[o_m_min]
		y_arr_av[t]+=[o_m_avg]

x_arr_avg = [0]*len(x_arr[0])
y_arr_avg = [0]*len(x_arr[0])
y_arr_min_avg = [0]*len(x_arr[0])
y_arr_av_avg = [0]*len(x_arr[0])
for t in range(NUM_TRIES):
	for n in range(len(range(1, len(count_tuple_list)+1, STEP))):
		x_arr_avg[n] += x_arr[t][n]
		y_arr_avg[n] += y_arr[t][n]
		y_arr_min_avg[n] += y_arr_min[t][n]
		y_arr_av_avg[n] += y_arr_av[t][n]
for n in range(len(range(1, len(count_tuple_list)+1, STEP))):
	x_arr_avg[n] /= NUM_TRIES
	y_arr_avg[n] /= NUM_TRIES
	y_arr_min_avg[n] /= NUM_TRIES
	y_arr_av_avg[n] /= NUM_TRIES

plt.plot(x_arr_avg, y_arr_avg)
plt.xlabel("Number of vehicles")
plt.ylabel("Error percentage")
plt.show()

plt.semilogy(x_arr_avg, y_arr_min_avg, label='Min')
plt.semilogy(x_arr_avg, y_arr_av_avg, label='Avg')
plt.xlabel("Number of vehicles")
plt.ylabel("Object probability")
plt.show()