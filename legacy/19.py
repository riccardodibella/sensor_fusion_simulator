import numpy as np
from math import pi, sin, cos, floor
from common import *
import random
import matplotlib.pyplot as plt

NUM_TRIES = 1

STEP = 4
num_vehicles = 25

random.seed(5)
np.random.seed(6)

cap_list = range(1,num_vehicles+1,STEP)

cm_spr_count = np.zeros((len(cap_list), NUM_TRIES))
omm_spr_count = np.zeros((len(cap_list), NUM_TRIES))
oma_spr_count = np.zeros((len(cap_list), NUM_TRIES))
cm_spr_alpha = np.zeros((len(cap_list), NUM_TRIES))
omm_spr_alpha = np.zeros((len(cap_list), NUM_TRIES))
oma_spr_alpha = np.zeros((len(cap_list), NUM_TRIES))

for t in range(NUM_TRIES):
	print(f"t={t}")
	road_map, vehicles, objects = gen_map(4, num_vehicles, 10, 10)
	# disp_road_matrix(road_map, vehicles, False)
	count_tuple_list = []
	for vehicle in vehicles:
		count_tuple_list += [(vehicle, generate_count_matrix(road_map, vehicle))]
	print("count matrices generated")

	for channel_capacity_index in range(len(cap_list)):
		channel_capacity = cap_list[channel_capacity_index]
		print(f"capacity={channel_capacity}")

		tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_SPREAD)
		print(".")
		merged_count = merge_count_matrices(tx, vehicles, MERGE_MODE_COUNT)
		cm_spr_count[channel_capacity_index, t] = count_metric(road_map, merged_count, vehicles)
		omm_spr_count[channel_capacity_index, t], oma_spr_count[channel_capacity_index, t] = object_metrics(merged_count, objects)
		print("..")
		merged_alpha = merge_count_matrices(tx, vehicles, MERGE_MODE_ALPHA)
		cm_spr_alpha[channel_capacity_index, t] = count_metric(road_map, merged_alpha, vehicles)
		omm_spr_alpha[channel_capacity_index, t], oma_spr_alpha[channel_capacity_index, t] = object_metrics(merged_alpha, objects)

		print("s")

plt.plot(cap_list, np.average(cm_spr_count, axis=1), label="count", color="blue")
plt.plot(cap_list, np.average(cm_spr_alpha, axis=1), label="alpha", color="red")
plt.xlabel("Channel capacity")
plt.ylabel("Error percentage")
plt.legend(loc='best')
plt.show()

plt.semilogy(cap_list, np.average(omm_spr_count, axis=1), label="count (min)", color="blue")
plt.semilogy(cap_list, np.average(oma_spr_count, axis=1), label="count (avg)", linestyle="dashdot", color="blue")
plt.semilogy(cap_list, np.average(omm_spr_alpha, axis=1), label="alpha (min)", color="red")
plt.semilogy(cap_list, np.average(oma_spr_alpha, axis=1), label="alpha (avg)", linestyle="dashdot", color="red")
plt.xlabel("Channel capacity")
plt.ylabel("Object probability")
plt.legend(loc='best')
plt.show()