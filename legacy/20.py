import numpy as np
from math import pi, sin, cos, floor
from common import *
import random
import matplotlib.pyplot as plt

NUM_TRIES = 1

STEP = 4
num_vehicles = 25

random.seed(7)
np.random.seed(8)

cap_list = range(1,num_vehicles+1,STEP)

cm_spr_alpha = np.zeros((len(cap_list), NUM_TRIES))
omm_spr_alpha = np.zeros((len(cap_list), NUM_TRIES))
oma_spr_alpha = np.zeros((len(cap_list), NUM_TRIES))

cm_clo_alpha = np.zeros((len(cap_list), NUM_TRIES))
omm_clo_alpha = np.zeros((len(cap_list), NUM_TRIES))
oma_clo_alpha = np.zeros((len(cap_list), NUM_TRIES))

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
		merged_alpha = merge_count_matrices(tx, vehicles, MERGE_MODE_ALPHA)
		cm_spr_alpha[channel_capacity_index, t] = count_metric(road_map, merged_alpha, vehicles)
		omm_spr_alpha[channel_capacity_index, t], oma_spr_alpha[channel_capacity_index, t] = object_metrics(merged_alpha, objects)

		print("s")

		tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_CLOSEST)
		merged_alpha = merge_count_matrices(tx, vehicles, MERGE_MODE_ALPHA)
		cm_clo_alpha[channel_capacity_index, t] = count_metric(road_map, merged_alpha, vehicles)
		omm_clo_alpha[channel_capacity_index, t], oma_clo_alpha[channel_capacity_index, t] = object_metrics(merged_alpha, objects)

		print("c")

print("Count metric")
print("Spread")
print(np.average(cm_spr_alpha, axis=1))
print("Closest")
print(np.average(cm_clo_alpha, axis=1))
print("Object metric (min)")
print("Spread")
print(np.average(omm_spr_alpha, axis=1))
print("Closest")
print(np.average(omm_clo_alpha, axis=1))
print("Object metric (avg)")
print("Spread")
print(np.average(oma_spr_alpha, axis=1))
print("Closest")
print(np.average(oma_clo_alpha, axis=1))

plt.plot(cap_list, np.average(cm_spr_alpha, axis=1), label="Spread", color="red")
plt.plot(cap_list, np.average(cm_clo_alpha, axis=1), label="Closest")
plt.xlabel("Channel capacity")
plt.ylabel("Error percentage")
plt.legend(loc='best')
plt.tight_layout()
plt.show()

plt.semilogy(cap_list, np.average(omm_spr_alpha, axis=1), label="Spread (min)", color="red")
plt.semilogy(cap_list, np.average(oma_spr_alpha, axis=1), label="Spread (avg)", linestyle="dashdot", color="red")
plt.semilogy(cap_list, np.average(omm_clo_alpha, axis=1), label="Closest (min)", color="blue")
plt.semilogy(cap_list, np.average(oma_clo_alpha, axis=1), label="Closest (avg)", linestyle="dashdot", color="blue")
plt.xlabel("Channel capacity")
plt.ylabel("Object probability")
plt.legend(loc='best')
plt.tight_layout()
plt.show()