import numpy as np
from math import pi, sin, cos, floor
from common import *
import random
import matplotlib.pyplot as plt

RANDOM_TESTS = 10
NUM_TRIES = 1

STEP = 24

random.seed(3)
np.random.seed(4)

num_vehicles = 25

cap_list = range(1,num_vehicles+1,STEP)

cm_rng = np.zeros((len(cap_list), NUM_TRIES*RANDOM_TESTS))
omm_rng = np.zeros((len(cap_list), NUM_TRIES*RANDOM_TESTS))
oma_rng = np.zeros((len(cap_list), NUM_TRIES*RANDOM_TESTS))

cm_inc = np.zeros((len(cap_list), NUM_TRIES))
omm_inc = np.zeros((len(cap_list), NUM_TRIES))
oma_inc = np.zeros((len(cap_list), NUM_TRIES))

cm_spr = np.zeros((len(cap_list), NUM_TRIES))
omm_spr = np.zeros((len(cap_list), NUM_TRIES))
oma_spr = np.zeros((len(cap_list), NUM_TRIES))

cm_clo = np.zeros((len(cap_list), NUM_TRIES))
omm_clo = np.zeros((len(cap_list), NUM_TRIES))
oma_clo = np.zeros((len(cap_list), NUM_TRIES))

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
		"""
		for i in range(RANDOM_TESTS):
			tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_RANDOM)
			merged = merge_count_matrices(tx, vehicles, MERGE_MODE_COUNT)
			cm_rng[channel_capacity_index, t*RANDOM_TESTS+i] = count_metric(road_map, merged, vehicles)
			omm_rng[channel_capacity_index, t*RANDOM_TESTS+i], oma_rng[channel_capacity_index, t*RANDOM_TESTS+i] = object_metrics(merged, objects)

		tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_INC_DST)
		merged = merge_count_matrices(tx, vehicles, MERGE_MODE_COUNT)
		cm_inc[channel_capacity_index, t] = count_metric(road_map, merged, vehicles)
		omm_inc[channel_capacity_index, t], oma_inc[channel_capacity_index, t] = object_metrics(merged, objects)

		tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_SPREAD)
		merged = merge_count_matrices(tx, vehicles, MERGE_MODE_COUNT)
		cm_spr[channel_capacity_index, t] = count_metric(road_map, merged, vehicles)
		omm_spr[channel_capacity_index, t], oma_spr[channel_capacity_index, t] = object_metrics(merged, objects)
		"""
		tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_CLOSEST)
		"""
		merged_count = merge_count_matrices(tx, vehicles, MERGE_MODE_COUNT)
		disp_prob_matrix(merged_count, False)
		cm_clo[channel_capacity_index, t] = count_metric(road_map, merged_count, vehicles)
		omm_clo[channel_capacity_index, t], oma_clo[channel_capacity_index, t] = object_metrics(merged_count, objects)
		"""
		merged_alpha = merge_count_matrices(tx, vehicles, MERGE_MODE_ALPHA)
		disp_prob_matrix(merged_alpha, False)
"""
print("Count metric")
print("Random")
print(np.average(cm_rng, axis=1))
print("Increasing distance")
print(np.average(cm_inc, axis=1))
print("Spread")
print(np.average(cm_spr, axis=1))
print("Closest")
print(np.average(cm_clo, axis=1))
print("Object metric (min)")
print("Random")
print(np.average(omm_rng, axis=1))
print("Increasing distance")
print(np.average(omm_inc, axis=1))
print("Spread")
print(np.average(omm_spr, axis=1))
print("Closest")
print(np.average(omm_clo, axis=1))
print("Object metric (avg)")
print("Random")
print(np.average(oma_rng, axis=1))
print("Increasing distance")
print(np.average(oma_inc, axis=1))
print("Spread")
print(np.average(oma_spr, axis=1))
print("Closest")
print(np.average(oma_clo, axis=1))
plt.plot(cap_list, np.average(cm_rng, axis=1), label="Random")
plt.plot(cap_list, np.average(cm_inc, axis=1), label="Increasing distance")
plt.plot(cap_list, np.average(cm_spr, axis=1), label="Spread")
plt.plot(cap_list, np.average(cm_clo, axis=1), label="Closest")
plt.xlabel("Channel capacity")
plt.ylabel("Error percentage")
plt.legend(loc='best')
plt.show()

plt.semilogy(cap_list, np.average(omm_rng, axis=1), label="Random (min)")
plt.semilogy(cap_list, np.average(oma_rng, axis=1), label="Random (avg)")
plt.semilogy(cap_list, np.average(omm_inc, axis=1), label="Increasing distance (min)")
plt.semilogy(cap_list, np.average(oma_inc, axis=1), label="Increasing distance (avg)")
plt.semilogy(cap_list, np.average(omm_spr, axis=1), label="Spread (min)")
plt.semilogy(cap_list, np.average(oma_spr, axis=1), label="Spread (avg)")
plt.semilogy(cap_list, np.average(omm_clo, axis=1), label="Closest (min)")
plt.semilogy(cap_list, np.average(oma_clo, axis=1), label="Closest (avg)")
plt.xlabel("Channel capacity")
plt.ylabel("Object probability")
plt.legend(loc='best')
plt.show()"""