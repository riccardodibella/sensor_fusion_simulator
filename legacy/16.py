import numpy as np
from math import pi, sin, cos, floor
from common import *
import random
import matplotlib.pyplot as plt

RANDOM_TESTS = 5
NUM_TRIES = 1

STEP = 24
num_vehicles = 25

random.seed(3)
np.random.seed(4)

cap_list = range(1,num_vehicles+1,STEP)

cm_rng_count = np.zeros((len(cap_list), NUM_TRIES*RANDOM_TESTS))
omm_rng_count = np.zeros((len(cap_list), NUM_TRIES*RANDOM_TESTS))
oma_rng_count = np.zeros((len(cap_list), NUM_TRIES*RANDOM_TESTS))
cm_rng_alpha = np.zeros((len(cap_list), NUM_TRIES*RANDOM_TESTS))
omm_rng_alpha = np.zeros((len(cap_list), NUM_TRIES*RANDOM_TESTS))
oma_rng_alpha = np.zeros((len(cap_list), NUM_TRIES*RANDOM_TESTS))

cm_inc_count = np.zeros((len(cap_list), NUM_TRIES))
omm_inc_count = np.zeros((len(cap_list), NUM_TRIES))
oma_inc_count = np.zeros((len(cap_list), NUM_TRIES))
cm_inc_alpha = np.zeros((len(cap_list), NUM_TRIES))
omm_inc_alpha = np.zeros((len(cap_list), NUM_TRIES))
oma_inc_alpha = np.zeros((len(cap_list), NUM_TRIES))

cm_spr_count = np.zeros((len(cap_list), NUM_TRIES))
omm_spr_count = np.zeros((len(cap_list), NUM_TRIES))
oma_spr_count = np.zeros((len(cap_list), NUM_TRIES))
cm_spr_alpha = np.zeros((len(cap_list), NUM_TRIES))
omm_spr_alpha = np.zeros((len(cap_list), NUM_TRIES))
oma_spr_alpha = np.zeros((len(cap_list), NUM_TRIES))

cm_clo_count = np.zeros((len(cap_list), NUM_TRIES))
omm_clo_count = np.zeros((len(cap_list), NUM_TRIES))
oma_clo_count = np.zeros((len(cap_list), NUM_TRIES))
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

		for i in range(RANDOM_TESTS):
			print(f"r{i}")
			tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_RANDOM)
			merged_count = merge_count_matrices(tx, vehicles, MERGE_MODE_COUNT)
			cm_rng_count[channel_capacity_index, t*RANDOM_TESTS+i] = count_metric(road_map, merged_count, vehicles)
			omm_rng_count[channel_capacity_index, t*RANDOM_TESTS+i], oma_rng_count[channel_capacity_index, t*RANDOM_TESTS+i] = object_metrics(merged_count, objects)
			merged_alpha = merge_count_matrices(tx, vehicles, MERGE_MODE_ALPHA)
			cm_rng_alpha[channel_capacity_index, t*RANDOM_TESTS+i] = count_metric(road_map, merged_alpha, vehicles)
			omm_rng_alpha[channel_capacity_index, t*RANDOM_TESTS+i], oma_rng_alpha[channel_capacity_index, t*RANDOM_TESTS+i] = object_metrics(merged_alpha, objects)

		print("r")

		tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_INC_DST)
		merged_count = merge_count_matrices(tx, vehicles, MERGE_MODE_COUNT)
		cm_inc_count[channel_capacity_index, t] = count_metric(road_map, merged_count, vehicles)
		omm_inc_count[channel_capacity_index, t], oma_inc_count[channel_capacity_index, t] = object_metrics(merged_count, objects)
		merged_alpha = merge_count_matrices(tx, vehicles, MERGE_MODE_ALPHA)
		cm_inc_alpha[channel_capacity_index, t] = count_metric(road_map, merged_alpha, vehicles)
		omm_inc_alpha[channel_capacity_index, t], oma_inc_alpha[channel_capacity_index, t] = object_metrics(merged_alpha, objects)

		print("i")

		tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_SPREAD)
		merged_count = merge_count_matrices(tx, vehicles, MERGE_MODE_COUNT)
		cm_spr_count[channel_capacity_index, t] = count_metric(road_map, merged_count, vehicles)
		omm_spr_count[channel_capacity_index, t], oma_spr_count[channel_capacity_index, t] = object_metrics(merged_count, objects)
		merged_alpha = merge_count_matrices(tx, vehicles, MERGE_MODE_ALPHA)
		cm_spr_alpha[channel_capacity_index, t] = count_metric(road_map, merged_alpha, vehicles)
		omm_spr_alpha[channel_capacity_index, t], oma_spr_alpha[channel_capacity_index, t] = object_metrics(merged_alpha, objects)

		print("s")

		tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_CLOSEST)
		merged_count = merge_count_matrices(tx, vehicles, MERGE_MODE_COUNT)
		cm_clo_count[channel_capacity_index, t] = count_metric(road_map, merged_count, vehicles)
		omm_clo_count[channel_capacity_index, t], oma_clo_count[channel_capacity_index, t] = object_metrics(merged_count, objects)
		merged_alpha = merge_count_matrices(tx, vehicles, MERGE_MODE_ALPHA)
		cm_clo_alpha[channel_capacity_index, t] = count_metric(road_map, merged_alpha, vehicles)
		omm_clo_alpha[channel_capacity_index, t], oma_clo_alpha[channel_capacity_index, t] = object_metrics(merged_alpha, objects)

		print("c")
print("Count metric")
print("Random")
print(np.average(cm_rng_count, axis=1))
print(np.average(cm_rng_alpha, axis=1))
print("Increasing distance")
print(np.average(cm_inc_count, axis=1))
print(np.average(cm_inc_alpha, axis=1))
print("Spread")
print(np.average(cm_spr_count, axis=1))
print(np.average(cm_spr_alpha, axis=1))
print("Closest")
print(np.average(cm_clo_count, axis=1))
print(np.average(cm_clo_alpha, axis=1))
print("Object metric (min)")
print("Random")
print(np.average(omm_rng_count, axis=1))
print(np.average(omm_rng_alpha, axis=1))
print("Increasing distance")
print(np.average(omm_inc_count, axis=1))
print(np.average(omm_inc_alpha, axis=1))
print("Spread")
print(np.average(omm_spr_count, axis=1))
print(np.average(omm_spr_alpha, axis=1))
print("Closest")
print(np.average(omm_clo_count, axis=1))
print(np.average(omm_clo_alpha, axis=1))
print("Object metric (avg)")
print("Random")
print(np.average(oma_rng_count, axis=1))
print(np.average(oma_rng_alpha, axis=1))
print("Increasing distance")
print(np.average(oma_inc_count, axis=1))
print(np.average(oma_inc_alpha, axis=1))
print("Spread")
print(np.average(oma_spr_count, axis=1))
print(np.average(oma_spr_alpha, axis=1))
print("Closest")
print(np.average(oma_clo_count, axis=1))
print(np.average(oma_clo_alpha, axis=1))
plt.plot(cap_list, np.average(cm_rng_count, axis=1), label="[C] Random")
plt.plot(cap_list, np.average(cm_inc_count, axis=1), label="[C] Increasing distance")
plt.plot(cap_list, np.average(cm_spr_count, axis=1), label="[C] Spread")
plt.plot(cap_list, np.average(cm_clo_count, axis=1), label="[C] Closest")
plt.plot(cap_list, np.average(cm_rng_alpha, axis=1), label="[A] Random", linestyle="dashdot")
plt.plot(cap_list, np.average(cm_inc_alpha, axis=1), label="[A] Increasing distance", linestyle="dashdot")
plt.plot(cap_list, np.average(cm_spr_alpha, axis=1), label="[A] Spread", linestyle="dashdot")
plt.plot(cap_list, np.average(cm_clo_alpha, axis=1), label="[A] Closest", linestyle="dashdot")
plt.xlabel("Channel capacity")
plt.ylabel("Error percentage")
plt.legend(loc='best')
plt.show()

plt.semilogy(cap_list, np.average(omm_rng_count, axis=1), label="[C] Random (min)")
plt.semilogy(cap_list, np.average(oma_rng_count, axis=1), label="[C] Random (avg)")
plt.semilogy(cap_list, np.average(omm_inc_count, axis=1), label="[C] Increasing distance (min)")
plt.semilogy(cap_list, np.average(oma_inc_count, axis=1), label="[C] Increasing distance (avg)")
plt.semilogy(cap_list, np.average(omm_spr_count, axis=1), label="[C] Spread (min)")
plt.semilogy(cap_list, np.average(oma_spr_count, axis=1), label="[C] Spread (avg)")
plt.semilogy(cap_list, np.average(omm_clo_count, axis=1), label="[C] Closest (min)")
plt.semilogy(cap_list, np.average(oma_clo_count, axis=1), label="[C] Closest (avg)")
plt.semilogy(cap_list, np.average(omm_rng_alpha, axis=1), label="[A] Random (min)", linestyle="dashdot")
plt.semilogy(cap_list, np.average(oma_rng_alpha, axis=1), label="[A] Random (avg)", linestyle="dashdot")
plt.semilogy(cap_list, np.average(omm_inc_alpha, axis=1), label="[A] Increasing distance (min)", linestyle="dashdot")
plt.semilogy(cap_list, np.average(oma_inc_alpha, axis=1), label="[A] Increasing distance (avg)", linestyle="dashdot")
plt.semilogy(cap_list, np.average(omm_spr_alpha, axis=1), label="[A] Spread (min)", linestyle="dashdot")
plt.semilogy(cap_list, np.average(oma_spr_alpha, axis=1), label="[A] Spread (avg)", linestyle="dashdot")
plt.semilogy(cap_list, np.average(omm_clo_alpha, axis=1), label="[A] Closest (min)", linestyle="dashdot")
plt.semilogy(cap_list, np.average(oma_clo_alpha, axis=1), label="[A] Closest (avg)", linestyle="dashdot")
plt.xlabel("Channel capacity")
plt.ylabel("Object probability")
plt.legend(loc='best')
plt.show()