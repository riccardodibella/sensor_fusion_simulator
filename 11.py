import numpy as np
from math import pi, sin, cos, floor
from common import *
import random
import matplotlib.pyplot as plt

RANDOM_TESTS = 5
"""
random.seed(1)
np.random.seed(2)
"""

road_map, vehicles, objects = gen_map(4, 10, 10, 10)

# disp_road_matrix(road_map, vehicles, False)

count_tuple_list = []
for vehicle in vehicles:
	count_tuple_list += [(vehicle, generate_count_matrix(road_map, vehicle))]
print("count matrices generated")

cm_rng_arr = []
omm_rng_arr = []
oma_rng_arr = []
for r in range(RANDOM_TESTS):
	tx = apply_transmission_strategy(count_tuple_list, 5, TRANSMISSION_STRATEGY_RANDOM)
	merged = merge_count_matrices(tx, vehicles, MERGE_MODE_COUNT)
	cm_rng = count_metric(road_map, merged, vehicles)
	omm_rng, oma_rng = object_metrics(merged, objects)

	cm_rng_arr+=[cm_rng]
	omm_rng_arr+=[omm_rng]
	oma_rng_arr+=[oma_rng]

cm_rng_avg = np.mean(cm_rng_arr)
omm_rng_avg = np.mean(omm_rng_arr)
oma_rng_avg = np.mean(oma_rng_arr)

print("random")
print(cm_rng_avg, omm_rng_avg, oma_rng_avg)

tx = apply_transmission_strategy(count_tuple_list, 5, TRANSMISSION_STRATEGY_CLOSEST)
merged = merge_count_matrices(tx, vehicles, MERGE_MODE_COUNT)
cm_clo = count_metric(road_map, merged, vehicles)
omm_clo, oma_clo = object_metrics(merged, objects)

print("closest")
print(cm_clo, omm_clo, oma_clo)