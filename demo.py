import numpy as np
import random
from simulator_utils import *

random.seed(0)
np.random.seed(0)

road_map, vehicles, objects = gen_map(4, 10, 10, 10)

disp_road_matrix(road_map, vehicles, False)

count_tuple_list = []
for vehicle in vehicles:
	count_tuple_list += [(vehicle, generate_count_matrix(road_map, vehicle))]

channel_capacity = 5

for (strategy, name) in [(TRANSMISSION_STRATEGY_RANDOM, "Random"), (TRANSMISSION_STRATEGY_INC_DST, "Increasing Distance"), (TRANSMISSION_STRATEGY_SPREAD, "Spread")]:
    print(f"Transmission strategy {name}...")
    tx = apply_transmission_strategy(count_tuple_list, channel_capacity, strategy)
    merged = merge_count_matrices(tx, vehicles, MERGE_MODE_ALPHA)
    disp_prob_matrix(merged, False)
    print(f"[{name}] Count metric: {count_metric(road_map, merged, vehicles)}")
    print(f"[{name}] Object metrics: {object_metrics(merged, objects)}")
    print("-----------------")

channel_capacity = 1

print(f"Transmission strategy Closest...")
tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_CLOSEST)
merged = merge_count_matrices(tx, vehicles, MERGE_MODE_ALPHA)
disp_prob_matrix(merged, False)
print(f"[Closest] Count metric: {count_metric(road_map, merged, vehicles)}")
print(f"[Closest] Object metrics: {object_metrics(merged, objects)}")