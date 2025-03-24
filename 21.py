import numpy as np
from math import pi, sin, cos, floor
from common import *
import random
import matplotlib.pyplot as plt

random.seed(0)
np.random.seed(0)

road_map, vehicles, objects = gen_map(4, 25, 10, 10)
# disp_road_matrix(road_map, vehicles, False)

count_tuple_list = []

for vehicle in vehicles:
		count_tuple_list += [(vehicle, generate_count_matrix(road_map, vehicle))]

channel_capacity = 4

"""
tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_RANDOM)
merged = merge_count_matrices(tx, vehicles, MERGE_MODE_ALPHA)
disp_prob_matrix(merged, False)

tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_INC_DST)
merged = merge_count_matrices(tx, vehicles, MERGE_MODE_ALPHA)
disp_prob_matrix(merged, False)

tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_SPREAD)
merged = merge_count_matrices(tx, vehicles, MERGE_MODE_ALPHA)
disp_prob_matrix(merged, False)
"""

channel_capacity = 1

tx = apply_transmission_strategy(count_tuple_list, channel_capacity, TRANSMISSION_STRATEGY_CLOSEST)
merged = merge_count_matrices(tx, vehicles, MERGE_MODE_ALPHA)
disp_prob_matrix(merged, False)