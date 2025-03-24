import numpy as np
from math import pi, sin, cos, floor
from common import *
import random
import matplotlib.pyplot as plt

random.seed(0)
np.random.seed(0)

road_map, vehicles, objects = gen_map(4, 25, 10, 10)
disp_road_matrix(road_map, vehicles, False)
count = generate_count_matrix(road_map, next(filter(lambda x: x.index == 3, vehicles)))
disp_count_matrix_slice(count[:,:,VOXEL_STATE_EMPTY], False)

