import numpy as np
import random
from simulator_utils import *

random.seed(0)
np.random.seed(0)

NUSCENES_DIR = r".\\nuscenes\\v1.0-mini"
map_dims = (100, 100)

bev, space = nuscenes_load_bev_points(NUSCENES_DIR, 300, -1.1)
#scatter_3d(space)
scatter_bev(bev, dimensions=map_dims)

cm = generate_count_matrix_bev_points(bev, car_coords_m=(0,0), vehicle_radius_m=2, map_dimensions_m=map_dims, voxels_per_meter=4)
print("Empty")
print(cm[:,:,VOXEL_STATE_EMPTY])
print("Occupied")
print(cm[:,:,VOXEL_STATE_OCCUPIED])
print("Hidden")
print(cm[:,:,VOXEL_STATE_HIDDEN])

disp_count_matrix_slice(cm[:,:,VOXEL_STATE_EMPTY], False)
disp_count_matrix_slice(cm[:,:,VOXEL_STATE_OCCUPIED], False)
disp_count_matrix_slice(cm[:,:,VOXEL_STATE_HIDDEN], False)

v = Vehicle(0,0,0,0,1,0)
res = merge_count_matrices([(v, cm)], [v], MERGE_MODE_COUNT)
disp_prob_matrix(res, False)