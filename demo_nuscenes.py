import numpy as np
import random
from simulator_utils import *

random.seed(0)
np.random.seed(0)

NUSCENES_DIR = r".\\nuscenes\\v1.0-mini"

bev, space = nuscenes_load_bev_points(NUSCENES_DIR, 300, -1.1)
#scatter_3d(space)
#scatter_bev(bev)

cm = generate_count_matrix_bev_points(bev, (0,0), 2, (200, 200), 1)
print(cm[:,:,0])
print("")
print(cm[:,:,1])
print("")
print(cm[:,:,2])

disp_count_matrix_slice(cm[:,:,0], False)
disp_count_matrix_slice(cm[:,:,1], False)
disp_count_matrix_slice(cm[:,:,2], False)