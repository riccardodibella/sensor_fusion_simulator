# Load matrix from txt https://stackoverflow.com/a/41491301

import numpy as np
from common import *

road_map = np.loadtxt("road_01.csv", dtype='i', delimiter=',')
disp_road_matrix(road_map)