# Load matrix from txt https://stackoverflow.com/a/41491301

import numpy as np
from common import *

road_map, vehicles = load_map("road_02.csv")
disp_road_matrix(road_map, vehicles)
