import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, NoNorm
import numpy as np
import json

ROAD_STATE_EMPTY = 0
ROAD_STATE_OCCUPIED = 1
ROAD_STATE_UNCLEAR = 2

EMP = ROAD_STATE_EMPTY
OCC = ROAD_STATE_OCCUPIED
UNC = ROAD_STATE_UNCLEAR

ROAD_STATE_COLOR_MAP = ListedColormap(["green", "red", "yellow"])

SENSOR_TYPE_DESCR_DICT = {1: "Standard LiDAR"}

def disp_road_matrix(mat, vehicles = None):
	# https://chatgpt.com/share/67a5ca7b-0c34-8007-beef-bdc41fcd1c19
	if(type(mat) is not np.ndarray):
		mat = np.array(mat) # in case it is not already np array
	plt.imshow(mat, cmap=ROAD_STATE_COLOR_MAP, norm=NoNorm(), interpolation="nearest")
	
	# Add grid lines explicitly aligned with cell boundaries
	num_rows, num_cols = mat.shape
	plt.gca().set_xticks(np.arange(-0.5, num_cols, 1), minor=True)
	plt.gca().set_yticks(np.arange(-0.5, num_rows, 1), minor=True)
	plt.gca().grid(which='minor', color='black', linestyle='-', linewidth=0.5)
	plt.gca().tick_params(which='minor', length=0)

	# Hide axes ticks
	plt.xticks([])
	plt.yticks([])

	if(vehicles != None):
		for v in vehicles:
			plt.plot(v.hpos - 0.5, v.vpos - 0.5, 'bo')

	# Show the plot
	plt.show()


class Vehicle:
	def __init__(self, vert, horiz, height, width, sensor_type, index):
		self.vpos = int(vert)
		self.hpos = int(horiz)
		self.htot = int(height)
		self.wtot = int(width)
		assert(height % 2 == 0 and width % 2 == 0)
		self.h2 = self.htot//2
		self.w2 = self.wtot//2
		self.sensor = int(sensor_type)
		self.index = int(index)
	def __repr__(self):
		return f"Vehicle #{self.index} ({self.vpos}, {self.hpos})"

def load_map(filename):
	road_map = np.loadtxt("road_01.csv", dtype='i', delimiter=',')
	vehicles = []
	with open(filename) as f:
		first_line = f.readline().strip('\n')
		first_line = first_line[1:] # remove the leading '#'
		vehicle_descr_list = json.loads(first_line)
		for index in range(len(vehicle_descr_list)):
			list_item = vehicle_descr_list[index]
			vehicles += [Vehicle(list_item["vert"], list_item["horiz"], list_item["height"], list_item["width"], list_item["sensor_type"], index)]

	return road_map, vehicles
