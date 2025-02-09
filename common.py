import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, NoNorm
import numpy as np
import json
import random

ROAD_STATE_EMPTY = 0
ROAD_STATE_OCCUPIED = 1
ROAD_STATE_UNCLEAR = 2

EMP = ROAD_STATE_EMPTY
OCC = ROAD_STATE_OCCUPIED
UNC = ROAD_STATE_UNCLEAR

ROAD_STATE_COLOR_MAP = ListedColormap(["green", "red", "yellow"])

SENSOR_TYPE_DESCR_DICT = {1: "Standard LiDAR"}

DIR_N, DIR_NE, DIR_E, DIR_SE, DIR_S, DIR_SW, DIR_W, DIR_NW = (1, 2, 3, 4, 5, 6, 7, 8)

VOXEL_STATE_EMPTY = 0
VOXEL_STATE_OCCUPIED = 1
VOXEL_STATE_HIDDEN = 2

PROB_LIDAR_HIT = {1: {ROAD_STATE_EMPTY: 0.002, ROAD_STATE_OCCUPIED: 0.9, ROAD_STATE_UNCLEAR: 0.4}} # first index is sensor type, second index is road state
SENSOR_ALPHA = {1: 0.9}

MERGE_MODE_COUNT = 1
MERGE_MODE_ALPHA = 2

def disp_road_matrix(mat, vehicles = None, show_grid = True):
	# https://chatgpt.com/share/67a5ca7b-0c34-8007-beef-bdc41fcd1c19
	if(type(mat) is not np.ndarray):
		mat = np.array(mat) # in case it is not already np array
	plt.imshow(mat, cmap=ROAD_STATE_COLOR_MAP, norm=NoNorm(), interpolation="nearest")
	
	if show_grid:
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
			plt.scatter(v.hpos - 0.5, v.vpos - 0.5, color='blue', s=40, marker=f"${v.index}$")

	# Show the plot
	plt.show()

def disp_count_matrix_slice(matrix, show_grid = True):
	plt.imshow(matrix, cmap='viridis', interpolation='nearest')
	if show_grid:
		num_rows, num_cols = matrix.shape
		plt.gca().set_xticks(np.arange(-0.5, num_cols, 1), minor=True)
		plt.gca().set_yticks(np.arange(-0.5, num_rows, 1), minor=True)
		plt.gca().grid(which='minor', color='black', linestyle='-', linewidth=0.5)
		plt.gca().tick_params(which='minor', length=0)
	plt.xticks([])
	plt.yticks([])
	plt.show()

def disp_prob_matrix(matrix, show_grid = True):
	num_rows, num_cols, num_states = matrix.shape
	for r in range(num_rows):
		for c in range(num_cols):
			for st in range(num_states):
				if(matrix[r,c,st] < 0):
					matrix[r,c,st] = 0
				elif(matrix[r,c,st] > 1):
					matrix[r,c,st] = 1
	plt.imshow(matrix, interpolation='nearest')
	if show_grid:
		
		plt.gca().set_xticks(np.arange(-0.5, num_cols, 1), minor=True)
		plt.gca().set_yticks(np.arange(-0.5, num_rows, 1), minor=True)
		plt.gca().grid(which='minor', color='black', linestyle='-', linewidth=0.5)
		plt.gca().tick_params(which='minor', length=0)
	plt.xticks([])
	plt.yticks([])
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
	# Load matrix from txt https://stackoverflow.com/a/41491301
	road_map = np.loadtxt(filename, dtype='i', delimiter=',')
	vehicles = []
	with open(filename) as f:
		first_line = f.readline().strip('\n')
		first_line = first_line[1:] # remove the leading '#'
		vehicle_descr_list = json.loads(first_line)
		for index in range(len(vehicle_descr_list)):
			list_item = vehicle_descr_list[index]
			vehicles += [Vehicle(list_item["vert"], list_item["horiz"], list_item["height"], list_item["width"], list_item["sensor_type"], index)]

	return road_map, vehicles


def gen_map(voxels_per_meter = 4, num_vehicles = 20, num_side_obstacles = 0, road_length_m = 100, building_spacing_m = 20, building_unc_border_m = 0):
	dim = road_length_m * voxels_per_meter
	building_spacing_voxels = building_spacing_m * voxels_per_meter
	building_unc_border_voxels = building_unc_border_m * voxels_per_meter

	mat = np.zeros((dim, dim), dtype='i')
	vehicles = []

	# buildings
	for v in range(dim):
		for h in range(dim):
			if (v < (dim / 2 - building_spacing_voxels / 2) or v >= (dim / 2 + building_spacing_voxels / 2)) and (h < (dim / 2 - building_spacing_voxels / 2) or h >= (dim / 2 + building_spacing_voxels / 2)):
				mat[v, h] = ROAD_STATE_OCCUPIED
			elif (v < (dim / 2 - building_spacing_voxels / 2 + building_unc_border_voxels) or v >= (dim / 2 + building_spacing_voxels / 2 - building_unc_border_voxels)) and (h < (dim / 2 - building_spacing_voxels / 2 + building_unc_border_voxels) or h >= (dim / 2 + building_spacing_voxels / 2 - building_unc_border_voxels)):
				mat[v, h] = ROAD_STATE_UNCLEAR

	# vehicles
	vehicle_dim_long = 3
	vehicle_dim_short = 2
	slot_dim_long = 5
	slot_dim_short = 4
	padding_long = 1
	padding_short = 1
	slots_per_branch = ((road_length_m / 2)-slot_dim_long) / slot_dim_long

	num_slots = int(slots_per_branch*8+4)

	slot_assign_list = [-1 for i in range(num_slots)]
	for i in range(num_vehicles):
		chosen = random.randint(0, len(slot_assign_list)-1)
		while slot_assign_list[chosen] != -1:
			chosen = random.randint(0, len(slot_assign_list)-1)
		slot_assign_list[chosen] = i

	for i in range(num_slots):
		if(slot_assign_list[i] == -1):
			continue

		if(i < 2*slots_per_branch):
			# North branches
			# if the number is even the vehicle will be on the left, if odd on the right
			h_offs = -4 if i % 2 == 0 else 0

			v_pos_index = i // 2
			v_offs = 5 * v_pos_index

			start_v = 0
			start_h = dim//2
			for v in range(vehicle_dim_long*voxels_per_meter):
				for h in range(vehicle_dim_short*voxels_per_meter):
					mat[int(start_v + v_offs * voxels_per_meter + padding_long*voxels_per_meter + v), int(start_h + h_offs * voxels_per_meter + padding_short*voxels_per_meter + h)] = ROAD_STATE_OCCUPIED
			vehicles += [ 
				Vehicle(
					start_v + v_offs * voxels_per_meter + padding_long*voxels_per_meter + (vehicle_dim_long*voxels_per_meter)/2, 
					start_h + h_offs * voxels_per_meter + padding_short*voxels_per_meter + (vehicle_dim_short*voxels_per_meter)/2,
					vehicle_dim_long*voxels_per_meter,
					vehicle_dim_short*voxels_per_meter,
					1,
					slot_assign_list[i]
				)
			]
		elif(i < 4*slots_per_branch):
			# West branches
			# if the number is even the vehicle will be on the top, if odd on the bottom
			v_offs = -4 if i % 2 == 0 else 0

			h_pos_index = (i-2*slots_per_branch) // 2
			h_offs = 5 * h_pos_index

			start_h = 0
			start_v = dim//2
			for h in range(vehicle_dim_long*voxels_per_meter):
				for v in range(vehicle_dim_short*voxels_per_meter):
					mat[int(start_v + v_offs * voxels_per_meter + padding_short*voxels_per_meter + v), int(start_h + h_offs * voxels_per_meter + padding_long*voxels_per_meter + h)] = ROAD_STATE_OCCUPIED
			vehicles += [ 
				Vehicle(
					start_v + v_offs * voxels_per_meter + padding_short*voxels_per_meter + (vehicle_dim_short*voxels_per_meter)/2, 
					start_h + h_offs * voxels_per_meter + padding_long*voxels_per_meter + (vehicle_dim_long*voxels_per_meter)/2,
					vehicle_dim_short*voxels_per_meter,
					vehicle_dim_long*voxels_per_meter,
					1,
					slot_assign_list[i]
				)
			]
		elif(i < 6*slots_per_branch):
			# South branches
			# if the number is even the vehicle will be on the left, if odd on the right
			h_offs = -4 if i % 2 == 0 else 0

			v_pos_index = (i-4*slots_per_branch) // 2
			v_offs = 5 * v_pos_index

			start_v = dim//2 + slot_dim_long*voxels_per_meter
			start_h = dim//2
			for v in range(vehicle_dim_long*voxels_per_meter):
				for h in range(vehicle_dim_short*voxels_per_meter):
					mat[int(start_v + v_offs * voxels_per_meter + padding_long*voxels_per_meter + v), int(start_h + h_offs * voxels_per_meter + padding_short*voxels_per_meter + h)] = ROAD_STATE_OCCUPIED
			vehicles += [ 
				Vehicle(
					start_v + v_offs * voxels_per_meter + padding_long*voxels_per_meter + (vehicle_dim_long*voxels_per_meter)/2, 
					start_h + h_offs * voxels_per_meter + padding_short*voxels_per_meter + (vehicle_dim_short*voxels_per_meter)/2,
					vehicle_dim_long*voxels_per_meter,
					vehicle_dim_short*voxels_per_meter,
					1,
					slot_assign_list[i]
				)
			]
		elif(i < 8*slots_per_branch):
			# East branches
			# if the number is even the vehicle will be on the top, if odd on the bottom
			v_offs = -4 if i % 2 == 0 else 0

			h_pos_index = (i-6*slots_per_branch) // 2
			h_offs = 5 * h_pos_index

			start_h = dim//2 + slot_dim_long*voxels_per_meter
			start_v = dim//2
			for h in range(vehicle_dim_long*voxels_per_meter):
				for v in range(vehicle_dim_short*voxels_per_meter):
					mat[int(start_v + v_offs * voxels_per_meter + padding_short*voxels_per_meter + v), int(start_h + h_offs * voxels_per_meter + padding_long*voxels_per_meter + h)] = ROAD_STATE_OCCUPIED
			vehicles += [ 
				Vehicle(
					start_v + v_offs * voxels_per_meter + padding_short*voxels_per_meter + (vehicle_dim_short*voxels_per_meter)/2, 
					start_h + h_offs * voxels_per_meter + padding_long*voxels_per_meter + (vehicle_dim_long*voxels_per_meter)/2,
					vehicle_dim_short*voxels_per_meter,
					vehicle_dim_long*voxels_per_meter,
					1,
					slot_assign_list[i]
				)
			]
		else:
			start_h = dim//2
			start_v = dim//2
			h_offs = -4 if i % 2 == 0 else 0
			v_offs = -4 if (i // 2) % 2 == 0 else 0

			direction = random.randint(0,1)

			if direction == 0:
				for h in range(vehicle_dim_long*voxels_per_meter):
					for v in range(vehicle_dim_short*voxels_per_meter):
						mat[int(start_v + v_offs * voxels_per_meter + padding_short*voxels_per_meter + v), int(start_h + h_offs * voxels_per_meter + padding_long*voxels_per_meter + h)] = ROAD_STATE_OCCUPIED
				vehicles += [ 
					Vehicle(
						start_v + v_offs * voxels_per_meter + padding_short*voxels_per_meter + (vehicle_dim_short*voxels_per_meter)/2, 
						start_h + h_offs * voxels_per_meter + padding_long*voxels_per_meter + (vehicle_dim_long*voxels_per_meter)/2,
						vehicle_dim_short*voxels_per_meter,
						vehicle_dim_long*voxels_per_meter,
						1,
						slot_assign_list[i]
					)
				]
			else:
				for v in range(vehicle_dim_long*voxels_per_meter):
					for h in range(vehicle_dim_short*voxels_per_meter):
						mat[int(start_v + v_offs * voxels_per_meter + padding_long*voxels_per_meter + v), int(start_h + h_offs * voxels_per_meter + padding_short*voxels_per_meter + h)] = ROAD_STATE_OCCUPIED
				vehicles += [ 
					Vehicle(
						start_v + v_offs * voxels_per_meter + padding_long*voxels_per_meter + (vehicle_dim_long*voxels_per_meter)/2, 
						start_h + h_offs * voxels_per_meter + padding_short*voxels_per_meter + (vehicle_dim_short*voxels_per_meter)/2,
						vehicle_dim_long*voxels_per_meter,
						vehicle_dim_short*voxels_per_meter,
						1,
						slot_assign_list[i]
					)
				]

	# side obstacles
	voxel_margin = 2
	for v in range(dim):
		for h in range(dim):
			if 	( \
					(h >= dim / 2 - building_spacing_voxels / 2 + building_unc_border_voxels and h < (road_length_m/2 - slot_dim_short)*voxels_per_meter) or (( h >= (road_length_m/2 + slot_dim_short)*voxels_per_meter) and (h < dim / 2 + building_spacing_voxels / 2 - building_unc_border_voxels)) \
					or \
					(v >= dim / 2 - building_spacing_voxels / 2 + building_unc_border_voxels and v < (road_length_m/2 - slot_dim_short)*voxels_per_meter) or (( v >= (road_length_m/2 + slot_dim_short)*voxels_per_meter) and (v < dim / 2 + building_spacing_voxels / 2 - building_unc_border_voxels)) \
				) and not ((h >= (road_length_m/2 - slot_dim_short)*voxels_per_meter and h < (road_length_m/2 + slot_dim_short)*voxels_per_meter) or (v >= (road_length_m/2 - slot_dim_short)*voxels_per_meter and v < (road_length_m/2 + slot_dim_short)*voxels_per_meter)):
				mat[v, h] = -1

	for n in range(num_side_obstacles):
		width_m = random.randrange(1,5)
		height_m = random.randrange(1,5)
		width = width_m*voxels_per_meter
		height = height_m*voxels_per_meter
		while True:
			h_start = random.randrange(voxel_margin, dim - width - voxel_margin)
			v_start = random.randrange(voxel_margin, dim - height - voxel_margin)
			h_end = h_start + width
			v_end = v_start + height
			ok = True
			for v in range(v_start-voxel_margin, v_end+voxel_margin):
				for h in range(h_start-voxel_margin, h_end+voxel_margin):
					if mat[v, h] != -1:
						ok = False
			if(ok):
				break
		for v in range(v_start, v_end):
			for h in range(h_start, h_end):
				mat[v, h] = ROAD_STATE_OCCUPIED

	for v in range(dim):
		for h in range(dim):
			if mat[v, h] == -1:
				mat[v, h] = ROAD_STATE_EMPTY

	return mat, vehicles