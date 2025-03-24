# ex common.py
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, NoNorm
import numpy as np
import json
import random
from math import pi, sin, cos, floor, hypot


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

PROB_LIDAR_HIT = {1: {ROAD_STATE_EMPTY: 0, ROAD_STATE_OCCUPIED: 1}}
# PROB_LIDAR_HIT = {1: {ROAD_STATE_EMPTY: 0.001, ROAD_STATE_OCCUPIED: 0.95, ROAD_STATE_UNCLEAR: 0.4}} # first index is sensor type, second index is road state
SENSOR_ALPHA = {1: 0.8}

MERGE_MODE_COUNT = 1
MERGE_MODE_ALPHA = 2

TRANSMISSION_STRATEGY_RANDOM = 1
TRANSMISSION_STRATEGY_INC_DST = 2
TRANSMISSION_STRATEGY_SPREAD = 3
TRANSMISSION_STRATEGY_CLOSEST = 4


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

def gen_map(voxels_per_meter = 4, num_vehicles = 20, num_side_obstacles = 0, num_road_obstacles = 0, road_length_m = 100, building_spacing_m = 20, building_unc_border_m = 0):
	dim = road_length_m * voxels_per_meter
	building_spacing_voxels = building_spacing_m * voxels_per_meter
	building_unc_border_voxels = building_unc_border_m * voxels_per_meter

	mat = np.zeros((dim, dim), dtype='i')
	vehicles = []
	objects = []

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
		# width_m = random.randrange(1,5)
		# height_m = random.randrange(1,5)
		width_m = 1
		height_m = 1
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
		new_obj_entry_border = []
		new_obj_entry_interior = []
		for v in range(v_start, v_end):
			for h in range(h_start, h_end):
				mat[v, h] = ROAD_STATE_OCCUPIED
				if v == v_start or v == v_end-1 or h == h_start or h == h_end-1:
					new_obj_entry_border += [(v,h)]
				else:
					new_obj_entry_interior += [(v,h)]
		objects+=[(new_obj_entry_border,new_obj_entry_interior)]

	for v in range(dim):
		for h in range(dim):
			if mat[v, h] == -1:
				mat[v, h] = ROAD_STATE_EMPTY

	# road obstacles
	for i in range(len(slot_assign_list)):
		if slot_assign_list[i] != -1:
			continue
		if(i < 2*slots_per_branch):
			# North branches
			# if the number is even the vehicle will be on the left, if odd on the right
			h_offs = -4 if i % 2 == 0 else 0

			v_pos_index = i // 2
			v_offs = 5 * v_pos_index

			start_v = 0
			start_h = dim//2
			for v in range(slot_dim_long*voxels_per_meter):
				for h in range(slot_dim_short*voxels_per_meter):
					mat[int(start_v + v_offs * voxels_per_meter + v), int(start_h + h_offs * voxels_per_meter + h)] = -1
		elif(i < 4*slots_per_branch):
			# West branches
			# if the number is even the vehicle will be on the top, if odd on the bottom
			v_offs = -4 if i % 2 == 0 else 0

			h_pos_index = (i-2*slots_per_branch) // 2
			h_offs = 5 * h_pos_index

			start_h = 0
			start_v = dim//2
			for h in range(slot_dim_long*voxels_per_meter):
				for v in range(slot_dim_short*voxels_per_meter):
					mat[int(start_v + v_offs * voxels_per_meter + v), int(start_h + h_offs * voxels_per_meter + h)] = -1
		elif(i < 6*slots_per_branch):
			# South branches
			# if the number is even the vehicle will be on the left, if odd on the right
			h_offs = -4 if i % 2 == 0 else 0

			v_pos_index = (i-4*slots_per_branch) // 2
			v_offs = 5 * v_pos_index

			start_v = dim//2 + slot_dim_long*voxels_per_meter
			start_h = dim//2
			for v in range(slot_dim_long*voxels_per_meter):
				for h in range(slot_dim_short*voxels_per_meter):
					mat[int(start_v + v_offs * voxels_per_meter + v), int(start_h + h_offs * voxels_per_meter + h)] = -1
		elif(i < 8*slots_per_branch):
			# East branches
			# if the number is even the vehicle will be on the top, if odd on the bottom
			v_offs = -4 if i % 2 == 0 else 0

			h_pos_index = (i-6*slots_per_branch) // 2
			h_offs = 5 * h_pos_index

			start_h = dim//2 + slot_dim_long*voxels_per_meter
			start_v = dim//2
			for h in range(slot_dim_long*voxels_per_meter):
				for v in range(slot_dim_short*voxels_per_meter):
					mat[int(start_v + v_offs * voxels_per_meter + v), int(start_h + h_offs * voxels_per_meter + h)] = -1
		else:
			start_h = dim//2
			start_v = dim//2
			h_offs = -4 if i % 2 == 0 else 0
			v_offs = -4 if (i // 2) % 2 == 0 else 0

			for v in range(slot_dim_long*voxels_per_meter):
					for h in range(slot_dim_long*voxels_per_meter):
						mat[int(start_v + v_offs * voxels_per_meter  + v), int(start_h + h_offs * voxels_per_meter + h)] = -1

	for n in range(num_road_obstacles):
		# width_m = random.randrange(1,5)
		# height_m = random.randrange(1,5)
		width_m = 1
		height_m = 1
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
		new_obj_entry_border = []
		new_obj_entry_interior = []
		for v in range(v_start, v_end):
			for h in range(h_start, h_end):
				mat[v, h] = ROAD_STATE_OCCUPIED
				if v == v_start or v == v_end-1 or h == h_start or h == h_end-1:
					new_obj_entry_border += [(v,h)]
				else:
					new_obj_entry_interior += [(v,h)]
		objects+=[(new_obj_entry_border,new_obj_entry_interior)]
	for v in range(dim):
		for h in range(dim):
			if mat[v, h] == -1:
				mat[v, h] = ROAD_STATE_EMPTY

	return mat, vehicles, objects

def coord_noise():
	# tiny noise to calculating rays on the border of a cell
	res = 0
	while res == 0:
		res = np.random.normal(0, 0.1)
	return res

eps = 0.01 # tiny deviation used to enter inside a cell and avoid problems when checking on the border

def is_point_inside_vehicle(point, vehicle):
	# point is (ver, hor)
	return 	point[0] >= vehicle.vpos-vehicle.h2 and point[0] < vehicle.vpos+vehicle.h2 and point[1] >= vehicle.hpos-vehicle.w2 and point[1] < vehicle.hpos+vehicle.w2
def is_point_on_vehicle_border(point, vehicle):
	# point is (ver, hor)
	return 	(is_point_inside_vehicle(point, vehicle)) and (point[0] == vehicle.vpos-vehicle.h2 or point[0] == vehicle.vpos+vehicle.h2 -1 or point[1] == vehicle.hpos-vehicle.w2 or point[1] == vehicle.hpos+vehicle.w2 -1)

def gen_hit(road_state, sensor_type):
	prob_hit = PROB_LIDAR_HIT[sensor_type][road_state]
	return np.random.random_sample() < prob_hit


def generate_count_matrix(m, vehicle):
	map_height, map_width = m.shape
	result_shape = (map_height, map_width, 3)
	result = np.zeros(result_shape, dtype='i')

	start_coord_hor = vehicle.hpos + coord_noise()
	start_coord_ver = vehicle.vpos + coord_noise()

	degrs = np.linspace(0,360,360*10)
	for deg in degrs:
		angle = deg * 2 * pi / 360
		v_increase = sin(angle) # positive if going downwards
		h_increase = cos(angle) # positive if going rightwards
		
		voxel_list = [] # list of tuples (h, v) to be traversed in order by the ray

		cur_hpos = start_coord_hor
		cur_vpos = start_coord_ver
		while(True):
			new_coords = (floor(cur_vpos), floor(cur_hpos))
			if(new_coords[0] < 0 or new_coords[1] < 0 or new_coords[0] >= map_height or new_coords[1] >= map_width):
				break
			voxel_list += [new_coords]
			
			# update current position
			# https://chatgpt.com/share/67a63e35-96bc-8007-8c1d-85598a92b8e3
			upper_limit = floor(cur_vpos)
			lower_limit = floor(cur_vpos)+1
			left_limit = floor(cur_hpos)
			right_limit = floor(cur_hpos)+1

			if(v_increase != 0):
				t_down = (lower_limit - cur_vpos)/v_increase
				t_up = (cur_vpos - upper_limit)/(-1*v_increase)
			else: 
				t_down = -1
				t_up = -1
			if(h_increase != 0):
				t_right = (right_limit - cur_hpos)/h_increase
				t_left = (cur_hpos - left_limit)/(-1*h_increase)
			else: 
				t_right = -1
				t_left = -1
			
			t_legal = []
			if(t_down > 0):
				t_legal += [t_down]
			if(t_right > 0):
				t_legal += [t_right]
			if(t_left > 0):
				t_legal += [t_left]
			if(t_up > 0):
				t_legal += [t_up]

			t = min(t_legal)
			cur_hpos += h_increase * (t+eps)
			cur_vpos += v_increase * (t+eps)

		ray_blocked = False
		for vpos, hpos in voxel_list:
			if(is_point_inside_vehicle((vpos, hpos), vehicle)):
				continue
			if ray_blocked:
				result[vpos, hpos, VOXEL_STATE_HIDDEN] += 1
			else:
				road_state = m[vpos, hpos]
				hit = gen_hit(road_state, vehicle.sensor)
				if hit:
					result[vpos, hpos, VOXEL_STATE_OCCUPIED] += 1
					ray_blocked = True
				else:
					result[vpos, hpos, VOXEL_STATE_EMPTY] += 1
	return result


def merge_count_matrices(tup_list, vehicles, merge_mode):
	res_shape = tup_list[0][1].shape
	result = np.ndarray(res_shape)
	for ver in range(res_shape[0]):
		for hor in range(res_shape[1]):
			v = False
			b = False
			for vehicle in vehicles:
				if(is_point_inside_vehicle((ver,hor), vehicle)):
					v = True
					if(is_point_on_vehicle_border((ver,hor), vehicle)):
						b = True
			if(v):
				if(b):
					# Occupied
					result[ver, hor, VOXEL_STATE_EMPTY] = 0
					result[ver, hor, VOXEL_STATE_OCCUPIED] = 1
					result[ver, hor, VOXEL_STATE_HIDDEN] = 0
				else:
					# Hidden
					result[ver, hor, VOXEL_STATE_EMPTY] = 0
					result[ver, hor, VOXEL_STATE_OCCUPIED] = 0
					result[ver, hor, VOXEL_STATE_HIDDEN] = 1
				continue
			alphas = []
			counts = []
			for tup in tup_list:
				vehicle = tup[0]
				mat = tup[1]
				alphas += [SENSOR_ALPHA[vehicle.sensor]]
				counts += [mat[ver, hor]]
			weights = []
			if(merge_mode == MERGE_MODE_COUNT):
				tot_count = sum(sum(counts[:]))
				for c in counts:
					if tot_count > 0:
						weights += [sum(c)/tot_count]
					else:
						weights += [1/len(counts)]
			elif(merge_mode == MERGE_MODE_ALPHA):
				tot_count = sum(sum(counts[:]))
				sum_inv_alpha_n = 0
				for i in range(len(counts)):
					c = counts[i]
					n = sum(c)
					alpha = alphas[i]
					alpha_n = alpha ** n
					inv_alpha_n = n*(1/alpha_n)
					sum_inv_alpha_n += inv_alpha_n
				for i in range(len(counts)):
					c = counts[i]
					n = sum(c)
					alpha = alphas[i]
					alpha_n = alpha ** n
					inv_alpha_n = n*(1/alpha_n)
					if tot_count > 0:
						weights += [inv_alpha_n/sum_inv_alpha_n]
					else:
						weights += [1/len(counts)]
			else:
				print("todo invalid merge mode")
				exit()

			for state in [VOXEL_STATE_EMPTY, VOXEL_STATE_OCCUPIED, VOXEL_STATE_HIDDEN]:
				prob = 0
				found = False
				for num_vehicle in range(len(alphas)):
					n = sum(counts[num_vehicle])
					if n > 0:
						found = True
						alpha_n = alphas[num_vehicle]**n
						prob += weights[num_vehicle] * ((1/3) * alpha_n + (1-alpha_n)*(counts[num_vehicle][state]/n))
				if not found:
					# if the angular resolution is low, some voxels may not be in the trajectory of any ray
					prob = 1/3
				result[ver, hor, state] = prob
	return result

def is_point_hidden(mat, v_point, h_point):
	if(mat[v_point, h_point] != ROAD_STATE_OCCUPIED):
		return False

	map_height, map_width = mat.shape
	hidden = True
	for v in [v_point-1, v_point, v_point+1]:
		for h in [h_point-1, h_point, h_point+1]:
			if(v < 0 or h < 0 or v >= map_height or h >= map_width):
				continue
			if(mat[v, h] != ROAD_STATE_OCCUPIED):
				hidden = False
				break
	return hidden

def inflate_matrix(mat):
	height, width = mat.shape
	res = np.zeros((height, width, 3))
	for v in range(height):
		for h in range(width):
			res[v,h,mat[v,h]]=1
	return res
def count_metric(road_matrix, prob_matrix, vehicles, building_spacing_voxels = 20*4):
	height, width = road_matrix.shape
	expected_matrix = np.empty(road_matrix.shape, dtype='i')
	for v in range(height):
		for h in range(width):
			if(road_matrix[v,h]==ROAD_STATE_EMPTY):
				expected_matrix[v,h] = VOXEL_STATE_EMPTY
			else:
				expected_matrix[v,h] = VOXEL_STATE_HIDDEN if is_point_hidden(road_matrix, v, h) else VOXEL_STATE_OCCUPIED
	# disp_prob_matrix(inflate_matrix(expected_matrix), False)
	obtained_matrix = np.empty(road_matrix.shape, dtype='i')
	for v in range(height):
		for h in range(width):
			obtained_matrix[v,h] = np.argmax(prob_matrix[v,h])
	##disp_prob_matrix(inflate_matrix(obtained_matrix), False)

	error_matrix = np.empty(road_matrix.shape, dtype='i')
	for v in range(height):
		for h in range(width):
			error_matrix[v,h] = 1 if obtained_matrix[v,h] != expected_matrix[v,h] else 0
	##disp_road_matrix(error_matrix, None, False)

	# print(np.sum(np.sum(error_matrix)))

	mask_matrix = np.ones(road_matrix.shape, dtype='i')
	for v in range(height):
		for h in range(width):
			for vehicle in vehicles:
				if(is_point_inside_vehicle((v,h), vehicle)):
					mask_matrix[v,h] = 0
	for v in range(height):
		for h in range(width):
			if (v < (height / 2 - building_spacing_voxels / 2 - 1) or v >= (height / 2 + building_spacing_voxels / 2 + 1)) and (h < (width / 2 - building_spacing_voxels / 2 - 1) or h >= (width / 2 + building_spacing_voxels / 2 + 1)):
				mask_matrix[v, h] = 0

	masked_error_matrix = error_matrix*mask_matrix
	# disp_road_matrix(masked_error_matrix, None, False)
	# print(f"{np.sum(np.sum(masked_error_matrix))}/{np.sum(np.sum(mask_matrix))}")
	
	return np.sum(np.sum(masked_error_matrix))/np.sum(np.sum(mask_matrix))

def object_metrics(prob_matrix, objects):
	s = 0
	worst = 1
	for ob in objects:
		border, interior = ob
		prob = 1
		for p_i in interior:
			prob*=prob_matrix[p_i[0],p_i[1],VOXEL_STATE_HIDDEN]
		for p_b in border:
			prob*=(prob_matrix[p_i[0],p_i[1],VOXEL_STATE_OCCUPIED] + prob_matrix[p_i[0],p_i[1],VOXEL_STATE_HIDDEN]/2)
		if(prob < worst):
			worst = prob
		s+=prob
	return worst, s/len(objects)

def calc_max_min_distance(height, width, vehicles):
	max_possible_distance = hypot(height, width)
	max_min_distance = 0
	for v in range(height):
		for h in range(width):
			min_distance = max_possible_distance
			for vehicle in vehicles:
				dst = hypot(v - vehicle.vpos, h - vehicle.hpos)
				if(dst < min_distance):
					min_distance = dst
			if min_distance > max_min_distance:
				max_min_distance = min_distance
	return max_min_distance

def find_furthest_vehicle(vehicles, chosen_vehicles):
	current_furthest = None
	current_highest_min_distance = 0
	for v in vehicles:
		min_distance = None
		for v1 in chosen_vehicles:
			dst = hypot(v.vpos - v1.vpos, v.hpos - v1.hpos)
			if min_distance == None or dst < min_distance:
				min_distance = dst
		if min_distance > current_highest_min_distance:
			current_highest_min_distance = min_distance
			current_furthest = v
	return current_furthest

def apply_transmission_strategy(count_tuple_list, num, strategy):
	if(len(count_tuple_list) == 0):
		return count_tuple_list
	if strategy == TRANSMISSION_STRATEGY_RANDOM:
		perm = np.random.permutation(range(len(count_tuple_list)))
		filtered_perm = perm[0:num]
		filtered = []
		for index in filtered_perm:
			filtered+=[count_tuple_list[index]]
		return filtered		
	# sort by random index to avoid bias towards some region in the map
	count_tuple_list = sorted(count_tuple_list, key=lambda t: t[0].index)
	if strategy == TRANSMISSION_STRATEGY_INC_DST:
		height, width, _ = count_tuple_list[0][1].shape
		operating_list = []
		for t in count_tuple_list:
			min_dst = hypot(height, width)
			for v in range(height):
				for h in range(width):
					if t[1][v,h,VOXEL_STATE_OCCUPIED]>0:
						dst = hypot(v - t[0].vpos, h - t[0].hpos)
						if(dst < min_dst):
							min_dst = dst
			operating_list += [(t[0], t[1], min_dst)]
		operating_list = sorted(operating_list, key=lambda p: p[2])
		to_return = []
		for n in range(num):
			to_return+=[(operating_list[n][0], operating_list[n][1])]
		return to_return
	if strategy == TRANSMISSION_STRATEGY_SPREAD:
		height, width, _ = count_tuple_list[0][1].shape
		candidate_sets = []
		vehicles = []
		for t in count_tuple_list:
			vehicles+=[t[0]]
		for i in range(len(vehicles)):
			# take vehicle i as the first point
			chosen_vehicles = [vehicles[i]]
			while len(chosen_vehicles) < num:
				chosen_vehicles += [find_furthest_vehicle(vehicles, chosen_vehicles)]
			max_min_distance = calc_max_min_distance(height, width, chosen_vehicles)
			chosen_tuple_sublist = []
			for t in count_tuple_list:
				if t[0] in chosen_vehicles:
					chosen_tuple_sublist += [t]
			candidate_sets += [(max_min_distance, chosen_tuple_sublist)]
		odered_sets = sorted(candidate_sets, key=lambda p: p[0])
		# return the chosen_tuple_sublist for the best candidate
		return odered_sets[0][1]
	if strategy == TRANSMISSION_STRATEGY_CLOSEST:
		to_return = []
		for t in count_tuple_list:
			to_return += [(t[0], np.zeros_like(t[1]))]
		height, width, _ = count_tuple_list[0][1].shape
		for v in range(height):
			for h in range(width):
				unordered = []
				i = 0
				for t in count_tuple_list:
					unordered += [(hypot(v - t[0].vpos, h - t[0].hpos), i)]
					i += 1
				ordered = sorted(unordered, key=lambda p: p[0])
				for i in range(num):
					tuple_index = ordered[i][1]
					to_return[tuple_index][1][v,h] = count_tuple_list[tuple_index][1][v,h]
		return to_return
	
	print(f"Invalid transmission strategy {strategy}")