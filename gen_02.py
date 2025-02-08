import numpy as np
from common import *
import random

def gen_road_map(num_vehicles = 20):
	"""
	road_length_m = 200
	building_spacing_m = 20
	building_unc_border_m = 1
	"""
	road_length_m = 100
	building_spacing_m = 20
	building_unc_border_m = 1
	lane_width_m = 4
	lane_center_m = 2

	voxels_per_meter = 1
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
					mat[int(start_v + v_offs * voxels_per_meter + padding_long*voxels_per_meter + v), int(start_h + h_offs * voxels_per_meter + padding_short*voxels_per_meter + h)] = ROAD_STATE_OCCUPIED
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
					mat[int(start_v + v_offs * voxels_per_meter + padding_long*voxels_per_meter + v), int(start_h + h_offs * voxels_per_meter + padding_short*voxels_per_meter + h)] = ROAD_STATE_OCCUPIED
		else:
			start_h = dim//2
			start_v = dim//2
			h_offs = -4 if i % 2 == 0 else 0
			v_offs = -4 if (i // 2) % 2 == 0 else 0

			direction = random.randint(0,1)

			if direction == 0:
				for h in range(vehicle_dim_long*voxels_per_meter):
					for v in range(vehicle_dim_short*voxels_per_meter):
						mat[int(start_v + v_offs * voxels_per_meter + padding_long*voxels_per_meter + v), int(start_h + h_offs * voxels_per_meter + padding_short*voxels_per_meter + h)] = ROAD_STATE_OCCUPIED
			else:
				for v in range(vehicle_dim_long*voxels_per_meter):
					for h in range(vehicle_dim_short*voxels_per_meter):
						mat[int(start_v + v_offs * voxels_per_meter + padding_long*voxels_per_meter + v), int(start_h + h_offs * voxels_per_meter + padding_short*voxels_per_meter + h)] = ROAD_STATE_OCCUPIED




	return mat, vehicles


m, v = gen_road_map(25)
disp_road_matrix(m, v, True)