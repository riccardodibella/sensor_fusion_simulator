import numpy as np
from math import pi, sin, cos, floor
from common import *

def coord_noise():
	# tiny noise to calculating rays on the border of a cell
	res = 0
	while res == 0:
		res = np.random.normal(0, 0.1)
	return res

eps = 0.01 # tiny deviation used to enter inside a cell and avoid problems when checking on the border

def calc_direction(v_increase, h_increase):
	if(v_increase == 0):
		return DIR_E if h_increase > 0 else DIR_W
	if(h_increase == 0):
		return DIR_S if v_increase > 0 else DIR_N
	if(v_increase > 0 and h_increase > 0):
		return DIR_SE
	if(v_increase > 0 and h_increase < 0):
		return DIR_SW
	if(v_increase < 0 and h_increase < 0):
		return DIR_NW
	if(v_increase < 0 and h_increase > 0):
		return DIR_NE
	assert(False)

def is_point_inside_veichle(point, vehicle):
	# point is (ver, hor)
	return 	point[0] >= vehicle.vpos-vehicle.h2 and point[0] < vehicle.vpos+vehicle.h2 and point[1] >= vehicle.hpos-vehicle.w2 and point[1] < vehicle.hpos+vehicle.w2

def gen_hit(road_state, sensor_type):
	prob_hit = PROB_LIDAR_HIT[sensor_type][road_state]
	return np.random.random_sample() < prob_hit


def generate_count_matrix(m, vehicle):
	map_height, map_width = m.shape
	result_shape = (map_height, map_width, 3)
	result = np.zeros(result_shape, dtype='i')

	start_coord_hor = vehicle.hpos + coord_noise()
	start_coord_ver = vehicle.vpos + coord_noise()

	angles = np.linspace(0,2*pi,3600)
	for angle in angles:
		v_increase = sin(angle) # positive if going downwards
		h_increase = cos(angle) # positive if going rightwards
		direction = calc_direction(v_increase, h_increase)

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

			if(direction == DIR_N):
				cur_vpos = upper_limit - eps
			elif(direction == DIR_E):
				cur_hpos = right_limit + eps
			elif(direction == DIR_S):
				cur_vpos = lower_limit + eps
			elif(direction == DIR_W):
				cur_hpos = left_limit - eps
			elif(direction == DIR_SE):
				t_down = (lower_limit - cur_vpos)/v_increase
				t_right = (right_limit - cur_hpos)/h_increase
				t = min(t_down, t_right)
				cur_hpos += h_increase * t + eps
				cur_vpos += v_increase * t + eps
			elif(direction == DIR_SW):
				t_down = (lower_limit - cur_vpos)/v_increase
				t_left = (cur_hpos - left_limit)/(-1*h_increase)
				t = min(t_down, t_left)
				cur_hpos += h_increase * t - eps
				cur_vpos += v_increase * t + eps
			elif(direction == DIR_NW):
				t_up = (cur_vpos - upper_limit)/(-1*v_increase)
				t_left = (cur_hpos - left_limit)/(-1*h_increase)
				t = min(t_up, t_left)
				cur_hpos += h_increase * t - eps
				cur_vpos += v_increase * t - eps
			elif(direction == DIR_NE):
				t_up = (cur_vpos - upper_limit)/(-1*v_increase)
				t_right = (right_limit - cur_hpos)/h_increase
				t = min(t_up, t_right)
				cur_hpos += h_increase * t + eps
				cur_vpos += v_increase * t - eps
			else:
				break
		ray_blocked = False
		for vpos, hpos in voxel_list:
			if(is_point_inside_veichle((vpos, hpos), vehicle)):
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

road_map, vehicles = load_map("road_03.csv")

count_matrix = generate_count_matrix(road_map, vehicles[0])

matrix = count_matrix[:,:,0]
plt.imshow(matrix, cmap='viridis', interpolation='nearest')
plt.colorbar(label='Value')
num_rows, num_cols = matrix.shape
plt.gca().set_xticks(np.arange(-0.5, num_cols, 1), minor=True)
plt.gca().set_yticks(np.arange(-0.5, num_rows, 1), minor=True)
plt.gca().grid(which='minor', color='black', linestyle='-', linewidth=0.5)
plt.gca().tick_params(which='minor', length=0)
plt.xticks([])
plt.yticks([])
plt.show()

disp_road_matrix(road_map, vehicles)