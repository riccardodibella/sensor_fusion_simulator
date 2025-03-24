import numpy as np
from common import *


def genera_griglia_csv(righe, colonne, angolo_altezza, angolo_larghezza, nome_file="road_5.csv"):
    mat = np.zeros((righe, colonne), dtype='i')
    vehicles = []
    vehicle_index = 0
    
    with open(nome_file, 'w', newline='') as file_csv:
        """writer = csv.writer(file_csv)
                                writer.writerow(["#", "[]"])"""
        
        for i in range(righe):
            riga = []
            for j in range(colonne):
                if (i < angolo_altezza or i > righe - (angolo_altezza + 1)) and (j < angolo_larghezza or j > colonne - (angolo_larghezza + 1)):
                    val = OCC
                elif ((angolo_altezza <= i < angolo_altezza + 1 or righe - angolo_altezza - 1 <= i < righe - angolo_altezza) and 
                      (j < angolo_larghezza or j > colonne - (angolo_larghezza + 2))) or \
                     ((angolo_larghezza <= j < angolo_larghezza + 1 or colonne - angolo_larghezza - 1 <= j < colonne - angolo_larghezza) and 
                      (i < angolo_altezza or i > righe - (angolo_altezza + 2))):
                    val = UNC  # Contorno giallo intorno agli angoli rossi
                elif (i == 5 and j == 3) or (i == 12 and j == 15):
                    val = UNC
                    vehicles.append(Vehicle(i, j, 2, 4, 1, vehicle_index))
                    vehicle_index += 1
                else:
                    val = EMP
                riga.append(val)
                mat[i, j] = val
            """writer.writerow(riga)"""
    
    """with open(nome_file, 'r') as file_csv:
                    lines = file_csv.readlines()"""
    
    vehicle_json_list = [v.__dict__ for v in vehicles]
    # lines[0] = f"# {json.dumps(vehicle_json_list)}\n"
    
    """with open(nome_file, 'w') as file_csv:
                    file_csv.writelines(lines)"""
    
    disp_road_matrix(mat, vehicles)

# genera_griglia_csv(40, 50, angolo_altezza=6, angolo_larghezza=10)

def gen_road_map():
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

	#buildings
	for v in range(dim):
		for h in range(dim):
			if (v < (dim / 2 - building_spacing_voxels / 2) or v > (dim / 2 + building_spacing_voxels / 2)) and (h < (dim / 2 - building_spacing_voxels / 2) or h > (dim / 2 + building_spacing_voxels / 2)):
				mat[v, h] = ROAD_STATE_OCCUPIED
			elif (v < (dim / 2 - building_spacing_voxels / 2 + building_unc_border_voxels) or v > (dim / 2 + building_spacing_voxels / 2 - building_unc_border_voxels)) and (h < (dim / 2 - building_spacing_voxels / 2 + building_unc_border_voxels) or h > (dim / 2 + building_spacing_voxels / 2 - building_unc_border_voxels)):
				mat[v, h] = ROAD_STATE_UNCLEAR

	return mat, vehicles


m, v = gen_road_map()
disp_road_matrix(m, v, False)