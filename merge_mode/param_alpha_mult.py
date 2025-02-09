import matplotlib.pyplot as plt


ST_EMP = 0
ST_OCC = 1
ST_HID = 2
states = [ST_EMP, ST_OCC, ST_HID]

NUM_SENSORS = 2

#target_percentages = [{ST_EMP: 0.1, ST_OCC: 0.7, ST_HID: 0.2}, {ST_EMP: 0.2, ST_OCC: 0.3, ST_HID: 0.5}]
target_percentages = [{ST_EMP: 0.8, ST_OCC: 0.05, ST_HID: 0.15}, {ST_EMP: 0.05, ST_OCC: 0.9, ST_HID: 0.05}]


alpha = [0.55, 0.9]
n_fixed = 50 #first sensor (high precision)
n_list = [10,20,30,40,50,60,70,80,90,100,150,200,250,300,1000] #second sensor (low precision)
res = {ST_EMP: [], ST_OCC: [], ST_HID: []}

for n_var in n_list:
	n_fixed_st = {}
	for st in states:
		n_fixed_st[st] = n_fixed * target_percentages[0][st]
	n_var_st = {}
	for st in states:
		n_var_st[st] = n_var * target_percentages[1][st]
	for st in states:
		alpha_fixed = alpha[0]**n_fixed
		alpha_var = alpha[1]**n_var
		fixed_frac = n_fixed_st[st] / (n_fixed + n_var)
		var_frac = n_var_st[st] / (n_fixed + n_var)
		res[st] = res[st] + [(1/3) * (alpha_fixed*n_fixed/(n_fixed + n_var) + alpha_var*n_var/(n_fixed + n_var)) + ((fixed_frac + var_frac) - (alpha_fixed * fixed_frac) - (alpha_var * var_frac))]

print(res)
plt.plot(n_list, res[0])
plt.plot(n_list, res[1])
plt.plot(n_list, res[2])
plt.plot(n_list, [sum(res[st][i] for st in states) for i in range(len(n_list))])
plt.hlines(target_percentages[0].values(), min(n_list), max(n_list), linestyles = 'dashed', colors='black')
plt.hlines(target_percentages[1].values(), min(n_list), max(n_list), linestyles = 'dashed', colors='red')
plt.show()