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
n_list = range(10,1001, 10) #second sensor (low precision)
res_count = {ST_EMP: [], ST_OCC: [], ST_HID: []}
res_alpha = {ST_EMP: [], ST_OCC: [], ST_HID: []}

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

		prob_fixed = ((1/3) * alpha_fixed + (1-alpha_fixed)*(n_fixed_st[st]/n_fixed))
		prob_var = ((1/3) * alpha_var + (1-alpha_var)*(n_var_st[st]/n_var))

		w_fixed = n_fixed
		w_var = n_var
		res_count[st]+=[prob_fixed*(w_fixed/(w_fixed+w_var)) + prob_var*(w_var/(w_fixed+w_var))]

		w_fixed = n_fixed*(1/alpha_fixed)
		w_var = n_var*(1/alpha_var)
		res_alpha[st]+=[prob_fixed*(w_fixed/(w_fixed+w_var)) + prob_var*(w_var/(w_fixed+w_var))]
		

fig, axs = plt.subplots(2)
res = res_count
axs[0].plot(n_list, res[0])
axs[0].plot(n_list, res[1])
axs[0].plot(n_list, res[2])
axs[0].plot(n_list, [sum(res[st][i] for st in states) for i in range(len(n_list))])
axs[0].hlines(target_percentages[0].values(), min(n_list), max(n_list), linestyles = 'dashdot', colors='black')
axs[0].hlines(target_percentages[1].values(), min(n_list), max(n_list), linestyles = 'dashed', colors='red')
axs[0].legend(["empty", "occupied", "hidden"], loc="upper right")
res = res_alpha
axs[1].plot(n_list, res[0])
axs[1].plot(n_list, res[1])
axs[1].plot(n_list, res[2])
axs[1].plot(n_list, [sum(res[st][i] for st in states) for i in range(len(n_list))])
axs[1].hlines(target_percentages[0].values(), min(n_list), max(n_list), linestyles = 'dashdot', colors='black')
axs[1].hlines(target_percentages[1].values(), min(n_list), max(n_list), linestyles = 'dashed', colors='red')
axs[1].legend(["empty", "occupied", "hidden"], loc="upper right")
plt.tight_layout()
plt.show()


