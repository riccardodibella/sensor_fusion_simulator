import matplotlib.pyplot as plt


ST_EMP = 0
ST_OCC = 1
ST_HID = 2
states = [ST_EMP, ST_OCC, ST_HID]

target_percentages = {ST_EMP: 0.1, ST_OCC: 0.7, ST_HID: 0.2}

n_list = [10,20,30,40,50]
res = {ST_EMP: [], ST_OCC: [], ST_HID: []}
alpha = 0.95
for n in n_list:
	n_st = {}
	for st in states:
		n_st[st] = n * target_percentages[st]
	a_n = alpha**n
	for st in states:
		res[st] = res[st] + [(1/3) * a_n + (1-a_n)*(n_st[st] / n)]

print(res)
plt.plot(n_list, res[0])
plt.plot(n_list, res[1])
plt.plot(n_list, res[2])
plt.show()