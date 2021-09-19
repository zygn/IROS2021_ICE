import numpy as np
import matplotlib.pyplot as plt

algo_a = np.genfromtxt('SOCHI_FGM_CONV.csv', delimiter=',')
algo_b = np.genfromtxt('SOCHI_FGM_GNU_CONV.csv', delimiter=',')

start = [-156.68159080844768,-121.23484964729177, 0]

for i in range(len(algo_a)):
    algo_a[i] = algo_a[i] - start
for i in range(len(algo_b)):
    algo_b[i] = algo_b[i] - start

plt.plot(algo_a, c='b')
plt.plot(algo_b, c='r')
plt.show()