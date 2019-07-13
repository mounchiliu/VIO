import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import os
filepath=os.path.abspath('.')

mu = []

#read
with open(filepath + '/lambda.txt', 'r') as f:
    data = f.readlines()
    print(data)

    for line in data:
        mu_ = line.split()
        numbers_float = map(float, mu_)

        mu.append(numbers_float[0])

#plot

#x-axis
ax = list(range(0, len(mu), 1))

#label
plt.xlabel("iteration")
plt.ylabel("lambda")
plt.plot(ax, mu,linestyle='-', marker='o')
plt.show()

