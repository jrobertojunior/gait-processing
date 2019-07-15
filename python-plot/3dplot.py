from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt

data = genfromtxt('../recordings/alana-01-spinepos.csv', delimiter=',')
print(data.shape)
v = np.array(data)
print(v.shape)

fig = plt.figure()
plt.plot(v)
ax = plt.axes(projection="3d")
ax.scatter3D(v[:, 0], v[:, 1], v[:, 2])
plt.show()
