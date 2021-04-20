from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
import numpy as np
x = [1000,1000,1000,1000,1000,5000,5000,5000,5000,5000,10000,10000,10000,10000,10000]
y = [13,21,29,37,45,13,21,29,37,45,13,21,29,37,45]
z = [75.2,79.21,80.02,81.2,81.62,84.79,87.38,87.9,88.54,88.56,88.34,89.66,90.11,90.79,90.87]
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_trisurf(x, y, z, cmap=cm.jet, linewidth=0.2)
plt.show()
