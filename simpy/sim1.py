import numpy as np
import matplotlib.pyplot as plt

# Set limits and number of points in grid
#y, x = np.mgrid[10:-10:100j, 10:-10:100j]
y, x = np.mgrid[10:-10:10j, 10:-10:10j]

x_obstacle, y_obstacle = -5.0, 5.0
alpha_obstacle, a_obstacle, b_obstacle = 1.0, 1e3, 2e3

p = -alpha_obstacle * np.exp(-((x - x_obstacle)**2 / a_obstacle
                               + (y - y_obstacle)**2 / b_obstacle))

# For the absolute values of "dx" and "dy" to mean anything, we'll need to
# specify the "cellsize" of our grid.  For purely visual purposes, though,
# we could get away with just "dy, dx = np.gradient(p)".
dy, dx = np.gradient(p, np.diff(y[:2, 0]), np.diff(x[0, :2]))
print(y)
print(x)
print(y[:2,0])
print(x[:2,0])
print(dy[:2,0])
print(dx[:2,0])

fig, ax = plt.subplots()
#ax.quiver(x, y, dx, dy, p)
ax.quiver(x, y, dx, dy)
ax.set(aspect=1, title='Quiver Plot')
plt.show()
