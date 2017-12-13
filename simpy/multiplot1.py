#!/usr/bin/python
import matplotlib.pyplot as plt
import numpy as np

x = np.arange(50)
y = np.exp(x)
fig1 = plt.figure(0)
ax1 = fig1.add_subplot(111)
ax1.plot(x, y)
plt.show()

z = np.sin(x)
fig2 = plt.figure(1)
ax2 = fig2.add_subplot(111)
ax2.plot(x, z)
plt.show()

w = np.cos(x)
ax1.plot(x, w) # can continue plotting on the first axis
plt.show()

