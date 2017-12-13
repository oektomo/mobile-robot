#!/usr/bin/python
from pylab import *
import time

ion()

tstart = time.time()               # for profiling
x = arange(0,2*pi,0.01)            # x-array
y = arange(0,2*pi,0.01)            # x-array
line1, = plot(x,sin(x))
line2, = plot(x,cos(x))
for i in arange(1,200):
    line1.set_ydata(sin(x+i/10.0))  # update the data
    draw()                         # redraw the canvas
    line2.set_ydata(cos(x+i/10.0))  # update the data
    draw()                         # redraw the canvas

print 'FPS:' , 200/(time.time()-tstart)
