#!/usr/bin/python
import serial
from pylab import *
import time
#from __future__ import print_function

ser = serial.Serial('/dev/ttyACM0', 2000000, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False) 
#Tried with and without the last 3 parameters, and also at 1Mbps, same happens.
ser.flushInput()
ser.flushOutput()
data = ser.read()

ion()
x=arange(0,1,0.01)
lineAx, = plot(x,x*65000-32500)
lineAy, = plot(x,x*65000-32500)
lineAz, = plot(x,x*65000-32500)
lineGx, = plot(x,x*65000-32500)
lineGy, = plot(x,x*65000-32500)
lineGz, = plot(x,x*65000-32500)
draw()
i=0
imu_Ax=x*0
imu_Ay=x*0
imu_Az=x*0
imu_Gx=x*0
imu_Gy=x*0
imu_Gz=x*0

while True:
  data_raw = ser.read()
  data = data+data_raw
  if data_raw=='&':
    i = i + 1
    if i==98 :
      i=0
      imu_Ax=x*0
      imu_Ay=x*0
      imu_Az=x*0
      imu_Gx=x*0
      imu_Gy=x*0
      imu_Gz=x*0

    #print(data.find('*'))
    index1 = data.find('*')
    #print(data[index1+1])
    blockdata = data.split('*')[1]
    imu_vector=blockdata.split(',')
    imu_Ax[i]=imu_vector[0]
    imu_Ay[i]=imu_vector[1]
    imu_Az[i]=imu_vector[2]
    imu_Gx[i]=imu_vector[3]
    imu_Gy[i]=imu_vector[4]
    imu_Gz[i]=imu_vector[5]
    #print(imu_vector1[i])
    lineAx.set_ydata(imu_Ax)
    lineAy.set_ydata(imu_Ay)
    lineAz.set_ydata(imu_Az)
    lineGx.set_ydata(imu_Gx)
    lineGy.set_ydata(imu_Gy)
    lineGz.set_ydata(imu_Gz)
    draw()

  if data_raw=='&': data=''
