#!/usr/bin/python
import serial
import math
from pylab import *
import time
import FreeIMU

imu1 = FreeIMU.FreeIMU()
ser = serial.Serial('/dev/ttyACM0', 2000000, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False) 
#Tried with and without the last 3 parameters, and also at 1Mbps, same happens.
ser.flushInput()
ser.flushOutput()
data = ser.read()

ion()
x=arange(0,1,0.01)
#lineAx, = plot(x,x*40-20)
#lineAy, = plot(x,x*40-20)
#lineAz, = plot(x,x*40-20)
#lineGx, = plot(x,x*500-250)
#lineGy, = plot(x,x*500-250)
#lineGz, = plot(x,x*500-250)
#lineTetax, = plot(x,x*720-360, label="Y")
#lineTetay, = plot(x,x*720-360, label="X")
#lineTetaz, = plot(x,x*720-360, label="Z")
lineTetax, = plot(x,x*4*math.pi-2*math.pi, label="Yaw, Z")
lineTetay, = plot(x,x*4*math.pi-2*math.pi, label="Pitch, Y")
lineTetaz, = plot(x,x*4*math.pi-2*math.pi, label="Roll, X")
legend()
#legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
draw()
i=0
imu_Ax=x*0
imu_Ay=x*0
imu_Az=x*0
imu_Gx=x*0
imu_Gy=x*0
imu_Gz=x*0

imu_fAx=x*0
imu_fAy=x*0
imu_fAz=x*0
imu_fGx=x*0
imu_fGy=x*0
imu_fGz=x*0

TetaAccX = x*0;
TetaAccY = x*0;
TetaAccZ = x*0;
TetaX = x*0;
TetaY = x*0;
TetaZ = x*0;
TetafX = 1;
TetafY = 0;
TetafZ = 2;
ypr1 = [0.0, 0.0, 0.0]
ofTot = 0

while True:
  data_raw = ser.read()
  data = data+data_raw
  if data_raw=='&':
    i = i + 1
    if i==98 :
      i=0

    index1 = data.find('*')
    blockdata = data.split('*')[1]
    imu_vector=blockdata.split(',')

    if ofTot<3 :
       imu1.offsetAccGy(imu_vector)
       ofTot += 1
    if ofTot==3 :
       imu1.offsetVector = [x / 3 for x in imu1.offsetVector]
       ofTot += 1

    vectAccGy = imu1.digitalCalibration(imu_vector)
    print(vectAccGy)
    ypr1 = imu1.getYawPitchRollRad(ypr1)
#    dt=imu_vector[6]
#    dt=float(dt)/1000
    
#    imu_fAx[i]=vectAccGy[0]
#    imu_fAy[i]=vectAccGy[1]
#    imu_fAz[i]=vectAccGy[2]
#    imu_fGx[i]=vectAccGy[3]
#    imu_fGy[i]=vectAccGy[4]
#    imu_fGz[i]=vectAccGy[5]
    #print("%s %s %s"%(imu_fGx[i], imu_fGy[i], imu_fGz[i]))

#    pitchY = math.atan(imu_fAx[i]/math.sqrt(math.pow(imu_fAy[i],2)+math.pow(imu_fAz[i],2)))
#    rollX = math.atan2(imu_fAy[i], math.sqrt(math.pow(imu_fAx[i],2)+math.pow(imu_fAz[i],2)))
#    yawZ = math.atan(imu_fAz[i]/math.sqrt(math.pow(imu_fAy[i],2)+math.pow(imu_fAx[i],2)))
#    yawZ = math.atan2(imu_fAx[i],imu_fAy[i])

#    TetaAccX[i] = pitchY*180/math.pi
#    TetaAccY[i] = rollX*180/math.pi
#    TetaAccZ[i] = yawZ*180/math.pi
    TetaAccX[i] = ypr1[0]
    TetaAccY[i] = ypr1[1]
    TetaAccZ[i] = ypr1[2]

#    TetaX[i]=TetafX+dt*imu_fGx[i]
#    TetaY[i]=TetafY+dt*imu_fGy[i]
#    TetaZ[i]=TetafZ+dt*imu_fGz[i]
    #print("%s %s %s %s %s %s "%(TetafX, imu_fGx[i], TetafY, imu_fGy[i], TetafZ, imu_fGz[i]))
    #print("%s %s %s"%(TetafX, TetafY, TetafZ))

#    lineTetax.set_ydata(TetaX)
#    lineTetay.set_ydata(TetaY)
#    lineTetaz.set_ydata(TetaZ)
    
    lineTetax.set_ydata(TetaAccX)
    lineTetay.set_ydata(TetaAccY)
    lineTetaz.set_ydata(TetaAccZ)
    draw()
#    TetafX=TetaX[i]
#    TetafY=TetaY[i]
#    TetafZ=TetaZ[i]
  # reset data
  if data_raw=='&': data=''
