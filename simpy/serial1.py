#!/usr/bin/python
import serial
#from __future__ import print_function

ser = serial.Serial('/dev/ttyACM0', 2000000, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False) 
#Tried with and without the last 3 parameters, and also at 1Mbps, same happens.
ser.flushInput()
ser.flushOutput()
data = ser.read()
while True:
#  data_raw = ser.readline()
  data_raw = ser.read()
  data = data+data_raw
#  print(data, end='')
  if data_raw=='&':
    print(data)
  if data_raw=='&': data=''
