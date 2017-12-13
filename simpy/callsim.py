import FreeIMU
imu1=FreeIMU.FreeIMU()
imu1.setValues((1,0.1,0.1,0.2,0.2,0.2),0.1)
ypr1=[1,2,3]
ypr1 = imu1.getYawPitchRollRad(ypr1)

