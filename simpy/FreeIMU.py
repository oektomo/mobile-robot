import math

class FreeIMU:
	
	def __init__(self):
	   self.q0 = 1.0
	   self.q1 = 0.0
	   self.q2 = 0.0
	   self.q3 = 0.0

	   self.accGyroVal = [0,0,0,0,0,0]
	   self.sampleFreq=0.1
	   self.twoKp = (2.0 * 0.5) # 2 * proportional gain
	   self.twoKi = (2.0 * 0.1) # 2 * integral gain

	   self.integralFBx = 0.0
	   self.integralFBy = 0.0
	   self.integralFBz = 0.0
	   self.offsetVector = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	
	def offsetAccGy(self, rawData):
	   rawData = rawData[0:6]
	   rawDataf = [float(i) for i in rawData]
	   self.offsetVector =map(sum, zip(self.offsetVector,rawDataf))

	def digitalCalibration(self, value):
	   self.sampleFreq = value[6]
	   self.sampleFreq = 1000/float(self.sampleFreq)
	   vectorAccGy = [0.0,0.0,0.0,0.0,0.0,0.0]
	   vectorAccGy[0]= float(value[0])-self.offsetVector[0]
	   vectorAccGy[1]= float(value[1])-self.offsetVector[1]
	   vectorAccGy[2]= float(value[2])-self.offsetVector[2]
	   vectorAccGy[3]= float(value[3])-self.offsetVector[3]
	   vectorAccGy[4]= float(value[4])-self.offsetVector[4]
	   vectorAccGy[5]= float(value[5])-self.offsetVector[5]

	   vectorAccGy[0]=vectorAccGy[0]*2/32768.0*9.8
	   vectorAccGy[1]=vectorAccGy[1]*2/32768.0*9.8
	   vectorAccGy[2]=vectorAccGy[2]*2/32768.0*9.8
	   vectorAccGy[3]=vectorAccGy[3]*250/32768.0*math.pi/180
	   vectorAccGy[4]=vectorAccGy[4]*250/32768.0*math.pi/180
	   vectorAccGy[5]=vectorAccGy[5]*250/32768.0*math.pi/180

	   self.accGyroVal = vectorAccGy
	   return vectorAccGy
    

	def setValues(self, accGyro, dt):
	   self.accGyroVal = accGyro
	   self.sampleFreq = dt

	def getValues(self):
	   return self.accGyroVal

	def invSqrt(self, value):
		return value**-0.5

	def AHRSupdate(self, accGyro):
	  q0q0 = self.q0*self.q0
	  q0q1 = self.q0*self.q1
	  q0q2 = self.q0*self.q2
	  q0q3 = self.q0*self.q3
	  q1q1 = self.q1*self.q1
	  q1q2 = self.q1*self.q2
	  q1q3 = self.q1*self.q3
	  q2q2 = self.q2*self.q2
	  q2q3 = self.q2*self.q3
	  q3q3 = self.q3*self.q3
	  recipNorm = 0.0
	  halfex = 0.0
	  halfey = 0.0
	  halfez = 0.0
	  qa = 0.0
	  qb = 0.0
	  qc = 0.0
	  
	  #Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	  ax = accGyro[0]
	  ay = accGyro[1]
	  az = accGyro[2]
	  gx = accGyro[3]
	  gy = accGyro[4]
	  gz = accGyro[5]
	  
	  if (ax!=0.0) & (ay!=0.0) & (az!=0.0):
	  	halfvx = 0.0
		halfvy = 0.0
		halfvz = 0.0
		
		#Normalise accelerometer measurement
		recipNorm = self.invSqrt(ax*ax + ay*ay + az*az)
		ax = ax*recipNorm
		ay = ay*recipNorm
		az = az*recipNorm

		#Estimated direction of gravity
		halfvx = q1q3 - q0q2
		halfvy = q0q1 + q2q3
		halfvz = q0q0 - 0.5 + q3q3

		#Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += (ay * halfvz - az * halfvy)
		halfey += (az * halfvx - ax * halfvz)
		halfez += (ax * halfvy - ay * halfvx)
	  #Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	  if (halfex!=0.0) & (halfey!=0.0) & (halfez!=0.0):
	        # Compute and apply integral feedback if enabled
	   	if self.twoKi>0.0:
		    self.integralFBx += self.twoKi * halfex * (1.0 / self.sampleFreq)# integral error scaled by Ki
		    self.integralFBy += self.twoKi * halfey * (1.0 / self.sampleFreq)
		    self.integralFBz += self.twoKi * halfez * (1.0 / self.sampleFreq)
		    gx += self.integralFBx  # apply integral feedback
		    gy += self.integralFBy
		    gz += self.integralFBz
		else:
		    self.integralFBx = 0.0 # prevent integral windup
		    self.integralFBy = 0.0
		    self.integralFBz = 0.0
		# Apply proportional feedback
		gx += self.twoKp * halfex
		gy += self.twoKp * halfey
		gz += self.twoKp * halfez
	  # Integrate rate of change of quaternion
	  gx *= (0.5 * (1.0 / self.sampleFreq))   # pre-multiply common factors
	  gy *= (0.5 * (1.0 / self.sampleFreq))
	  gz *= (0.5 * (1.0 / self.sampleFreq))
	  qa = self.q0
	  qb = self.q1
	  qc = self.q2
	  self.q0 += (-qb * gx - qc * gy - self.q3 * gz)
	  self.q1 += (qa * gx + qc * gz - self.q3 * gy)
	  self.q2 += (qa * gy - qb * gz + self.q3 * gx)
	  self.q3 += (qa * gz + qb * gy - qc * gx)

	  # Normalise quaternion
	  recipNorm = self.invSqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
	  self.q0 *= recipNorm
	  self.q1 *= recipNorm
	  self.q2 *= recipNorm
	  self.q3 *= recipNorm

	def getQ(self, q):

	   val = self.getValues()

	   valAHRS = (val[3]*math.pi/180, val[4]*math.pi/180, val[5]*math.pi/180, val[0], val[1], val[2])

	   self.AHRSupdate(valAHRS)

	   q[0] = self.q0
	   q[1] = self.q1
	   q[2] = self.q2
	   q[3] = self.q3
	   return q


	def getYawPitchRollRad(self, ypr):
	   q = [0, 0, 0, 0]
	   self.getQ(q)

	   # estimated gravity direction
	   gx = 2 * (q[1]*q[3] - q[0]*q[2])
	   gy = 2 * (q[0]*q[1] + q[2]*q[3])
	   gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]

	   ypr[0] = math.atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] -1)
	   ypr[1] = math.atan(gx / math.sqrt(gy*gy + gz*gz))
	   ypr[2] = math.atan(gy / math.sqrt(gz*gz + gz*gz))

	   return ypr
