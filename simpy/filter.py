def printValue(str):
	print(str+1)
	return

#float Complementary(float newAngle, float newRate,int looptime) {
def Complementary(newAngle, newRate, looptime):
	k = 10
	dtc2 = looptime/1000
	x1 = (newAngle-x_angle2C)*k*k
	y1 = dtc2*x1 + y1
	x2 = y1 + (newAngle-x_angle2C)*2*k + newRate
	x_angle2C = dtc2*x2 + x_angle2C

	return x_angle2C
