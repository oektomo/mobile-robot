// a=tau / (tau + loop time)
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float tau=0.075;
float a=0.0;

float Complementary(float newAngle, float newRate,int looptime) {
	float dtC = float(looptime)/1000.0;
	a=tau/(tau+dtC);
	x_angleC= a* (x_angleC + newRate * dtC) + (1-a) * (newAngle);
return x_angleC;
}

// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()

float Complementary2(float newAngle, float newRate,int looptime) {
	float k=10;
	float dtc2=float(looptime)/1000.0;

	x1 = (newAngle -   x_angle2C)*k*k;
	y1 = dtc2*x1 + y1;
	x2 = y1 + (newAngle -   x_angle2C)*2*k + newRate;
	x_angle2C = dtc2*x2 + x_angle2C;

return x_angle2C;
}

// KasBot V1 - Kalman filter module

float Q_angle  =  0.01; //0.001
float Q_gyro   =  0.0003;  //0.003
float R_angle  =  0.01;  //0.03

float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
float  y, S;
float K_0, K_1;

// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()

float kalmanCalculate(float newAngle, float newRate,int looptime)
{
float dt = float(looptime)/1000;
x_angle += dt * (newRate - x_bias);
P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
P_01 +=  - dt * P_11;
P_10 +=  - dt * P_11;
P_11 +=  + Q_gyro * dt;

y = newAngle - x_angle;
S = P_00 + R_angle;
K_0 = P_00 / S;
K_1 = P_10 / S;

x_angle +=  K_0 * y;
x_bias  +=  K_1 * y;
P_00 -= K_0 * P_00;
P_01 -= K_0 * P_01;
P_10 -= K_1 * P_00;
P_11 -= K_1 * P_01;

return x_angle;
}

