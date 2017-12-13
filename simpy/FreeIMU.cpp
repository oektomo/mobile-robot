

#include "FreeIMU.h"


FreeIMU::FreeIMU() {
  #if HAS_ADXL345()
    acc = ADXL345();
  #elif HAS_BMA180()
    acc = BMA180();
  #endif
  
  #if HAS_HMC5883L()
    magn = HMC58X3();
  #endif
  
  #if HAS_ITG3200()
    gyro = ITG3200();
  #elif HAS_MPU6050()
    accgyro = MPU60X0(); // I2C
  #elif HAS_MPU6000()
    accgyro = MPU60X0(); // SPI for Arduimu v3
  #endif
    
  #if HAS_MS5611()
    baro = MS561101BA();
  #endif
  
  // initialize quaternion
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;
  twoKp = twoKpDef;
  twoKi = twoKiDef;
  integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
  lastUpdate = 0;
  now = 0;
  
  #ifndef CALIBRATION_H
  // initialize scale factors to neutral values
  acc_scale_x = 1;
  acc_scale_y = 1;
  acc_scale_z = 1;
  magn_scale_x = 1;
  magn_scale_y = 1;
  magn_scale_z = 1;
  #else
  // get values from global variables of same name defined in calibration.h
  acc_off_x = ::acc_off_x;
  acc_off_y = ::acc_off_y;
  acc_off_z = ::acc_off_z;
  acc_scale_x = ::acc_scale_x;
  acc_scale_y = ::acc_scale_y;
  acc_scale_z = ::acc_scale_z;
  magn_off_x = ::magn_off_x;
  magn_off_y = ::magn_off_y;
  magn_off_z = ::magn_off_z;
  magn_scale_x = ::magn_scale_x;
  magn_scale_y = ::magn_scale_y;
  magn_scale_z = ::magn_scale_z;
  #endif
}


/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in radians between
 * the Earth North and the IMU X axis (yaw), the Earth ground plane and the IMU X axis (pitch)
 * and the Earth ground plane and the IMU Y axis.
 * 
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 * 
 * @param ypr three floats array which will be populated by Yaw, Pitch and Roll angles in radians
*/
void FreeIMU::getYawPitchRollRad(float * ypr) {
  float q[4]; // quaternion
  float gx, gy, gz; // estimated gravity direction
  getQ(q);
  
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
  ypr[1] = atan(gx / sqrt(gy*gy + gz*gz));
  ypr[2] = atan(gy / sqrt(gx*gx + gz*gz));
}

/**
 * Populates array q with a quaternion representing the IMU orientation with respect to the Earth
 * 
 * @param q the quaternion to populate
*/
void FreeIMU::getQ(float * q) {
  float val[9];
  getValues(val);
  
  DEBUG_PRINT(val[3] * M_PI/180);
  DEBUG_PRINT(val[4] * M_PI/180);
  DEBUG_PRINT(val[5] * M_PI/180);
  DEBUG_PRINT(val[0]);
  DEBUG_PRINT(val[1]);
  DEBUG_PRINT(val[2]);
  DEBUG_PRINT(val[6]);
  DEBUG_PRINT(val[7]);
  DEBUG_PRINT(val[8]);
  
  now = micros();
  sampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);
  lastUpdate = now;
  
  // gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
  #if IS_9DOM()
    #if HAS_AXIS_ALIGNED()
      AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
    #elif defined(SEN_10724)
      AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[7], -val[6], val[8]);
    #elif defined(ARDUIMU_v3)
      AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], -val[6], -val[7], val[8]);
    #endif
  #else
    AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2]);
  #endif
  
  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
  
}

/**
 * Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
 * compensation algorithms from Sebastian Madgwick's filter which eliminates the need for a reference
 * direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
 * axis only.
 * 
 * @see: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
*/
#if IS_9DOM()
void  FreeIMU::AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
#elif IS_6DOM()
void  FreeIMU::AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) {
#endif
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
  float qa, qb, qc;

  // Auxiliary variables to avoid repeated arithmetic
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;
  
  #if IS_9DOM() && not defined(DISABLE_MAGN)
  // Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
  if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
    float hx, hy, bx, bz;
    float halfwx, halfwy, halfwz;
    
    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;
    
    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
    
    // Estimated direction of magnetic field
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
    
    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex = (my * halfwz - mz * halfwy);
    halfey = (mz * halfwx - mx * halfwz);
    halfez = (mx * halfwy - my * halfwx);
  }
  #endif

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
    float halfvx, halfvy, halfvz;
    
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // Estimated direction of gravity
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
  
    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex += (ay * halfvz - az * halfvy);
    halfey += (az * halfvx - ax * halfvz);
    halfez += (ax * halfvy - ay * halfvx);
  }

  // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
  if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);
  
  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

    values[0] = (float) accval[0];
    values[1] = (float) accval[1];
    values[2] = (float) accval[2];
    accgyroval[3] = accgyroval[3] - gyro_off_x;
    accgyroval[4] = accgyroval[4] - gyro_off_y;
    accgyroval[5] = accgyroval[5] - gyro_off_z;
