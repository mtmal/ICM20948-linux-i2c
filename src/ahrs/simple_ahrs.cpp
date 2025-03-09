////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2020 Waveshare Electronics
//
// This code has been taken from Waveshare's example implementation of ICM20948
// and adjusted to the current project.
//
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include "ahrs/ahrs_math.h"
#include "ahrs/simple_ahrs.h"

#define Kp 4.50f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 1.0f    // integral gain governs rate of convergence of gyroscope biases

void SimpleAHRSupdate(IMUData& data)
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;
  float ex, ey, ez, halfT = 0.024f;

  float q0q0 = data.mQuat[0] * data.mQuat[0];
  float q0q1 = data.mQuat[0] * data.mQuat[1];
  float q0q2 = data.mQuat[0] * data.mQuat[2];
  float q0q3 = data.mQuat[0] * data.mQuat[3];
  float q1q1 = data.mQuat[1] * data.mQuat[1];
  float q1q2 = data.mQuat[1] * data.mQuat[2];
  float q1q3 = data.mQuat[1] * data.mQuat[3];
  float q2q2 = data.mQuat[2] * data.mQuat[2];
  float q2q3 = data.mQuat[2] * data.mQuat[3];
  float q3q3 = data.mQuat[3] * data.mQuat[3];

  norm = invSqrt(data.mAcc[0] * data.mAcc[0] + data.mAcc[1] * data.mAcc[1] + data.mAcc[2] * data.mAcc[2]);
  data.mAcc[0] = data.mAcc[0] * norm;
  data.mAcc[1] = data.mAcc[1] * norm;
  data.mAcc[2] = data.mAcc[2] * norm;

  norm = invSqrt(data.mMag[0] * data.mMag[0] + data.mMag[1] * data.mMag[1] + data.mMag[2] * data.mMag[2]);
  data.mMag[0] = data.mMag[0] * norm;
  data.mMag[1] = data.mMag[1] * norm;
  data.mMag[2] = data.mMag[2] * norm;

  // compute reference direction of flux
  hx = 2 * data.mMag[0] * (0.5f - q2q2 - q3q3) + 2 * data.mMag[1] * (q1q2 - q0q3) + 2 * data.mMag[2] * (q1q3 + q0q2);
  hy = 2 * data.mMag[0] * (q1q2 + q0q3) + 2 * data.mMag[1] * (0.5f - q1q1 - q3q3) + 2 * data.mMag[2] * (q2q3 - q0q1);
  hz = 2 * data.mMag[0] * (q1q3 - q0q2) + 2 * data.mMag[1] * (q2q3 + q0q1) + 2 * data.mMag[2] * (0.5f - q1q1 - q2q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = hz;

  // estimated direction of gravity and flux (v and w)
  vx = 2 * (q1q3 - q0q2);
  vy = 2 * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
  wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
  wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2);

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (data.mAcc[1] * vz - data.mAcc[2] * vy) + (data.mMag[1] * wz - data.mMag[2] * wy);
  ey = (data.mAcc[2] * vx - data.mAcc[0] * vz) + (data.mMag[2] * wx - data.mMag[0] * wz);
  ez = (data.mAcc[0] * vy - data.mAcc[1] * vx) + (data.mMag[0] * wy - data.mMag[1] * wx);

  if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
  {
    exInt = exInt + ex * Ki * halfT;
    eyInt = eyInt + ey * Ki * halfT;
    ezInt = ezInt + ez * Ki * halfT;

    data.mGyro[0] = data.mGyro[0] + Kp * ex + exInt;
    data.mGyro[1] = data.mGyro[1] + Kp * ey + eyInt;
    data.mGyro[2] = data.mGyro[2] + Kp * ez + ezInt;
  }

  data.mQuat[0] = data.mQuat[0] + (-data.mQuat[1] * data.mGyro[0] - data.mQuat[2] * data.mGyro[1] - data.mQuat[3] * data.mGyro[2]) * halfT;
  data.mQuat[1] = data.mQuat[1] + ( data.mQuat[0] * data.mGyro[0] + data.mQuat[2] * data.mGyro[2] - data.mQuat[3] * data.mGyro[1]) * halfT;
  data.mQuat[2] = data.mQuat[2] + ( data.mQuat[0] * data.mGyro[1] - data.mQuat[1] * data.mGyro[2] + data.mQuat[3] * data.mGyro[0]) * halfT;
  data.mQuat[3] = data.mQuat[3] + ( data.mQuat[0] * data.mGyro[2] + data.mQuat[1] * data.mGyro[1] - data.mQuat[2] * data.mGyro[0]) * halfT;

  norm = invSqrt(data.mQuat[0] * data.mQuat[0] + data.mQuat[1] * data.mQuat[1] + data.mQuat[2] * data.mQuat[2] + data.mQuat[3] * data.mQuat[3]);
  data.mQuat[0] = data.mQuat[0] * norm;
  data.mQuat[1] = data.mQuat[1] * norm;
  data.mQuat[2] = data.mQuat[2] * norm;
  data.mQuat[3] = data.mQuat[3] * norm;

  quatToAngles(data);
}
