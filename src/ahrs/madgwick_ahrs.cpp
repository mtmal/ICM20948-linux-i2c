//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date          Author          Notes
// 29/09/2011    SOH Madgwick    Initial release
// 02/10/2011    SOH Madgwick    Optimised for reduced CPU load
// 19/02/2012    SOH Madgwick    Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include <cmath>
#include "ahrs/ahrs_math.h"
#include "ahrs/madgwick_ahrs.h"

//---------------------------------------------------------------------------------------------------
// Variable definitions

#define beta 0.1f                                            // 2 * proportional gain (Kp)

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate(IMUData& data)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-data.mQuat[1] * data.mGyro[0] - data.mQuat[2] * data.mGyro[1] - data.mQuat[3] * data.mGyro[2]);
    qDot2 = 0.5f * ( data.mQuat[0] * data.mGyro[0] + data.mQuat[2] * data.mGyro[2] - data.mQuat[3] * data.mGyro[1]);
    qDot3 = 0.5f * ( data.mQuat[0] * data.mGyro[1] - data.mQuat[1] * data.mGyro[2] + data.mQuat[3] * data.mGyro[0]);
    qDot4 = 0.5f * ( data.mQuat[0] * data.mGyro[2] + data.mQuat[1] * data.mGyro[1] - data.mQuat[2] * data.mGyro[0]);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((data.mAcc[0] == 0.0f) && (data.mAcc[1] == 0.0f) && (data.mAcc[2] == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(data.mAcc[0] * data.mAcc[0] + data.mAcc[1] * data.mAcc[1] + data.mAcc[2] * data.mAcc[2]);
        data.mAcc[0] *= recipNorm;
        data.mAcc[1] *= recipNorm;
        data.mAcc[2] *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(data.mMag[0] * data.mMag[0] + data.mMag[1] * data.mMag[1] + data.mMag[2] * data.mMag[2]);
        data.mMag[0] *= recipNorm;
        data.mMag[1] *= recipNorm;
        data.mMag[2] *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * data.mQuat[0] * data.mMag[0];
        _2q0my = 2.0f * data.mQuat[0] * data.mMag[1];
        _2q0mz = 2.0f * data.mQuat[0] * data.mMag[2];
        _2q1mx = 2.0f * data.mQuat[1] * data.mMag[0];
        _2q0 = 2.0f * data.mQuat[0];
        _2q1 = 2.0f * data.mQuat[1];
        _2q2 = 2.0f * data.mQuat[2];
        _2q3 = 2.0f * data.mQuat[3];
        _2q0q2 = 2.0f * data.mQuat[0] * data.mQuat[2];
        _2q2q3 = 2.0f * data.mQuat[2] * data.mQuat[3];
        q0q0 = data.mQuat[0] * data.mQuat[0];
        q0q1 = data.mQuat[0] * data.mQuat[1];
        q0q2 = data.mQuat[0] * data.mQuat[2];
        q0q3 = data.mQuat[0] * data.mQuat[3];
        q1q1 = data.mQuat[1] * data.mQuat[1];
        q1q2 = data.mQuat[1] * data.mQuat[2];
        q1q3 = data.mQuat[1] * data.mQuat[3];
        q2q2 = data.mQuat[2] * data.mQuat[2];
        q2q3 = data.mQuat[2] * data.mQuat[3];
        q3q3 = data.mQuat[3] * data.mQuat[3];

        // Reference direction of Earth's magnetic field
        hx = data.mMag[0] * q0q0 - _2q0my * data.mQuat[3] + _2q0mz * data.mQuat[2] + data.mMag[0] * q1q1 + _2q1 * data.mMag[1] * data.mQuat[2] + _2q1 * data.mMag[2] * data.mQuat[3] - data.mMag[0] * q2q2 - data.mMag[0] * q3q3;
        hy = _2q0mx * data.mQuat[3] + data.mMag[1] * q0q0 - _2q0mz * data.mQuat[1] + _2q1mx * data.mQuat[2] - data.mMag[1] * q1q1 + data.mMag[1] * q2q2 + _2q2 * data.mMag[2] * data.mQuat[3] - data.mMag[1] * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * data.mQuat[2] + _2q0my * data.mQuat[1] + data.mMag[2] * q0q0 + _2q1mx * data.mQuat[3] - data.mMag[2] * q1q1 + _2q2 * data.mMag[1] * data.mQuat[3] - data.mMag[2] * q2q2 + data.mMag[2] * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - data.mAcc[0]) + _2q1 * (2.0f * q0q1 + _2q2q3 - data.mAcc[1]) - _2bz * data.mQuat[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - data.mMag[0]) + (-_2bx * data.mQuat[3] + _2bz * data.mQuat[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - data.mMag[1]) + _2bx * data.mQuat[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - data.mMag[2]);
        s1 =  _2q3 * (2.0f * q1q3 - _2q0q2 - data.mAcc[0]) + _2q0 * (2.0f * q0q1 + _2q2q3 - data.mAcc[1]) - 4.0f * data.mQuat[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - data.mAcc[2]) + _2bz * data.mQuat[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - data.mMag[0]) + (_2bx * data.mQuat[2] + _2bz * data.mQuat[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - data.mMag[1]) + (_2bx * data.mQuat[3] - _4bz * data.mQuat[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - data.mMag[2]);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - data.mAcc[0]) + _2q3 * (2.0f * q0q1 + _2q2q3 - data.mAcc[1]) - 4.0f * data.mQuat[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - data.mAcc[2]) + (-_4bx * data.mQuat[2] - _2bz * data.mQuat[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - data.mMag[0]) + (_2bx * data.mQuat[1] + _2bz * data.mQuat[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - data.mMag[1]) + (_2bx * data.mQuat[0] - _4bz * data.mQuat[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - data.mMag[2]);
        s3 =  _2q1 * (2.0f * q1q3 - _2q0q2 - data.mAcc[0]) + _2q2 * (2.0f * q0q1 + _2q2q3 - data.mAcc[1]) + (-_4bx * data.mQuat[3] + _2bz * data.mQuat[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - data.mMag[0]) + (-_2bx * data.mQuat[0] + _2bz * data.mQuat[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - data.mMag[1]) + _2bx * data.mQuat[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - data.mMag[2]);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    data.mQuat[0] += qDot1 * data.mUpdatePeriod;
    data.mQuat[1] += qDot2 * data.mUpdatePeriod;
    data.mQuat[2] += qDot3 * data.mUpdatePeriod;
    data.mQuat[3] += qDot4 * data.mUpdatePeriod;

    // Normalise quaternion
    recipNorm = invSqrt(data.mQuat[0] * data.mQuat[0] + data.mQuat[1] * data.mQuat[1] + data.mQuat[2] * data.mQuat[2] + data.mQuat[3] * data.mQuat[3]);
    data.mQuat[0] *= recipNorm;
    data.mQuat[1] *= recipNorm;
    data.mQuat[2] *= recipNorm;
    data.mQuat[3] *= recipNorm;

    quatToAngles(data);
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(IMUData& data)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-data.mQuat[1] * data.mGyro[0] - data.mQuat[2] * data.mGyro[1] - data.mQuat[3] * data.mGyro[2]);
    qDot2 = 0.5f * ( data.mQuat[0] * data.mGyro[0] + data.mQuat[2] * data.mGyro[2] - data.mQuat[3] * data.mGyro[1]);
    qDot3 = 0.5f * ( data.mQuat[0] * data.mGyro[1] - data.mQuat[1] * data.mGyro[2] + data.mQuat[3] * data.mGyro[0]);
    qDot4 = 0.5f * ( data.mQuat[0] * data.mGyro[2] + data.mQuat[1] * data.mGyro[1] - data.mQuat[2] * data.mGyro[0]);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((data.mAcc[0] == 0.0f) && (data.mAcc[1] == 0.0f) && (data.mAcc[2] == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(data.mAcc[0] * data.mAcc[0] + data.mAcc[1] * data.mAcc[1] + data.mAcc[2] * data.mAcc[2]);
        data.mAcc[0] *= recipNorm;
        data.mAcc[1] *= recipNorm;
        data.mAcc[2] *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * data.mQuat[0];
        _2q1 = 2.0f * data.mQuat[1];
        _2q2 = 2.0f * data.mQuat[2];
        _2q3 = 2.0f * data.mQuat[3];
        _4q0 = 4.0f * data.mQuat[0];
        _4q1 = 4.0f * data.mQuat[1];
        _4q2 = 4.0f * data.mQuat[2];
        _8q1 = 8.0f * data.mQuat[1];
        _8q2 = 8.0f * data.mQuat[2];
        q0q0 = data.mQuat[0] * data.mQuat[0];
        q1q1 = data.mQuat[1] * data.mQuat[1];
        q2q2 = data.mQuat[2] * data.mQuat[2];
        q3q3 = data.mQuat[3] * data.mQuat[3];

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * data.mAcc[0] + _4q0 * q1q1 - _2q1 * data.mAcc[1];
        s1 = _4q1 * q3q3 - _2q3 * data.mAcc[0] + 4.0f * q0q0 * data.mQuat[1] - _2q0 * data.mAcc[1] - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * data.mAcc[2];
        s2 = 4.0f * q0q0 * data.mQuat[2] + _2q0 * data.mAcc[0] + _4q2 * q3q3 - _2q3 * data.mAcc[1] - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * data.mAcc[2];
        s3 = 4.0f * q1q1 * data.mQuat[3] - _2q1 * data.mAcc[0] + 4.0f * q2q2 * data.mQuat[3] - _2q2 * data.mAcc[1];
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    data.mQuat[0] += qDot1 * data.mUpdatePeriod;
    data.mQuat[1] += qDot2 * data.mUpdatePeriod;
    data.mQuat[2] += qDot3 * data.mUpdatePeriod;
    data.mQuat[3] += qDot4 * data.mUpdatePeriod;

    // Normalise quaternion
    recipNorm = invSqrt(data.mQuat[0] * data.mQuat[0] + data.mQuat[1] * data.mQuat[1] + data.mQuat[2] * data.mQuat[2] + data.mQuat[3] * data.mQuat[3]);
    data.mQuat[0] *= recipNorm;
    data.mQuat[1] *= recipNorm;
    data.mQuat[2] *= recipNorm;
    data.mQuat[3] *= recipNorm;

    quatToAngles(data);
}

//====================================================================================================
// END OF CODE
//====================================================================================================
