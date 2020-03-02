//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date      Author          Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
// 19/02/2012 SOH Madgwick  Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include <Arduino.h>
#include "MadgwickAHRS.h"

//---------------------------------------------------------------------------------------------------
// Definitions

#define betaDef   0.1   // 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

float beta = betaDef;               // 2 * proportional gain (Kp)
float q_a[4] = { 1.0, 0.0, 0.0, 0.0 };  // quaternion of sensor frame relative to auxiliary frame

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltat) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _2q1, _2q2, q0q0, q0q1, q0q2, q1q1, q1q3, q2q2, q3q3, fg0, fg1, fg2, fb0, fb1, fb2;
  bool isAccelGravity;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, deltat);
    return;
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5 * (-q_a[1] * gx - q_a[2] * gy - q_a[3] * gz);
  qDot2 = 0.5 * (q_a[0] * gx + q_a[2] * gz - q_a[3] * gy);
  qDot3 = 0.5 * (q_a[0] * gy - q_a[1] * gz + q_a[3] * gx);
  qDot4 = 0.5 * (q_a[0] * gz + q_a[1] * gy - q_a[2] * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

    // Check if magnitude is close to 1.0
    recipNorm = sqrt(ax * ax + ay * ay + az * az);
    isAccelGravity = (abs(recipNorm - 1.0) <= 0.05);

    // Normalise accelerometer measurement
    recipNorm = 1.0 / recipNorm;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = 1.0 / sqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0 * q_a[0] * mx;
    _2q0my = 2.0 * q_a[0] * my;
    _2q0mz = 2.0 * q_a[0] * mz;
    _2q1mx = 2.0 * q_a[1] * mx;
    _2q1 = 2.0 * q_a[1];
    _2q2 = 2.0 * q_a[2];
    q0q0 = q_a[0] * q_a[0];
    q0q1 = q_a[0] * q_a[1];
    q0q2 = q_a[0] * q_a[2];
    q1q1 = q_a[1] * q_a[1];
    q1q3 = q_a[1] * q_a[3];
    q2q2 = q_a[2] * q_a[2];
    q3q3 = q_a[3] * q_a[3];

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q_a[3] + _2q0mz * q_a[2] + mx * q1q1 + _2q1 * my * q_a[2] + _2q1 * mz * q_a[3] - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q_a[3] + my * q0q0 - _2q0mz * q_a[1] + _2q1mx * q_a[2] - my * q1q1 + my * q2q2 + _2q2 * mz * q_a[3] - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q_a[2] + _2q0my * q_a[1] + mz * q0q0 + _2q1mx * q_a[3] - mz * q1q1 + _2q2 * my * q_a[3] - mz * q2q2 + mz * q3q3;

    // objective function vector for accelerometer and magnetometer
    fg0 = isAccelGravity ? (2.0 * q1q3 - 2.0 * q_a[0] * q_a[2] - ax) : 0.0;
    fg1 = isAccelGravity ? (2.0 * q0q1 + 2.0 * q_a[2] * q_a[3] - ay) : 0.0;
    fg2 = isAccelGravity ? (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az) : 0.0;
    fb0 = _2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx;
    fb1 = _2bx * (q_a[1] * q_a[2] - q_a[0] * q_a[3]) + _2bz * (q0q1 + q_a[2] * q_a[3]) - my;
    fb2 = _2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * fg0 + _2q1 * fg1 - _2bz * q_a[2] * fb0 + (-_2bx * q_a[3] + _2bz * q_a[1]) * fb1 + _2bx * q_a[2] * fb2;
    s1 = 2.0 * q_a[3] * fg0 + 2.0 * q_a[0] * fg1 - 4.0 * q_a[1] * fg2 + _2bz * q_a[3] * fb0 + (_2bx * q_a[2] + _2bz * q_a[0]) * fb1 + (_2bx * q_a[3] - 2.0 * _2bz * q_a[1]) * fb2;
    s2 = -2.0 * q_a[0] * fg0 + 2.0 * q_a[3] * fg1 - 4.0 * q_a[2] * fg2 + (-2.0 * _2bx * q_a[2] - _2bz * q_a[0]) * fb0 + (_2bx * q_a[1] + _2bz * q_a[3]) * fb1 + (_2bx * q_a[0] - 2.0 * _2bz * q_a[2]) * fb2;
    s3 = _2q1 * fg0 + _2q2 * fg1 + (-2.0 * _2bx * q_a[3] + _2bz * q_a[1]) * fb0 + (-_2bx * q_a[0] + _2bz * q_a[2]) * fb1 + _2bx * q_a[1] * fb2;
    recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
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
  q_a[0] += qDot1 * deltat;
  q_a[1] += qDot2 * deltat;
  q_a[2] += qDot3 * deltat;
  q_a[3] += qDot4 * deltat;

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q_a[0] * q_a[0] + q_a[1] * q_a[1] + q_a[2] * q_a[2] + q_a[3] * q_a[3]);
  q_a[0] *= recipNorm;
  q_a[1] *= recipNorm;
  q_a[2] *= recipNorm;
  q_a[3] *= recipNorm;

  if (q_a[0] < 0.0) {
    q_a[0] = -q_a[0];
    q_a[1] = -q_a[1];
    q_a[2] = -q_a[2];
    q_a[3] = -q_a[3];
  }
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float deltat) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5 * (-q_a[1] * gx - q_a[2] * gy - q_a[3] * gz);
  qDot2 = 0.5 * (q_a[0] * gx + q_a[2] * gz - q_a[3] * gy);
  qDot3 = 0.5 * (q_a[0] * gy - q_a[1] * gz + q_a[3] * gx);
  qDot4 = 0.5 * (q_a[0] * gz + q_a[1] * gy - q_a[2] * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

    // Check if magnitude is close to 1.0
    recipNorm = sqrt(ax * ax + ay * ay + az * az);
    if (abs(recipNorm - 1.0) <= 0.05) {

      // Normalise accelerometer measurement
      recipNorm = 1.0 / recipNorm;
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Auxiliary variables to avoid repeated arithmetic
      _2q0 = 2.0 * q_a[0];
      _2q1 = 2.0 * q_a[1];
      _2q2 = 2.0 * q_a[2];
      _2q3 = 2.0 * q_a[3];
      _4q0 = 4.0 * q_a[0];
      _4q1 = 4.0 * q_a[1];
      _4q2 = 4.0 * q_a[2];
      _8q1 = 8.0 * q_a[1];
      _8q2 = 8.0 * q_a[2];
      q0q0 = q_a[0] * q_a[0];
      q1q1 = q_a[1] * q_a[1];
      q2q2 = q_a[2] * q_a[2];
      q3q3 = q_a[3] * q_a[3];

      // Gradient decent algorithm corrective step
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q_a[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0 * q0q0 * q_a[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0 * q1q1 * q_a[3] - _2q1 * ax + 4.0 * q2q2 * q_a[3] - _2q2 * ay;
      recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
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
  }

  // Integrate rate of change of quaternion to yield quaternion
  q_a[0] += qDot1 * deltat;
  q_a[1] += qDot2 * deltat;
  q_a[2] += qDot3 * deltat;
  q_a[3] += qDot4 * deltat;

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q_a[0] * q_a[0] + q_a[1] * q_a[1] + q_a[2] * q_a[2] + q_a[3] * q_a[3]);
  q_a[0] *= recipNorm;
  q_a[1] *= recipNorm;
  q_a[2] *= recipNorm;
  q_a[3] *= recipNorm;

  if (q_a[0] < 0.0) {
    q_a[0] = -q_a[0];
    q_a[1] = -q_a[1];
    q_a[2] = -q_a[2];
    q_a[3] = -q_a[3];
  }
}

//====================================================================================================
// END OF CODE
//====================================================================================================
