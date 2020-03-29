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
#include "QuaternionPID.h"
//---------------------------------------------------------------------------------------------------
// Definitions

#define ROTATIONAL_EPSILON 1e-5
//---------------------------------------------------------------------------------------------------
// Variable definitions

double madgwick_beta = MadgwickBetaDef;               // 2 * proportional gain (Kp)
double q_a[4] = { 1.0, 0.0, 0.0, 0.0 };  // quaternion of sensor frame relative to auxiliary frame

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz, double deltat) {
  double recipNorm;
  double s_a[4];   // errors from absolute orientation
  double qDot1, qDot2, qDot3, qDot4;
  double hx, hy;
  double _2q0mx = 0.0, _2q0my = 0.0, _2q0mz = 0.0, _2q1mx = 0.0, _2bx = 0.0, _2bz = 0.0, 
        _2q1, _2q2, q0q0, q0q1, q0q2, q1q1, q1q3, q2q2, q3q3, fg0 = 0.0, fg1 = 0.0, fg2 = 0.0, fb0 = 0.0, fb1 = 0.0, fb2 = 0.0;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5 * (-q_a[1] * gx - q_a[2] * gy - q_a[3] * gz);
  qDot2 = 0.5 * (q_a[0] * gx + q_a[2] * gz - q_a[3] * gy);
  qDot3 = 0.5 * (q_a[0] * gy - q_a[1] * gz + q_a[3] * gx);
  qDot4 = 0.5 * (q_a[0] * gz + q_a[1] * gy - q_a[2] * gx);

  // Auxiliary variables to avoid repeated arithmetic
  _2q1 = 2.0 * q_a[1];
  _2q2 = 2.0 * q_a[2];
  q0q0 = q_a[0] * q_a[0];
  q0q1 = q_a[0] * q_a[1];
  q0q2 = q_a[0] * q_a[2];
  q1q1 = q_a[1] * q_a[1];
  q1q3 = q_a[1] * q_a[3];
  q2q2 = q_a[2] * q_a[2];
  q3q3 = q_a[3] * q_a[3];

  // Compute feedback only if magnetometer measurement valid (avoids NaN in magnetometer normalisation)
  if(!((mx == 0.0) && (my == 0.0) && (mz == 0.0))) {
    
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

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q_a[3] + _2q0mz * q_a[2] + mx * q1q1 + _2q1 * my * q_a[2] + _2q1 * mz * q_a[3] - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q_a[3] + my * q0q0 - _2q0mz * q_a[1] + _2q1mx * q_a[2] - my * q1q1 + my * q2q2 + _2q2 * mz * q_a[3] - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q_a[2] + _2q0my * q_a[1] + mz * q0q0 + _2q1mx * q_a[3] - mz * q1q1 + _2q2 * my * q_a[3] - mz * q2q2 + mz * q3q3;

    // objective function vector for magnetometer
    fb0 = _2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx;
    fb1 = _2bx * (q_a[1] * q_a[2] - q_a[0] * q_a[3]) + _2bz * (q0q1 + q_a[2] * q_a[3]) - my;
    fb2 = _2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

    // Normalise accelerometer measurement
    recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // objective function vector for accelerometer
    fg0 = 2.0 * q1q3 - 2.0 * q_a[0] * q_a[2] - ax;
    fg1 = 2.0 * q0q1 + 2.0 * q_a[2] * q_a[3] - ay;
    fg2 = 1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az;
  }

  // Gradient decent algorithm corrective step
  s_a[0] = -_2q2 * fg0 + _2q1 * fg1 - _2bz * q_a[2] * fb0 + (-_2bx * q_a[3] + _2bz * q_a[1]) * fb1 + _2bx * q_a[2] * fb2;
  s_a[1] = 2.0 * q_a[3] * fg0 + 2.0 * q_a[0] * fg1 - 4.0 * q_a[1] * fg2 + _2bz * q_a[3] * fb0 + (_2bx * q_a[2] + _2bz * q_a[0]) * fb1 + (_2bx * q_a[3] - 2.0 * _2bz * q_a[1]) * fb2;
  s_a[2] = -2.0 * q_a[0] * fg0 + 2.0 * q_a[3] * fg1 - 4.0 * q_a[2] * fg2 + (-2.0 * _2bx * q_a[2] - _2bz * q_a[0]) * fb0 + (_2bx * q_a[1] + _2bz * q_a[3]) * fb1 + (_2bx * q_a[0] - 2.0 * _2bz * q_a[2]) * fb2;
  s_a[3] = _2q1 * fg0 + _2q2 * fg1 + (-2.0 * _2bx * q_a[3] + _2bz * q_a[1]) * fb0 + (-_2bx * q_a[0] + _2bz * q_a[2]) * fb1 + _2bx * q_a[1] * fb2;

  if(!((s_a[0] == 0.0) && (s_a[1] == 0.0) && (s_a[2]== 0.0) && (s_a[3]== 0.0))) {
    // normalise step magnitude
    recipNorm = 1.0 / sqrt(s_a[0] * s_a[0] + s_a[1] * s_a[1] + s_a[2] * s_a[2] + s_a[3] * s_a[3]);
  
    // Apply feedback step
    qDot1 -= madgwick_beta * s_a[0] * recipNorm;
    qDot2 -= madgwick_beta * s_a[1] * recipNorm;
    qDot3 -= madgwick_beta * s_a[2] * recipNorm;
    qDot4 -= madgwick_beta * s_a[3] * recipNorm;
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

void MadgwickGYROupdate(double gx, double gy, double gz, double deltat)
{
  double g_mag = sqrt(gx*gx + gy*gy + gz*gz), tmp, q_diff[4], q_a_new[4];

  if (g_mag < ROTATIONAL_EPSILON) {
    MadgwickAHRSupdate(gx, gy, gz,
                       0, 0, 0,
                       0, 0, 0, deltat);
    return;
  }

  // https://math.stackexchange.com/a/39565/493974
  tmp = g_mag * deltat / 2.0;
  gx /= g_mag; gy /= g_mag; gz /= g_mag;
  q_diff[0] = cos(tmp);
  tmp = sin(tmp);
  q_diff[1] = tmp * gx;
  q_diff[2] = tmp * gy;
  q_diff[3] = tmp * gz;
  if (q_diff[0] < 0.0) {
    q_diff[0] = -q_diff[0];
    q_diff[1] = -q_diff[1];
    q_diff[2] = -q_diff[2];
    q_diff[3] = -q_diff[3];
  }
  quaternion_prod(q_a, q_diff, q_a_new);
  if (q_a_new[0] < 0.0) {
    q_a[0] = -q_a_new[0];
    q_a[1] = -q_a_new[1];
    q_a[2] = -q_a_new[2];
    q_a[3] = -q_a_new[3];
  } else {
    q_a[0] = q_a_new[0];
    q_a[1] = q_a_new[1];
    q_a[2] = q_a_new[2];
    q_a[3] = q_a_new[3];
  }
}

//====================================================================================================
// END OF CODE
//====================================================================================================
