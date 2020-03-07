#include <Arduino.h>
#include "QuaternionPID.h"

void rotate_vector_by_quaternion(
  const float (&q)[4],
  float _vx, float _vy, float _vz,
  float& rx, float& ry, float& rz)
{
  // rotates vector (_vx, _vy, _vz) by unit quaternion q
  rx = _vx * (0.5 - q[2] * q[2] - q[3] * q[3])
       + _vy * (-q[0] * q[3] + q[1] * q[2])
       + _vz * (q[0] * q[2] + q[1] * q[3]);
  ry = _vx * (q[1] * q[2] + q[0] * q[3])
       + _vy * (0.5 - q[1] * q[1] - q[3] * q[3])
       + _vz * (-q[0] * q[1] + q[2] * q[3]);
  rz = _vx * (-q[0] * q[2] + q[1] * q[3])
       + _vy * (q[0] * q[1] + q[2] * q[3])
       + _vz * (0.5 - q[1] * q[1] - q[2] * q[2]);
  rx *= 2.0; ry *= 2.0; rz *= 2.0;
}

void quaternion_prod(const float (&q_a)[4], const float (&q_b)[4], float (&res)[4]) {
  res[0] = q_a[0] * q_b[0] - q_a[1] * q_b[1] - q_a[2] * q_b[2] - q_a[3] * q_b[3];
  res[1] = q_a[0] * q_b[1] + q_a[1] * q_b[0] + q_a[2] * q_b[3] - q_a[3] * q_b[2];
  res[2] = q_a[0] * q_b[2] - q_a[1] * q_b[3] + q_a[2] * q_b[0] + q_a[3] * q_b[1];
  res[3] = q_a[0] * q_b[3] + q_a[1] * q_b[2] - q_a[2] * q_b[1] + q_a[3] * q_b[0];
}

// constructor
QuaternionPID::QuaternionPID(float _Kp, float _Ki, float _Kd,
  float _Xmin, float _Xmax,
  float _Ymin, float _Ymax,
  float _Zmin, float _Zmax) :
  Kp { _Kp }, Ki { _Ki }, Kd { _Kd },
  Xmin { _Xmin }, Xmax { _Xmax },
  Ymin { _Ymin }, Ymax { _Ymax },
  Zmin { _Zmin }, Zmax { _Zmax }
{
  if (_Kp < 0.0) {
    Kp = 0.0;
  }
  if (_Ki < 0.0) {
    Ki = 0.0;
  }
  if (_Kd < 0.0) {
    Kd = 0.0;
  }
}

// getter and setter functions
void QuaternionPID::getGains(float &_Kp, float &_Ki, float &_Kd) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
}
void QuaternionPID::setGains(float _Kp, float _Ki, float _Kd) {
  if (_Kp < 0.0 || _Kd < 0.0) return;
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
}
void QuaternionPID::getDesiredQuaternion(float (&_q_d)[4]) {
  memcpy(&_q_d, &q_d, sizeof(q_d));
}
void QuaternionPID::setDesiredQuaternion(const float (&&_q_d)[4]) {
  memcpy(&q_d, &_q_d, sizeof(q_d));
}
void QuaternionPID::getOutputLimitsX(float &_Xmin, float &_Xmax) {
  _Xmin = Xmin;
  _Xmax = Xmax;
}
void QuaternionPID::getOutputLimitsY(float &_Ymin, float &_Ymax) {
  _Ymin = Ymin;
  _Ymax = Ymax;
}
void QuaternionPID::getOutputLimitsZ(float &_Zmin, float &_Zmax) {
  _Zmin = Zmin;
  _Zmax = Zmax;
}
void QuaternionPID::setOutputLimitsX(float _Xmin, float _Xmax) {
  if (_Xmin >= _Xmax) return;
  Xmin = _Xmin;
  Xmax = _Xmax;
}
void QuaternionPID::setOutputLimitsY(float _Ymin, float _Ymax) {
  if (_Ymin >= _Ymax) return;
  Ymin = _Ymin;
  Ymax = _Ymax;
}
void QuaternionPID::setOutputLimitsZ(float _Zmin, float _Zmax) {
  if (_Zmin >= _Zmax) return;
  Zmin = _Zmin;
  Zmax = _Zmax;
}

// actual pid compute function
void QuaternionPID::compute(const float (&q_a)[4], float gx, float gy, float gz, float &ux, float &uy, float &uz) {
  float q_e[4], q_e_coeff, proportionals[3];
  // q_e = q_a^-1 * q_d
  q_e[0] = q_a[0] * q_d[0] + q_a[1] * q_d[1] + q_a[2] * q_d[2] + q_a[3] * q_d[3];
  q_e[1] = q_a[0] * q_d[1] - q_a[1] * q_d[0] - q_a[2] * q_d[3] + q_a[3] * q_d[2];
  q_e[2] = q_a[0] * q_d[2] + q_a[1] * q_d[3] - q_a[2] * q_d[0] - q_a[3] * q_d[1];
  q_e[3] = q_a[0] * q_d[3] - q_a[1] * q_d[2] + q_a[2] * q_d[1] - q_a[3] * q_d[0];

  if (q_e[0] < 0.0) {
    q_e[0] = -q_e[0];
    q_e[1] = -q_e[1];
    q_e[2] = -q_e[2];
    q_e[3] = -q_e[3];
  }

  if (1.0 - q_e[0] < 1e-4) {
    ux = constrain(-Kd * gx + integrals[0], Xmin, Xmax);
    uy = constrain(-Kd * gy + integrals[1], Ymin, Ymax);
    uz = constrain(-Kd * gz + integrals[2], Zmin, Zmax);
  } else {
    q_e_coeff = 2.0 * acos(q_e[0]) / sqrt(1.0 - q_e[0] * q_e[0]);
    proportionals[0] = q_e_coeff * q_e[1];
    proportionals[1] = q_e_coeff * q_e[2];
    proportionals[2] = q_e_coeff * q_e[3];
    integrals[0] = constrain(integrals[0] + Ki * proportionals[0], Xmin, Xmax);
    integrals[1] = constrain(integrals[1] + Ki * proportionals[1], Xmin, Xmax);
    integrals[2] = constrain(integrals[2] + Ki * proportionals[2], Xmin, Xmax);
    ux = constrain(Kp * proportionals[0] - Kd * gx + integrals[0], Xmin, Xmax);
    uy = constrain(Kp * proportionals[1] - Kd * gy + integrals[1], Ymin, Ymax);
    uz = constrain(Kp * proportionals[2] - Kd * gz + integrals[2], Zmin, Zmax);
  }
}
