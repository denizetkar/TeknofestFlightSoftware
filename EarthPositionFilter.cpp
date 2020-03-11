#include <limits.h>
#include "EarthPositionFilter.h"

// getter functions
int64_t EarthPositionFilter::get_pos_mm()
{
  return pos_mm;
}
float EarthPositionFilter::get_vel_mm_per_sec()
{
  return vel_mm_per_sec;
}
void EarthPositionFilter::get_P(float (&_P)[2][2])
{
  memcpy(&_P, &P, sizeof(P));
}
float EarthPositionFilter::get_deltat()
{
  return dt;
}
float EarthPositionFilter::get_proc_var()
{
  return proc_var;
}
float EarthPositionFilter::get_R()
{
  return R;
}
// setter functions
void EarthPositionFilter::set_pos_mm(int64_t _pos_mm)
{
  pos_mm = _pos_mm;
}
void EarthPositionFilter::set_vel_mm_per_sec(float _vel_mm_per_sec)
{
  vel_mm_per_sec = _vel_mm_per_sec;
}
void EarthPositionFilter::set_P(const float (&&_P)[2][2])
{
  memcpy(&P, &_P, sizeof(P));
}
void EarthPositionFilter::set_deltat(float _dt)
{
  dt = _dt;
  dt_dt = _dt * _dt;
  dt_dt_dt_2 = dt_dt * _dt / 2.0;
  dt_dt_dt_dt_4 = dt_dt * dt_dt / 4.0;
}
void EarthPositionFilter::set_proc_var(float _proc_var)
{
  proc_var = _proc_var;
}
void EarthPositionFilter::set_R(float _R)
{
  R = _R;
}

// predict and update functions
void EarthPositionFilter::predict(float accel_m_per_sec)
{
  float temp[2][2], accel_mm_per_sec = (accel_m_per_sec * 1000.0);
  // update state vector, (x)
  pos_mm += static_cast<int64_t>( round(dt * vel_mm_per_sec + 0.5 * accel_mm_per_sec * dt_dt) );
  vel_mm_per_sec += dt * accel_mm_per_sec;
  // update state covariance matrix, (P)
  // P = F*P
  temp[0][0] = P[0][0] + dt * P[1][0];
  temp[0][1] = P[0][1] + dt * P[1][1];
  temp[1][0] = P[1][0];
  temp[1][1] = P[1][1];
  memcpy(&P, &temp, sizeof(temp));
  // P = P*F_t
  temp[0][0] = P[0][0] + P[0][1] * dt;
  temp[0][1] = P[0][1];
  temp[1][0] = P[1][0] + P[1][1] * dt;
  temp[1][1] = P[1][1];
  memcpy(&P, &temp, sizeof(temp));
  // P = P + Q
  P[0][0] += dt_dt_dt_dt_4 * proc_var;
  P[0][1] += dt_dt_dt_2 * proc_var;
  P[1][0] += dt_dt_dt_2 * proc_var;
  P[1][1] += dt_dt * proc_var;
}
void EarthPositionFilter::update(int64_t actual_pos_mm)
{
  int64_t residual;
  float K[2], temp, temp2[2][2];
  // calculate kalman matrix, (K)
  residual = actual_pos_mm - pos_mm;
  temp = 1.0 / (P[0][0] + R);
  K[0] = P[0][0] * temp;
  K[1] = P[1][0] * temp;
  // update state vector, (x)
  pos_mm += static_cast<int64_t>( round(K[0] * static_cast<float>(residual)) );
  vel_mm_per_sec += K[1] * static_cast<float>(residual);
  //update state covariance matrix, (P)
  // P = (I - KH)*P
  temp2[0][0] = (1.0 - K[0]) * P[0][0];
  temp2[0][1] = (1.0 - K[0]) * P[0][1];
  temp2[1][0] = -K[1] * P[0][0] + P[1][0];
  temp2[1][1] = -K[1] * P[0][1] + P[1][1];
  memcpy(&P, &temp2, sizeof(temp2));
  // P = (P+P_t)/2
  P[0][1] = (P[0][1] + P[1][0]) / 2.0;
  P[1][0] = P[0][1];
}
