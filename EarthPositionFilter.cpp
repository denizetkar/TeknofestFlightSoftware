#include <limits.h>
#include "EarthPositionFilter.h"

#define G_ACCEL_M_S2 9.807

#ifndef STM32_CORE_VERSION

// getter functions
int64_t EarthPositionFilter::get_pos_mm()
{
  return pos_mm;
}
double EarthPositionFilter::get_vel_mm_per_sec()
{
  return vel_mm_per_sec;
}
void EarthPositionFilter::get_P(double (&P_)[2][2])
{
  memcpy(&P_, &P, sizeof(P));
}
double EarthPositionFilter::get_deltat()
{
  return dt;
}
double EarthPositionFilter::get_proc_var()
{
  return proc_var;
}
double EarthPositionFilter::get_R()
{
  return R;
}
// setter functions
void EarthPositionFilter::set_pos_mm(int64_t _pos_mm)
{
  pos_mm = _pos_mm;
}
void EarthPositionFilter::set_vel_mm_per_sec(double _vel_mm_per_sec)
{
  vel_mm_per_sec = _vel_mm_per_sec;
}
void EarthPositionFilter::set_P(const double (&&P_)[2][2])
{
  memcpy(&P, &P_, sizeof(P));
}
void EarthPositionFilter::set_deltat(double _dt)
{
  dt = _dt;
  dt_dt = _dt * _dt;
  dt_dt_dt_2 = dt_dt * _dt / 2.0;
  dt_dt_dt_dt_4 = dt_dt * dt_dt / 4.0;
}
void EarthPositionFilter::set_proc_var(double _proc_var)
{
  proc_var = _proc_var;
}
void EarthPositionFilter::set_R(double _R)
{
  R = _R;
}

// predict and update functions
void EarthPositionFilter::predict(double accel_in_g)
{
  double temp[2][2], accel_mm_per_sec = (accel_in_g * G_ACCEL_M_S2 * 1000.0);
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
  double K[2], temp, temp2[2][2];
  // calculate kalman matrix, (K)
  residual = actual_pos_mm - pos_mm;
  temp = 1.0 / (P[0][0] + R);
  K[0] = P[0][0] * temp;
  K[1] = P[1][0] * temp;
  // update state vector, (x)
  pos_mm += static_cast<int64_t>( round(K[0] * static_cast<double>(residual)) );
  vel_mm_per_sec += K[1] * static_cast<double>(residual);
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

#else

// getter functions
double EarthPositionFilter::get_pos_m()
{
  return pos_m;
}
double EarthPositionFilter::get_vel_m_per_sec()
{
  return vel_m_per_sec;
}
void EarthPositionFilter::get_P(double (&P_)[2][2])
{
  memcpy(&P_, &P, sizeof(P));
}
double EarthPositionFilter::get_deltat()
{
  return dt;
}
double EarthPositionFilter::get_proc_var()
{
  return proc_var;
}
double EarthPositionFilter::get_R()
{
  return R;
}
// setter functions
void EarthPositionFilter::set_pos_m(double _pos_m)
{
  pos_m = _pos_m;
}
void EarthPositionFilter::set_vel_m_per_sec(double _vel_m_per_sec)
{
  vel_m_per_sec = _vel_m_per_sec;
}
void EarthPositionFilter::set_P(const double (&&P_)[2][2])
{
  memcpy(&P, &P_, sizeof(P));
}
void EarthPositionFilter::set_deltat(double _dt)
{
  dt = _dt;
  dt_dt = _dt * _dt;
  dt_dt_dt_2 = dt_dt * _dt / 2.0;
  dt_dt_dt_dt_4 = dt_dt * dt_dt / 4.0;
}
void EarthPositionFilter::set_proc_var(double _proc_var)
{
  proc_var = _proc_var;
}
void EarthPositionFilter::set_R(double _R)
{
  R = _R;
}

// predict and update functions
void EarthPositionFilter::predict(double accel_in_g)
{
  double temp[2][2], accel_m_per_sec = accel_in_g * G_ACCEL_M_S2;
  // update state vector, (x)
  pos_m += dt * vel_m_per_sec + 0.5 * accel_m_per_sec * dt_dt;
  vel_m_per_sec += dt * accel_m_per_sec;
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
void EarthPositionFilter::update(double actual_pos_m)
{
  double residual, K[2], temp, temp2[2][2];
  // calculate kalman matrix, (K)
  residual = actual_pos_m - pos_m;
  temp = 1.0 / (P[0][0] + R);
  K[0] = P[0][0] * temp;
  K[1] = P[1][0] * temp;
  // update state vector, (x)
  pos_m += K[0] * residual;
  vel_m_per_sec += K[1] * residual;
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

#endif
