#include <limits.h>
#include "EarthPositionFilter.h"

// latitude & longitude in degrees into x,y 
// coordinates in centimeters on the surface of earth
void lat_lon_to_x_y_cm(float lat, float lon, int32_t &x, int32_t &y)
{
  float lat_rad = radians(lat);
  x = static_cast<int32_t>( round((((111412.84 * cos(lat_rad) - 93.5 * cos(3.0 * lat_rad)) + 0.118 * cos(5.0 * lat_rad)) * lon) * 100.0) );
  y = static_cast<int32_t>( round(1000196572.93 * (lat / 90.0)) );
}
int32_t safe_int_add(int32_t a, int32_t b)
{
  int32_t sum;
  sum = a + b;
  // out-of-range only possible when the signs are the same.
  if ((a < 0) == (b < 0)) {
    if (a < 0) {
      // Underflow here means the result is too negative.
      if (sum > b) sum = LONG_MIN;
    }
    else {
      // Overflow here means the result is too positive
      if (sum < b) sum = LONG_MAX;  
    }
  }
  return sum;
}


// getter functions
int32_t EarthPositionFilter::get_pos_cm()
{
  return pos_cm;
}
float EarthPositionFilter::get_vel_cm_per_sec()
{
  return vel_cm_per_sec;
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
void EarthPositionFilter::set_pos_cm(int32_t _pos_cm)
{
  pos_cm = _pos_cm;
}
void EarthPositionFilter::set_vel_cm_per_sec(float _vel_cm_per_sec)
{
  vel_cm_per_sec = _vel_cm_per_sec;
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
  float temp[2][2];
  // update state vector, (x)
  pos_cm = safe_int_add(pos_cm, static_cast<int32_t>( round(dt * vel_cm_per_sec + 0.5 * (accel_m_per_sec * 100.0) * dt_dt) ));
  vel_cm_per_sec += dt * (accel_m_per_sec * 100.0);
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
void EarthPositionFilter::update(int32_t actual_pos_cm)
{
  int32_t residual;
  float K[2], temp, temp2[2][2];
  // calculate kalman matrix, (K)
  residual = actual_pos_cm - pos_cm;
  temp = 1.0 / (P[0][0] + R);
  K[0] = P[0][0] * temp;
  K[1] = P[1][0] * temp;
  // update state vector, (x)
  pos_cm = safe_int_add(pos_cm, static_cast<int32_t>( round(K[0] * static_cast<float>(residual)) ));
  vel_cm_per_sec += K[1] * static_cast<float>(residual);
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
