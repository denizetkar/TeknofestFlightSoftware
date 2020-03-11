#ifndef EARTH_POSITION_FILTER_H
#define EARTH_POSITION_FILTER_H

#include <Arduino.h>

class EarthPositionFilter
{
  protected:
    // state variables (position, velocity), (x)
    int64_t pos_mm;
    float vel_mm_per_sec;
    // state covariance matrix, (P)
    float P[2][2];
    // state transition matrix, (F)
    // { {1, dt}, {0, 1} }
    float dt;
    // control input matrix, (B)
    // { 0.5*a*dt^2, dt }
    float dt_dt;
    // process noise matrix, (Q)
    // { {0.25*dt^4, 0.5*dt^3}, {0.5*dt^3, dt^2} } * process_variance
    float dt_dt_dt_2, dt_dt_dt_dt_4;
    float proc_var;
    // measurement matrix, (H)
    // { 1, 0 }
    // measurement noise matrix, (R)
    float R;

  public:
    EarthPositionFilter() :
      pos_mm{0}, vel_mm_per_sec{0.0},
      P{10000000.0, 0.0, 0.0, 10000000.0},
      proc_var{1000000.0}, R{4000000.0} {}
    // getter functions
    int64_t get_pos_mm();
    float   get_vel_mm_per_sec();
    void    get_P(float (&)[2][2]);
    float   get_deltat();
    float   get_proc_var();
    float   get_R();
    // setter functions
    void    set_pos_mm(int64_t);
    void    set_vel_mm_per_sec(float);
    void    set_P(const float (&&)[2][2]);
    void    set_deltat(float);
    void    set_proc_var(float);
    void    set_R(float);

    // predict and update functions
    void predict(float);
    void update(int64_t);
};

#endif  // EARTH_POSITION_FILTER_H
