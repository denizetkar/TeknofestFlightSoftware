#ifndef QUATERNION_PID_H
#define QUATERNION_PID_H

void rotate_vector_by_quaternion(const double (&)[4], double, double, double, double&, double&, double&);
void quaternion_prod(const double (&)[4], const double (&)[4], double (&)[4]);

class QuaternionPID {
  protected:
    double Kp, Ki, Kd;
    double integrals[3] = { 0.0, 0.0, 0.0 };
    double q_d[4] = {1.0, 0.0, 0.0, 0.0};
  public:
    static constexpr double Xmin = -20.0, Xmax = 20.0,
                           Ymin = -20.0, Ymax = 20.0,
                           Zmin = -20.0, Zmax = 20.0;

    QuaternionPID(double _Kp, double _Ki, double _Kd);

    // getter and setter functions
    void getGains(double&, double&, double&);
    void setGains(double, double, double);
    void getDesiredQuaternion(double (&)[4]);
    void setDesiredQuaternion(const double (&&)[4]);

    // actual pid compute function
    void compute(const double (&)[4], double, double, double, double&, double&, double&);
};

#endif
