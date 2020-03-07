#ifndef QUATERNION_PID_H
#define QUATERNION_PID_H

void rotate_vector_by_quaternion(const float (&)[4], float, float, float, float&, float&, float&);
void quaternion_prod(const float (&)[4], const float (&)[4], float (&)[4]);

class QuaternionPID {
  protected:
    float Kp, Ki, Kd;
    float integrals[3] = { 0.0, 0.0, 0.0 };
    float q_d[4] = {1.0, 0.0, 0.0, 0.0};
    float Xmin, Xmax, Ymin, Ymax, Zmin, Zmax;

  public:
    QuaternionPID(float _Kp, float _Ki, float _Kd,
      float _Xmin = -15.0, float _Xmax = 15.0,
      float _Ymin = -15.0, float _Ymax = 15.0,
      float _Zmin = -15.0, float _Zmax = 15.0);

    // getter and setter functions
    void getGains(float&, float&, float&);
    void setGains(float, float, float);
    void getDesiredQuaternion(float (&)[4]);
    void setDesiredQuaternion(const float (&&)[4]);
    void getOutputLimitsX(float&, float&);
    void getOutputLimitsY(float&, float&);
    void getOutputLimitsZ(float&, float&);
    void setOutputLimitsX(float, float);
    void setOutputLimitsY(float, float);
    void setOutputLimitsZ(float, float);

    // actual pid compute function
    void compute(const float (&)[4], float, float, float, float&, float&, float&);
};

#endif
