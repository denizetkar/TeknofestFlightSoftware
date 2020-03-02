#ifndef QUATERNION_PID_H
#define QUATERNION_PID_H

void rotate_vector_by_quaternion(const float (&)[4], float, float, float, float&, float&, float&);
void quaternion_prod(const float (&)[4], const float (&)[4], float (&)[4]);

class QuaternionPID {
  protected:
    float Kp, Kd;
    float q_d[4] = {1.0, 0.0, 0.0, 0.0};
    float Xmin, Xmax, Ymin, Ymax, Zmin, Zmax;

  public:
    QuaternionPID(float _Kp, float _Kd,
      float _Xmin = -128.0, float _Xmax = 128.0,
      float _Ymin = -128.0, float _Ymax = 128.0,
      float _Zmin = -128.0, float _Zmax = 128.0);

    // getter and setter functions
    void getGains(float&, float&);
    void setGains(float, float);
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
