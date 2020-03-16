#ifndef QUATERNION_PID_H
#define QUATERNION_PID_H

void rotate_vector_by_quaternion(const float (&)[4], float, float, float, float&, float&, float&);
void quaternion_prod(const float (&)[4], const float (&)[4], float (&)[4]);

class QuaternionPID {
  protected:
    float Kp, Ki, Kd;
    float integrals[3] = { 0.0, 0.0, 0.0 };
    float q_d[4] = {1.0, 0.0, 0.0, 0.0};
  public:
    static constexpr float Xmin = -20.0, Xmax = 20.0,
                           Ymin = -20.0, Ymax = 20.0,
                           Zmin = -20.0, Zmax = 20.0;

    QuaternionPID(float _Kp, float _Ki, float _Kd);

    // getter and setter functions
    void getGains(float&, float&, float&);
    void setGains(float, float, float);
    void getDesiredQuaternion(float (&)[4]);
    void setDesiredQuaternion(const float (&&)[4]);

    // actual pid compute function
    void compute(const float (&)[4], float, float, float, float&, float&, float&);
};

#endif
