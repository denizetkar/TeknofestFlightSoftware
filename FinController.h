#ifndef FIN_CONTROLLER_H
#define FIN_CONTROLLER_H

//#define FIN_CONTROL_BY_STEP_MOTOR
#define FIN_CONTROL_BY_SERVO

#if defined(FIN_CONTROL_BY_STEP_MOTOR)
#include <AccelStepper.h>
#elif defined(FIN_CONTROL_BY_SERVO)
#include <Servo.h>
#define SERVO_ZERO_ANGLE 90
#endif

class FinController {
  protected:
#if defined(FIN_CONTROL_BY_STEP_MOTOR)
    AccelStepper stepper0, stepper1, stepper2, stepper3;
#elif defined(FIN_CONTROL_BY_SERVO)
    Servo servo0, servo1, servo2, servo3;
#endif

  public:
#if defined(FIN_CONTROL_BY_STEP_MOTOR)
    FinController(const uint8_t(&&)[4], const uint8_t(&&)[4],
                  const uint8_t(&&)[4], const uint8_t(&&)[4]);
    void makeFinCorrections(float, float, float);
    void runSteppers();
#elif defined(FIN_CONTROL_BY_SERVO)
    FinController(uint8_t, uint8_t, uint8_t, uint8_t);
    void makeFinCorrections(float, float, float);
#endif
};

#endif
