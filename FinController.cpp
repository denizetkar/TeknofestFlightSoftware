#include "FinController.h"

#if defined(FIN_CONTROL_BY_STEP_MOTOR)

#define STEP_MOTOR_SPEED 400

FinController::FinController(const uint8_t(&&pins0)[4],
                             const uint8_t(&&pins1)[4],
                             const uint8_t(&&pins2)[4],
                             const uint8_t(&&pins3)[4]) :
  stepper0{ AccelStepper::FULL4WIRE, pins0[0], pins0[1], pins0[2], pins0[3] },
  stepper1{ AccelStepper::FULL4WIRE, pins1[0], pins1[1], pins1[2], pins1[3] },
  stepper2{ AccelStepper::FULL4WIRE, pins2[0], pins2[1], pins2[2], pins2[3] },
  stepper3{ AccelStepper::FULL4WIRE, pins3[0], pins3[1], pins3[2], pins3[3] }
{
}

void FinController::begin()
{
  stepper0.setSpeed(STEP_MOTOR_SPEED);
  stepper1.setSpeed(STEP_MOTOR_SPEED);
  stepper2.setSpeed(STEP_MOTOR_SPEED);
  stepper3.setSpeed(STEP_MOTOR_SPEED);
}

void FinController::makeFinCorrections(double ux, double uy, double uz)
{
  // stepper0 looks towards +X, stepper2 looks towards -X
  stepper0.moveTo(map_double<-180, 180, -1024, 1024>(-ux + uz));
  stepper2.moveTo(map_double<-180, 180, -1024, 1024>(ux + uz));
  // stepper1 looks towards -Y, stepper3 looks towards +Y
  stepper1.moveTo(map_double<-180, 180, -1024, 1024>(uy + uz));
  stepper3.moveTo(map_double<-180, 180, -1024, 1024>(-uy + uz));
}

void FinController::runMotors()
{
  stepper0.run();
  stepper1.run();
  stepper2.run();
  stepper3.run();
}

#elif defined(FIN_CONTROL_BY_SERVO)

FinController::FinController() :
  before{ 0 }
{
}

void FinController::begin(uint8_t pin0, uint8_t pin1,
                          uint8_t pin2, uint8_t pin3)
{
  servo0.attach(pin0);
  servo1.attach(pin1);
  servo2.attach(pin2);
  servo3.attach(pin3);
  servo0.write(SERVO_ZERO_ANGLE);
  servo1.write(SERVO_ZERO_ANGLE);
  servo2.write(SERVO_ZERO_ANGLE);
  servo3.write(SERVO_ZERO_ANGLE);
}

void FinController::makeFinCorrections(double ux, double uy, double uz)
{
  // servo0 looks towards +X, servo2 looks towards -X
  servo0_a = SERVO_ZERO_ANGLE - ux + uz;
  servo2_a = SERVO_ZERO_ANGLE + ux + uz;
  // servo1 looks towards -Y, servo3 looks towards +Y
  servo1_a = SERVO_ZERO_ANGLE + uy + uz;
  servo3_a = SERVO_ZERO_ANGLE - uy + uz;
}

void FinController::runMotors()
{
  if (millis() - before >= SERVO_PWM_PERIOD_MS) {
    servo0.write(servo0_a);
    servo1.write(servo1_a);
    servo2.write(servo2_a);
    servo3.write(servo3_a);
    before = millis();
  }
}

#endif
