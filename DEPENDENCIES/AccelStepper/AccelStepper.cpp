// AccelStepper.cpp
//
// Copyright (C) 2009-2013 Mike McCauley
// $Id: AccelStepper.cpp,v 1.23 2016/08/09 00:39:10 mikem Exp $

#include "AccelStepper.h"

double map_double(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#if 0
// Some debugging assistance
void dump(uint8_t* p, int l)
{
  int i;

  for (i = 0; i < l; i++)
  {
    Serial.print(p[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}
#endif

AccelStepper::AccelStepper(uint8_t interface, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, bool enable)
{
  _interface = interface;
  _currentPos = 0;
  _targetPos = 0;
  _speed = 0.0;
  _stepInterval = 0;
  _minPulseWidth = 1;
  _lastStepTime = 0;
  _pin[0] = pin1;
  _pin[1] = pin2;
  _pin[2] = pin3;
  _pin[3] = pin4;
  output_disabled = true;

  // NEW
  _direction = DIRECTION_CCW;

  if (enable)
    enableOutputs();
}

void AccelStepper::moveTo(long absolute)
{
  _targetPos = absolute;
}

void AccelStepper::move(long relative)
{
  moveTo(_currentPos + relative);
}

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
boolean AccelStepper::runSpeed()
{
  // Dont do anything unless we actually have a step interval
  if (!_stepInterval)
    return false;

  unsigned long time = micros();
  if (time - _lastStepTime >= _stepInterval)
  {
    if (output_disabled)
    {
      output_disabled = false;
    }
    else
    {
      if (_direction == DIRECTION_CW)
      {
        // Clockwise
        _currentPos += 1;
      }
      else
      {
        // Anticlockwise
        _currentPos -= 1;
      }
    }

    step(_currentPos);

    _lastStepTime = time; // Caution: does not account for costs in step()

    return true;
  }
  else
  {
    return false;
  }
}

long AccelStepper::distanceToGo()
{
  return _targetPos - _currentPos;
}

long AccelStepper::targetPosition()
{
  return _targetPos;
}

long AccelStepper::currentPosition()
{
  return _currentPos;
}

unsigned long AccelStepper::lastStepTime()
{
  return _lastStepTime;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void AccelStepper::setCurrentPosition(long position)
{
  _targetPos = _currentPos = position;
  _stepInterval = 0;
  _speed = 0.0;
}

// Exists for compatibility
boolean AccelStepper::run()
{
  return runSpeedToPosition();
}

void AccelStepper::setSpeed(double speed)
{
  if (speed == _speed)
    return;

  if (speed == 0.0)
    _stepInterval = 0;
  else
  {
    _stepInterval = fabs(1000000.0 / speed);
    _direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
  }
  _speed = speed;
}

double AccelStepper::speed()
{
  return _speed;
}

// Subclasses can override
void AccelStepper::step(long step)
{
  switch (_interface)
  {
    case DRIVER:
      step1(step);
      break;

    case FULL2WIRE:
      step2(step);
      break;

    case FULL3WIRE:
      step3(step);
      break;

    case FULL4WIRE:
      step4(step);
      break;

    case HALF3WIRE:
      step6(step);
      break;

    case HALF4WIRE:
      step8(step);
      break;
  }
}

// You might want to override this to implement eg serial output
// bit 0 of the mask corresponds to _pin[0]
// bit 1 of the mask corresponds to _pin[1]
// ....
void AccelStepper::setOutputPins(uint8_t mask)
{
  uint8_t numpins = 2;
  if (_interface == FULL4WIRE || _interface == HALF4WIRE)
    numpins = 4;
  else if (_interface == FULL3WIRE || _interface == HALF3WIRE)
    numpins = 3;
  uint8_t i;
  for (i = 0; i < numpins; i++)
    digitalWrite(_pin[i], (mask & (1 << i)) ? HIGH : LOW);
}

// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step1(long step)
{
  (void)(step); // Unused

  // _pin[0] is step, _pin[1] is direction
  setOutputPins(_direction ? 0b10 : 0b00); // Set direction first else get rogue pulses
  setOutputPins(_direction ? 0b11 : 0b01); // step HIGH
  // Caution 200ns setup time
  // Delay the minimum allowed pulse width
  delayMicroseconds(_minPulseWidth);
  setOutputPins(_direction ? 0b10 : 0b00); // step LOW
}


// 2 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step2(long step)
{
  switch (step & 0x3)
  {
    case 0: /* 01 */
      setOutputPins(0b10);
      break;

    case 1: /* 11 */
      setOutputPins(0b11);
      break;

    case 2: /* 10 */
      setOutputPins(0b01);
      break;

    case 3: /* 00 */
      setOutputPins(0b00);
      break;
  }
}
// 3 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step3(long step)
{
  switch (step % 3)
  {
    case 0:    // 100
      setOutputPins(0b100);
      break;

    case 1:    // 001
      setOutputPins(0b001);
      break;

    case 2:    //010
      setOutputPins(0b010);
      break;

  }
}

// 4 pin step function for half stepper
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step4(long step)
{
  switch (step & 0x3)
  {
    case 0:    // 1010
      setOutputPins(0b0101);
      break;

    case 1:    // 0110
      setOutputPins(0b0110);
      break;

    case 2:    //0101
      setOutputPins(0b1010);
      break;

    case 3:    //1001
      setOutputPins(0b1001);
      break;
  }
}

// 3 pin half step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step6(long step)
{
  switch (step % 6)
  {
    case 0:    // 100
      setOutputPins(0b100);
      break;

    case 1:    // 101
      setOutputPins(0b101);
      break;

    case 2:    // 001
      setOutputPins(0b001);
      break;

    case 3:    // 011
      setOutputPins(0b011);
      break;

    case 4:    // 010
      setOutputPins(0b010);
      break;

    case 5:    // 011
      setOutputPins(0b110);
      break;

  }
}

// 4 pin half step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step8(long step)
{
  switch (step & 0x7)
  {
    case 0:    // 1000
      setOutputPins(0b0001);
      break;

    case 1:    // 1010
      setOutputPins(0b0101);
      break;

    case 2:    // 0010
      setOutputPins(0b0100);
      break;

    case 3:    // 0110
      setOutputPins(0b0110);
      break;

    case 4:    // 0100
      setOutputPins(0b0010);
      break;

    case 5:    //0101
      setOutputPins(0b1010);
      break;

    case 6:    // 0001
      setOutputPins(0b1000);
      break;

    case 7:    //1001
      setOutputPins(0b1001);
      break;
  }
}

// Prevents power consumption on the outputs
void    AccelStepper::disableOutputs()
{
  setOutputPins(0); // Handles inversion automatically
  output_disabled = true;
}

void    AccelStepper::enableOutputs()
{
  pinMode(_pin[0], OUTPUT);
  pinMode(_pin[1], OUTPUT);
  digitalWrite(_pin[0], LOW);
  digitalWrite(_pin[1], LOW);
  if (_interface == FULL4WIRE || _interface == HALF4WIRE)
  {
    pinMode(_pin[2], OUTPUT);
    pinMode(_pin[3], OUTPUT);
    digitalWrite(_pin[2], LOW);
    digitalWrite(_pin[3], LOW);
  }
  else if (_interface == FULL3WIRE || _interface == HALF3WIRE)
  {
    pinMode(_pin[2], OUTPUT);
    digitalWrite(_pin[2], LOW);
  }
}

void AccelStepper::setMinPulseWidth(unsigned int minWidth)
{
  _minPulseWidth = minWidth;
}

boolean AccelStepper::runSpeedToPosition()
{
  if (_targetPos == _currentPos)
    return false;
  if (_targetPos > _currentPos)
    _direction = DIRECTION_CW;
  else
    _direction = DIRECTION_CCW;
  return runSpeed();
}
