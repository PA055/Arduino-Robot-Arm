#ifndef PA_BASE_H
#define PA_BASE_H

#include <Stepper.h>

class Base {
public:
  Base(int stepsPerRotation, int portA, int portB, int portC, int portD);
  void rotateTo(double angle, bool radians = false);
  void rotateBy(double angle, bool radians = false);
  double getAngle(bool radians = false);
  void resetAngle(double angle, bool radians = false);
private:
  const int stepsPerRotation;
  double currentAngle;
  Stepper motor;
  void rotateByRaw(double angle);
};

#endif // PA_BASE_H