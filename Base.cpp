#include "Base.h"
#include "IK.h"

#define PI 3.14159265358979323846264338327950

Base::Base(int stepsPerRotation, int portA, int portB, int portC, int portD)
            : stepsPerRotation(stepsPerRotation), 
              currentAngle(PI), 
              motor(stepsPerRotation, portA, portB, portC, portD) {
  motor.setSpeed(10);
}

double Base::getAngle(bool radians) {
  if (radians)
    return IK::deg_to_rad(currentAngle);
  return currentAngle;
}

void Base::resetAngle(double angle, bool radians) {
  if (radians)
    currentAngle = IK::rad_to_deg(angle)
  else
    currentAngle = angle;
}

void Base::rotateBy(double angle, bool radians) {
  if (radians)
    angle = IK::rad_to_deg(radians);
  
  if (angle + currentAngle > 360) {
    rotateByRaw(angle - 360);
    return;
  }

  if (angle + currentAngle < 0) {
    rotateByRaw(angle + 360);
    return;
  }

  rotateByRaw(angle);
}

void Base::rotateTo(double angle, bool radians) {
  if (radians)
    angle = IK::rad_to_deg(radians);
  
  rotateBy(angle - currentAngle, false);
}

void Base::rotateByRaw(double angle) {
  currentAngle += angle;
  motor.step(stepsPerRotation * angle / 360);
}