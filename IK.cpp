#include "IK.h"
#include <ArduinoSTL.h>
#include <vector>
#include <cmath>

#define PI 3.14159265358979323846264338327950

Vector2 Vector2::operator+(const Vector2 &other) {
  return {this->x + other.x, this->y + other.y};
}

Vector2 Vector2::operator-(const Vector2 &other) {
  return {this->x - other.x, this->y - other.y};
}

Vector2 Vector2::operator*(double other) {
  return {this->x * other, this->y * other};
}

Vector2 Vector2::operator/(double other) {
  return {this->x / other, this->y / other};
}

double Vector2::distanceTo(Vector2 other) {
  return (*this - other).length();
}

double Vector2::distanceSquaredTo(Vector2 other) {
  Vector2 offset = *this - other;
  return offset.x * offset.x + offset.y + offset.y;
}

double Vector2::length() {
  return std::sqrt(x*x + y*y);
}

double Vector2::angle() {
  return std::atan2(y, x);
}

double Vector2::angleToPoint(Vector2 point) {
  return (point - *this).angle();
}

Vector2 Vector2::normalized() {
  return *this / this->length();
}

Vector2 Vector2::rotated(double by) {
  double sin = std::sin(by);
  double cos = std::cos(by);
  return {x * cos - y * sin, x * sin + y * cos};
}

Vector2 Vector2::fromPolar(double angle, double radius) {
  return {radius * std::cos(angle), radius * std::sin(angle)};
}


IK::IK(std::vector<Limb> limbs, int bias, int iterations) : limbs(limbs), bias(bias), iterations(iterations) {
  totalLimbLength = 0;
  joints.push_back(basePoint);
  for (int i = 0; i < limbs.size(); i++) {
    totalLimbLength += limbs[i].length;
    joints[i + 1] = joints[i] + Vector2(limbs[i].length, 0);
  }
}

double IK::deg_to_rad(double rad) {
  return rad * 180 / PI;
}

double IK::rad_to_deg(double deg) {
  return deg * PI / 180;
}


std::vector<double> IK::solve(Vector2 target) {
  std::vector<Vector2> prev_joints(joints);
  double sqrDist = stepIK(target);
  if (sqrDist > bias * bias) {
    joints = prev_joints;
    stepIK(
      basePoint + (target - basePoint).normalized() *
      (basePoint.distanceTo(joints[joints.size() - 1]))
    );
    return;
  }

  std::vector<double> angles;
  for (int i = 0; i < joints.size() - 1; i++)
    angles.push_back(joints[i].angleToPoint(joints[i + 1]));
  return angles;
}

double IK::stepIK(Vector2 target) {
  double sqrDist = basePoint.distanceSquaredTo(target);
  int iters = 0;
  if (sqrDist > totalLimbLength * totalLimbLength) {
    backwardsPass(target);
    forwardsPass();
    return 0;
  }

  double minDist = 999999999;
  std::vector<Vector2> minJoints;
  while (iters < iterations) {
    backwardsPass(target);
    forwardsPass();
    iters++;

    sqrDist = joints[joints.size() - 1].distanceSquaredTo(target);
    if (minDist > sqrDist) {
      minDist = sqrDist;
      minJoints = joints;
    } else break;
  }

    if (sqrDist > minDist)
      joints = minJoints;

    return joints[joints.size() - 1].distanceSquaredTo(target);
}

double clamp(double value, double min, double max) {
  if (value > max) return max;
  if (value < min) return min;
  return value;
}

double wrap(double value, double min, double max) {
  double range = max - min;
  double result = range == 0 ? min : value - (range * std::floor((value - min) / range));
  if (result == max) return min;
  return result;
}

void IK::forwardsPass() {
  double rootAngle = 0.0;
  joints[0] = basePoint;

  for (int i = 0; i < joints.size() - 1; i++) {
    Limb limb = limbs[i];
    Vector2 a = joints[i];
    Vector2 b = joints[i + 1];

    double diffAngleRaw = wrap(a.angleToPoint(b) - rootAngle, -PI, PI);
    double diffAngle = clamp(diffAngleRaw, limb.minAngle, limb.maxAngle);
    double angle = rootAngle + diffAngle;

    joints[i + 1] = a + Vector2(limb.length, 0).rotated(angle);
    rootAngle = angle;
  }
}

void IK::backwardsPass(Vector2 target) {
  joints[joints.size() - 1] = target;
  for (int i = joints.size() - 1; i > 0; i--) {
    Limb limb = limbs[i - 1];
    Vector2 a = joints[i];
    Vector2 b = joints[i - 1];
    Vector2 c = i > 1 ? joints[i - 2] : basePoint;

    double rootAngle = c.angleToPoint(b);
    double diffAngleRaw = b.angleToPoint(a) - rootAngle;
    double diffAngle = clamp(diffAngleRaw, limb.minAngle, limb.maxAngle);
    double angle = rootAngle + diffAngle;

    joints[i + 1] = a + Vector2(limb.length, 0).rotated(angle + PI);
  }
}