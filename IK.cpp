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

Vector2 Vector2::operator*(float other) {
  return {this->x * other, this->y * other};
}

Vector2 Vector2::operator*(int other) {
  return {this->x * other, this->y * other};
}

Vector2 Vector2::operator/(double other) {
  return {this->x / other, this->y / other};
}

Vector2 Vector2::operator/(float other) {
  return {this->x / other, this->y / other};
}

Vector2 Vector2::operator/(int other) {
  return {this->x / other, this->y / other};
}

double Vector2::distanceTo(Vector2 other) {
  return (*this - other).length();
}

double Vector2::length() {
  return std::sqrt(this->x * this->x + this->y * this->y);
}

Vector2 Vector2::normalized() {
  return *this / this->length();
}


IK::IK(std::vector<Limb> limbs, int bias, int iterations) : limbs(limbs), bias(bias), iterations(iterations) {
  totalLimbLength = 0;
  joints.push_back(basePoint);
  for (int i = 0; i < limbs.size(); i++) {
    totalLimbLength += limbs[i].length;
    Vector2 offset{limbs[i].length, 0};
    joints[i + 1] = joints[i] + offset;
  }
}

double IK::deg_to_rad(double rad) {
  return rad * 180 / PI;
}

double IK::rad_to_deg(double deg) {
  return deg * PI / 180;
}


std::vector<double> IK::solve(Vector2 endPos) {
  std::vector<Vector2> prev_joints(joints);
}