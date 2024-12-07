#ifndef PA_IK_H
#define PA_IK_H

#include <ArduinoSTL.h>
#include <vector>

struct Limb {
  double length;
  /// In radians
  double minAngle;
  double maxAngle;
};

struct Vector2 {
  double x;
  double y;

  double distanceTo(Vector2 other);
  double length();
  Vector2 normalized();

  Vector2 operator+(Vector2 const& other);
  Vector2 operator-(Vector2 const& other);
  Vector2 operator*(double other);
  Vector2 operator*(float other);
  Vector2 operator*(int other);
  Vector2 operator/(double other);
  Vector2 operator/(float other);
  Vector2 operator/(int other);
};

Vector2 operator*(double left, Vector2 const& right) {
  return right * left;
}
Vector2 operator*(float left, Vector2 const& right) {
  return right * left;
}
Vector2 operator*(int left, Vector2 const& right) {
  return right * left;
}



class IK {
public:
  IK(std::vector<Limb> limbs, int bias = 3, int iterations = 32);
  /// returns angle positions in radians
  std::vector<double> solve(Vector2 endPos);
  static double rad_to_deg(double deg);
  static double deg_to_rad(double rad);

private:
  const int bias;
  const int iterations;
  const Vector2 basePoint{0, 0};
  std::vector<Limb> limbs;
  std::vector<Vector2> joints;
  double totalLimbLength;
};

#endif // PA_IK_H