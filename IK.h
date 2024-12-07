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
  double distanceSquaredTo(Vector2 other);
  double angle();
  double angleToPoint(Vector2 point);
  double length();
  Vector2 normalized();
  Vector2 rotated(double by);

  Vector2 operator+(Vector2 const& other);
  Vector2 operator-(Vector2 const& other);
  Vector2 operator*(double other);
  Vector2 operator/(double other);

  Vector2(double x, double y) : x(x), y(y) {}

  static Vector2 fromPolar(double angle, double radius = 1);
};

class IK {
public:
  IK(std::vector<Limb> limbs, int bias = 3, int iterations = 32);
  /// returns angle positions in radians
  std::vector<double> solve(Vector2 target);
  void resetPositions();
  void resetPositions(std::vector<Vector2> joints);
  Vector2 getEndEffectorPos();
  static double rad_to_deg(double rad);
  static double deg_to_rad(double deg);

private:
  const int bias;
  const int iterations;
  const Vector2 basePoint{0, 0};
  std::vector<Limb> limbs;
  std::vector<Vector2> joints;
  double totalLimbLength;

  double stepIK(Vector2 target);
  void forwardsPass();
  void backwardsPass(Vector2 target);
};

#endif // PA_IK_H