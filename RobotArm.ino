#include <Servo.h>
#include <ArduinoSTL.h>
#include <vector>
#include "Base.h"
#include "IK.h"

Servo s1, s2, s3;
Base m(2048, 8, 10, 9, 11);
int joyX = A1;
int joyY = A0;
int joyP = 2;
int openButton = 4;
int closeButton = 7;

double goalX;
double goalY;
double goalTheta;

// set up Inverse Kinematics with arm lengths and 
IK ik({
//  limb len (in)   min angle constraint    max angle constraint
  { 7,              IK::deg_to_rad(-90),    IK::deg_to_rad(90)},
  { 7,              IK::deg_to_rad(-90),    IK::deg_to_rad(90)},
  { 7,              IK::deg_to_rad(-90),    IK::deg_to_rad(90)}
});

void setup() {
  // setup servos and buttons
  s1.attach(3);
  s2.attach(5);
  s3.attach(6);
  pinMode(joyP, INPUT);
  pinMode(openButton, INPUT);
  pinMode(closeButton, INPUT);

  // setup initial goal position
  Vector2 pos = ik.getEndEffectorPos();
  goalX = pos.x;
  goalY = pos.y;

  // setup serial for debugging
  Serial.begin(9600);
}

void loop() {
  // get joystick and button values
  double x = (2.0 * analogRead(joyX) / 1023.0 - 1) * -5;
  double y = (2.0 * analogRead(joyY) / 1023.0 - 1) * -5;
  if ((x < 0.2 && x > 0) || (x > -0.2 && x < 0)) x = 0;
  if ((y < 0.2 && y > 0) || (y > -0.2 && y < 0)) y = 0;
  Serial.println(x);
  Serial.println(y);
  Serial.println("------------");

  // rotate base using buttons
  if (digitalRead(openButton) == HIGH) {
    m.rotateBy(5);
  }
  if (digitalRead(closeButton) == HIGH) {
    m.rotateBy(-5);
  }

  goalX += x;
  goalY += y;

  // Use inverse kinematics to get angle position 
  std::vector<double> angles = ik.solve({goalX, goalY});
  s1.write(IK::rad_to_deg(angles[0]) - 90);
  s2.write(IK::rad_to_deg(angles[1]) - 90);
  s3.write(IK::rad_to_deg(angles[2]) - 90);
}
