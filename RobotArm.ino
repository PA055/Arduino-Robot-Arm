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

IK ik({
//  limb len (in)   min angle constraint    max angle constraint
  { 7,              IK::deg_to_rad(-90),    IK::deg_to_rad(90)},
  { 7,              IK::deg_to_rad(-90),    IK::deg_to_rad(90)},
  { 7,              IK::deg_to_rad(-90),    IK::deg_to_rad(90)}
});

void setup() {
  // put your setup code here, to run once:
  s1.attach(3);
  s2.attach(5);
  s3.attach(6);
  pinMode(joyP, INPUT);
  pinMode(openButton, INPUT);
  pinMode(closeButton, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  double x = (2.0 * analogRead(joyX) / 1023.0 - 1) * -5;
  double y = (2.0 * analogRead(joyY) / 1023.0 - 1) * -5;
  if ((x < 0.2 && x > 0) || (x > -0.2 && x < 0)) x = 0;
  if ((y < 0.2 && y > 0) || (y > -0.2 && y < 0)) y = 0;
  Serial.println(x);
  Serial.println(y);
  Serial.println("------------");

  if (digitalRead(openButton) == HIGH) {
    m.rotateBy(5);
  }
  if (digitalRead(closeButton) == HIGH) {
    m.rotateBy(-5);
  }

  goalX += x;
  goalY += y;
  std::vector<double> angles = ik.solve({goalX, goalY});
  s1.write(IK::rad_to_deg(angles[0]));
  s2.write(IK::rad_to_deg(angles[1]));
  s3.write(IK::rad_to_deg(angles[2]));
}
