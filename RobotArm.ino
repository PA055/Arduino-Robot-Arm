#include <Servo.h>
#include "Base.h"
#include "IK.h"

Servo s1, s2, s3;
Base m(2048, 8, 10, 9, 11);
auto joyX = A1;
auto joyY = A0;
auto joyP = 2;

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
}
