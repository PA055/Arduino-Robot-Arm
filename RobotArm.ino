#include <Servo.h>
#include <Stepper.h>

Servo s1, s2, s3;
const int STEPS_PER_ROTATION = 2048;
Stepper m(STEPS_PER_ROTATION, 8, 10, 9, 11);
auto joyX = A1;
auto joyY = A0;
auto joyP = 2;

int angle = 90;
int motorAngle = 0;

void setup() {
  // put your setup code here, to run once:
  s1.attach(3);
  s2.attach(5);
  s3.attach(6);
  m.setSpeed(10);
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
  Serial.println(angle);
  Serial.println(motorAngle);
  Serial.println("------------");
  

  angle += y;
  if (angle > 180) angle = 180;
  if (angle < 0) angle = 0;
  s1.write(angle);
  s2.write(angle);
  s3.write(angle);

  int prevMotorAngle = motorAngle;
  motorAngle += x;
  m.step(STEPS_PER_ROTATION * (motorAngle - prevMotorAngle) / 360.0);
}
