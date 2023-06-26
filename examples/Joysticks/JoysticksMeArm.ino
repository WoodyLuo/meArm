/* meArm joysticks - Woody Lo, Wen-Cheng (Jun, 2023)
 * Using joysticks to control meArm
 * Uses two analogue joystcks (two pots each)
 * First stick moves gripper left, right, forwards, and backwards.
 * Second stick moves gripper up, down, and opens and closes.
 * 
 * I used Sparkfun thumbstick breakout boards, oriented 'upside down'.
 * 
 * Pins:
 * Arduino    Stick1    Stick2    Base   Shoulder  Elbow    Gripper
 *    GND       GND       GND    Brown     Brown   Brown     Brown
 *     5V       VCC       VCC      Red       Red     Red       Red
 *     A0       HOR
 *     A1       VER
 *     A2                 HOR
 *     A3                 VER
 *      7                       Yellow
 *      6                                 Yellow
 *      5                                         Yellow
 *      4                                                   Yellow
 */
#include "meArm.h"
#include <Servo.h>

// Servo motor's pin
int basePin = 7;
int shoulderPin = 6;
int elbowPin = 5;
int gripperPin = 4;

// Joystick's pin
int xdirPin = 0;  // Base
int ydirPin = 1;  // Shoulder
int zdirPin = 3;  // Elbow
int gdirPin = 2;  // Gripper

// Movement
float dx = 0;
float dy = 0;
float dz = 0;
float dg = 0;

// If you want to set minimum and maximum angle for your servo motors.
//meArm arm(0, 160, 0, 100, 0, 50, 0, 60);
// Using default angle for your servo motors.
meArm arm;

void setup()
{
  Serial.begin(9600);
  arm.begin(basePin, shoulderPin, elbowPin, gripperPin);
}

void loop()
{
  dx = map(analogRead(xdirPin),0,1023,-5.0,5.0);
  dy = map(analogRead(ydirPin),0,1023,5.0,-5.0);
  dz = map(analogRead(zdirPin),0,1023,-5.0,5.0);
  dg = map(analogRead(gdirPin),0,1023,5.0,-5.0);
  
  if (abs(dx) < 1.5) {
    dx = 0;
  }
  if (abs(dy) < 1.5) {
    dy = 0;
  }
  if (abs(dz) < 1.5) {
    dz = 0;
  }
  if (abs(dg) < 1.5) {
    dg = 0;
  }
  
  if (!(dx == 0 && dy == 0 && dz == 0 && dg == 0)){
    arm.goDirectlyTo(arm.getX() + dx, arm.getY() + dy, arm.getZ() + dz, arm.getG() + dg);
  }

  Serial.print("X(Base):");
  Serial.print(arm.getX());
  Serial.print(";\tY(Shoulder):");
  Serial.print(arm.getY());
  Serial.print(";\tZ(Elbow):");
  Serial.print(arm.getZ());
  Serial.print(";\tZ(Gripper):");
  Serial.println(arm.getG());

  delay(50);

}