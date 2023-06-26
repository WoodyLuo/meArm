/*!
 *  @file meArm.h
 *
 *  @mainpage This is a library for meArm of an open source robotic arm.
 *
 *  @section intro_sec Introduction
 *  You must calibrate your servo motors of meArm before use this library.
 *  x - Base servo motor: Turn a steering arm clockwise to the end, 
 *      and the steering arm need to place horizontal positioning to servo.
 *      Minimum angle is 0 degree, Maximum angle is 50 degree, Initial angle is 0 degree.
 *  x基座(pinBase)：順時鐘轉到底，初始90度，最大值140，最小值0，pin8，轉臂擺放平行舵機，可偏右。
 *  
 *  y - Shoulder servo motor: Turn a steering arm clockwise to the end, 
 *      and the steering arm need to place vertical positioning to servo.
 *      Minimum angle is 0 degree, Maximum angle is 100 degree, Initial angle is 0 degree.
 *  y右臂(pinShoulder)：順時鐘轉到底，初始0度，最大值100，pin10，轉臂擺放垂直舵機，可偏後。
 * 
 *  z - Elbow servo motor: Turn a steering arm clockwise to the end, 
 *      and the steering arm need to place vertical positioning to servo.
 *      Minimum angle is 0 degree, Maximum angle is 50 degree, Initial angle is 0 degree.
 *  z左肘(pinElbow)：順時鐘轉到底，初始0度，最大值50度，pin9，轉臂擺放垂直舵機，可偏後。
 * 
 *  g - Gripper servo motor: Turn a steering arm clockwise to the end, 
 *      and the steering arm need to place horizontal positioning to servo.
 *      Minimum angle is 0 degree, Maximum angle is 60 degree, Initial angle is 0 degree.
 *  g爪子pinGripper：順時鐘轉到底，初始0度，最大值60，pin4，轉臂擺放平行舵機，可偏左一格。 
 * 
 *  Usage:
 *    // if you want to set minimum and maximum angle for your servo motors.
 *    // meArm arm(0, 160, 0, 100, 0, 50, 0, 60);
 *    // Using default angle for your servo motors.
 *    meArm arm;
 *    arm.begin(7, 6, 5, 4);
 *    arm.goDirectlyTo(90, 45, 25, 30);
 * 
 *  @section author Author
 *  Written by Woody Lo, Wen-Cheng.
 *  Date: Jun, 26, 2023
 * 
 *  @section license License
 *  MIT license, all text above must be included in any redistribution
 */
#include <Arduino.h>
#include "meArm.h"
#include <Servo.h>
bool setup_servo (ServoInfo& svo, const int n_min, const int n_max)
{
    // Set limits
    svo.n_min = n_min;
    svo.n_max = n_max;

    return true;
}

//Full constructor with calibration data
meArm::meArm(int sweepMinBase, int sweepMaxBase, 
          int sweepMinShoulder, int sweepMaxShoulder, 
          int sweepMinElbow, int sweepMaxElbow, 
          int sweepMinGripper, int sweepMaxGripper) {
  //calroutine();
  setup_servo(_svoBase, sweepMinBase, sweepMaxBase);
  setup_servo(_svoShoulder, sweepMinShoulder, sweepMaxShoulder);
  setup_servo(_svoElbow, sweepMinElbow, sweepMaxElbow);
  setup_servo(_svoGripper, sweepMinGripper, sweepMaxGripper);
}

void meArm::begin(int pinBase, int pinShoulder, int pinElbow, int pinGripper) {
  _pinBase = pinBase;
  _pinShoulder = pinShoulder;
  _pinElbow = pinElbow;
  _pinGripper = pinGripper;
  _base.attach(_pinBase);
  _shoulder.attach(_pinShoulder);
  _elbow.attach(_pinElbow);
  _gripper.attach(_pinGripper);

  goDirectlyTo(90, 0, 0, 0);
}

void meArm::end() {
  _base.detach();
  _shoulder.detach();
  _elbow.detach();
  _gripper.detach();
}

//Set servos to reach a certain point directly without caring how we get there 
void meArm::goDirectlyTo(float x, float y, float z, float g) {
  setAngleX(x);
  _base.write(_x);
  setAngleY(y);
  _shoulder.write(_y);
  setAngleZ(z);
  _elbow.write(_z);
  setAngleG(g);
  _gripper.write(_g);
}

// Set servo's angle
void meArm::setAngleX(float x){
  int s = checkRangeX(x);
  if(s == 1){
    _x = x;
  }else if(s == -1){
    _x = _svoBase.n_min;
  }else if(s == 0){
    _x = _svoBase.n_max;
  }
}
void meArm::setAngleY(float y){
  int s = checkRangeY(y);
  if(s == 1){
    _y = y;
  }else if(s == -1){
    _y = _svoShoulder.n_min;
  }else if(s == 0){
    _y = _svoShoulder.n_max;
  }
}
void meArm::setAngleZ(float z){
  int s = checkRangeZ(z);
  if(s == 1){
    _z = z;
  }else if(s == -1){
    _z = _svoElbow.n_min;
  }else if(s == 0){
    _z = _svoElbow.n_max;
  }
}
void meArm::setAngleG(float g){
  int s = checkRangeG(g);
  if(s == 1){
    _g = g;
  }else if(s == -1){
    _g = _svoGripper.n_min;
  }else if(s == 0){
    _g = _svoGripper.n_max;
  }
}

// Check the range of servo's angle
int meArm::checkRangeX(float x){
  if(x <= _svoBase.n_min){
    return -1;
  }else if(x >= _svoBase.n_max) {
    return 0;
  }else{
    return 1;
  }
}
int meArm::checkRangeY(float y){
  if(y <= _svoShoulder.n_min){
    return -1;
  }else if(y >= _svoShoulder.n_max) {
    return 0;
  }else{
    return 1;
  }
}
int meArm::checkRangeZ(float z){
  if(z <= _svoElbow.n_min){
    return -1;
  }else if(z >= _svoElbow.n_max) {
    return 0;
  }else{
    return 1;
  }
}
int meArm::checkRangeG(float g){
  if(g <= _svoGripper.n_min){
    return -1;
  }else if(g >= _svoGripper.n_max) {
    return 0;
  }else{
    return 1;
  }
}

//Current x, y, z and g
float meArm::getX() {
  return _x;
}
float meArm::getY() {
  return _y;
}
float meArm::getZ() {
  return _z;
}
float meArm::getG() {
  return _g;
}

