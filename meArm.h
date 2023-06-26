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
 *  y - Shoulder servo motor (right servo): Turn a steering arm clockwise to the end, 
 *      and the steering arm need to place vertical positioning to servo.
 *      Minimum angle is 0 degree, Maximum angle is 100 degree, Initial angle is 0 degree.
 *  y右臂(pinShoulder)：順時鐘轉到底，初始0度，最大值100，pin10，轉臂擺放垂直舵機，可偏後。
 * 
 *  z - Elbow servo motor (left servo): Turn a steering arm clockwise to the end, 
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
#ifndef MEARM_H
#define MEARM_H

#include <Arduino.h>
#include <Servo.h>

const float pi=3.14159265359;

struct ServoInfo {
    int n_min, n_max;   // PWM 'soft' limits - should be just within range
};

class meArm {
  public:
    //Full constructor uses calibration data, or can just give pins
    meArm(int sweepMinBase=0, int sweepMaxBase=160, 
      int sweepMinShoulder=0, int sweepMaxShoulder=100, 
      int sweepMinElbow=0, int sweepMaxElbow=50, 
      int sweepMinGripper=0, int sweepMaxGripper=60);
    //required before running
    void begin(int pinBase, int pinShoulder, int pinElbow, int pinGripper);
    void end();
    //Set servos to reach a certain point directly without caring how we get there 
    void goDirectlyTo(float x, float y, float z, float g);

    // Set servo motor's angle
    void setAngleX(float x);  // Base
    void setAngleY(float y);  // Shoulder (right servo)
    void setAngleZ(float z);  // Elbow (left servo)
    //Grab something: _g++; Let go of something: _g--;
    void setAngleG(float g);  // Gripper

    //Check to see if possible to move
    int checkRangeX(float x);
    int checkRangeY(float y);
    int checkRangeZ(float z);
    int checkRangeG(float g);

    //Current angle of x, y, z and g
    float getX();
    float getY();
    float getZ();
    float getG();

  private:
    //void polarToCartesian(float theta, float r, float& x, float& y);
    float _x, _y, _z, _g;
    Servo _base, _elbow, _shoulder, _gripper;
    ServoInfo _svoBase, _svoElbow, _svoShoulder, _svoGripper;
    int _pinBase, _pinElbow, _pinShoulder, _pinGripper;
};

#endif
