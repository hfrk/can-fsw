/*
 * Library to provide PID control on 360 degree continuous rotation servo
 * Use in combination with any rotational/positional sensor of the servo
 */

#ifndef SERVO_PID_H
#define SERVO_PID_H

#include "Arduino.h"
#include "Servo.h"

#define SERVO_CCW -1
#define SERVO_CW   1

class Servo_PID: public Servo {
public:
  Servo_PID();
  Servo_PID(float P, float I, float D, float* PV, float target, float tolerance);
  
  void setPID(float P, float I, float D);
  void setPI(float P, float I);
  void setPD(float P, float D);
  void setProcessVariable(float* PV);
  void setTarget(float target);
  void setTolerance(float tolerance);
  void setDirection(int dir);
  void start();
  void update();

private:
  int rotate_dir;
  float P, I, D;
  float target;
  float* PV;
  float prev_PV;
  float last;
  float tolerance = 0;
  float error_sum = 0;
  float last_error = 0;
};

#endif /* SERVO_PID_H */
