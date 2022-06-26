/*
 * Library to provide PID control on 360 degree continuous rotation servo
 * Use in combination with any rotational/positional sensor of the servo
 */

#include "Servo_PID.h"

Servo_PID::Servo_PID() {
  this->P = 0;
  this->I = 0;
  this->D = 0;
  this->target = 0;
  this->PV = NULL;
  this->prev_PV = 0;
}
Servo_PID::Servo_PID(float P, float I, float D, float* PV, float target, float tolerance) {
  this->P = P;
  this->I = I;
  this->D = D;
  this->target = target;
  this->PV = PV;
  this->prev_PV = 0;
  this->tolerance = tolerance;
}

void Servo_PID::setPID(float P, float I, float D) {
  this->P = P;
  this->I = I;
  this->D = D;
}

void Servo_PID::setPI(float P, float I) {
  setPID(P, I, 0);
}

void Servo_PID::setPD(float P, float D) {
  setPID(P, 0, D);
}

void Servo_PID::setTarget(float target) {
  this->target = target;
}

void Servo_PID::setTolerance(float tolerance) {
  this->tolerance = tolerance;  
}
void Servo_PID::setProcessVariable(float* PV) {
  this->PV = PV;
}

void Servo_PID::setDirection(int dir) {
  this->rotate_dir = dir;
}

void Servo_PID::start() {
  this->last = millis();
}

void Servo_PID::update() {
  float error = target - (*PV);
  
  if(error < -180) error += 360;
  else if(error > 180) error -= 360;
  else if (abs(error) < tolerance) {
    error = 0;
  }
  float delta_time = float(millis() - last)/1000;
  last = millis();

  error_sum += error * delta_time;
  float error_diff = (error - last_error) / delta_time;

  float angle = P * error + I * error_sum + D * error_diff;
  if (angle < -90) {
    angle = -90;
  }
  else if (angle > 90) {
    angle = 90;
  }

  write(90 + angle * rotate_dir);

  last_error = error;
}
