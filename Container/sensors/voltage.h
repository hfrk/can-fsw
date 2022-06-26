/*
 * Library to encapsulate ADC-based voltage sensor
 */

#ifndef SENSOR_VOLTAGE_H
#define SENSOR_VOLTAGE_H

#include "Arduino.h"

class Sensor_Voltage {
private:
  int voltage_ratio = 11;
  int _pin;

public:
  Sensor_Voltage(int pin = A7) {
    _pin = pin;
    pinMode(_pin, INPUT_PULLDOWN);
    analogReadResolution(12);
  }

  float readVoltage() {
    return analogRead(_pin) * (3.3 / 4095.0) * voltage_ratio;
  }
};

#endif /* SENSOR_VOLTAGE_H */
