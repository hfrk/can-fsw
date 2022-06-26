/*
 * BMP388 Pressure sensor encapsulation
 */

#ifndef SENSOR_BMP_388_H
#define SENSOR_BMP_388_H

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

class Sensor_BMP388: private Adafruit_BMP3XX {
private:
  bool _fail = false;
  uint8_t _i2c_addr = 0x76;
  float _altitudeRef = 101325.0;
  int fail_counter = 0;

public:
  Sensor_BMP388(uint8_t i2c_addr) {
    _i2c_addr = i2c_addr;
    _fail = !begin_I2C(i2c_addr);
    if (_fail)
      return;
    // Set up oversampling and filter initialization
    setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    setPressureOversampling(BMP3_OVERSAMPLING_4X);
    setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    setOutputDataRate(BMP3_ODR_50_HZ);
  }

  bool getStatus() { return _fail; }

  void setReferenceAltitude(float ref) {
    _altitudeRef = ref;
  }
  void read() {
    fail_counter = performReading() ? 0 : ++fail_counter;
    if (fail_counter > 10) {
      fail_counter = begin_I2C(_i2c_addr) ? 0 : ++fail_counter;
    }
  }

  float getAltitude() {
    return 44330.0 * (1.0 - pow(pressure/_altitudeRef, 0.1903));
  }
  float getTemperature() {
    return temperature;
  }
  float getPressure() {
    read();
    return pressure;
  }
};

#endif /* SENSOR_BMP_388_H */
