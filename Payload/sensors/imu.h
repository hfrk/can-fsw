
// Select one
//#define NXP
#define BNO
#define GYRO_WEIGHT 0.96

#include <Wire.h>
#include <Adafruit_Sensor.h>

#ifdef BNO
  #include <Adafruit_BNO055.h>
  #include <utility/imumaths.h>
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
#endif

#ifdef NXP
  #include <Adafruit_FXAS21002C.h>
  #include <Adafruit_FXOS8700.h>
  Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
  Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
#endif

class Sensor_IMU {
private:
    sensors_event_t gevent;
    sensors_event_t aevent;
    sensors_event_t mevent;

#ifdef BNO
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
#endif

#ifdef NXP
    Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
    Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
#endif

public:
    Sensor_IMU() {
#ifdef NXP
        if (!gyro.begin())
           Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
        if (!accelmag.begin())
            Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
#endif

#ifdef BNO
        if(!bno.begin())
          Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        bno.setExtCrystalUse(true);
#endif
    }

  void update() {
  #ifdef NXP
    gyro.getEvent(&gevent);
    accelmag.getEvent(&aevent, &mevent);
    gevent.gyro.x -= 0.0103;
    gevent.gyro.y += 0.0115;
    gevent.gyro.z -= 0.0037;
    aevent.acceleration.x += 0.094747;
    aevent.acceleration.y -= 0.003134;
    aevent.acceleration.z += 0.024780;
    mevent.magnetic.x += 32.65;
    mevent.magnetic.y += 78.70;
#endif

#ifdef BNO
    bno.getEvent(&gevent, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&aevent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&mevent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
#endif

    //Serial.print(String(" X: ") + String(mevent.magnetic.x));
    //Serial.print(String(" Y: ") + String(mevent.magnetic.y));
    //float accelRoll = atan2(aevent.acceleration.y,
    //  sqrt((aevent.acceleration.z * aevent.acceleration.z) + (aevent.acceleration.x * aevent.acceleration.x)))
    //  * RAD_TO_DEG;
    //float accelPitch = atan2(-aevent.acceleration.x,
    //  sqrt((aevent.acceleration.z * aevent.acceleration.z) + (aevent.acceleration.y * aevent.acceleration.y)))
    //  * RAD_TO_DEG;

    // gyro measurement in degrees
    gyroR = gevent.gyro.y * RAD_TO_DEG;
    gyroP = -gevent.gyro.x * RAD_TO_DEG;
    gyroY = gevent.gyro.z * RAD_TO_DEG;
    // accelerometer measurement is already in m/s2
    accelR = aevent.acceleration.y;
    accelP = -aevent.acceleration.x;
    accelY = aevent.acceleration.z;
    // magnetometer measurement in tesla
    magR = mevent.magnetic.y;
    magP = -mevent.magnetic.x;
    magY = mevent.magnetic.z;

    //roll  = GYRO_WEIGHT * (roll + gyroR * delta) + (1-GYRO_WEIGHT) * accelRoll;
    //pitch = GYRO_WEIGHT * (pitch + gyroP * delta) + (1-GYRO_WEIGHT) * accelPitch;
    yaw   = atan2(mevent.magnetic.y, mevent.magnetic.x) * RAD_TO_DEG;
    if (yaw < 0)
        yaw += 360;
  }
    float gyroR, gyroP, gyroY;
    float accelR, accelP, accelY;
    float magR, magP, magY;
    float roll;
    float pitch;
    float yaw;
};
