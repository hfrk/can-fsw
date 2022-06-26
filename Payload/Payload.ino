/*
 * Narantaka @ 2022 CanSat Competition
 * Firmware for cansat payload
 * Using Teensy 4.0 microcontroller
 *
 * Communication:
 * * xbee (to container):
 *   2.4 GHz XBEE-PRO S2C using 115200 8-N-1
 *   Use Serial4 ports (16 and 17)
 *
 * - Ananda Hafidh R K (github.com/hfrk)
 */

// Serial communication
#define xbee Serial4
//#include <SoftwareSerial.h>
//SoftwareSerial xbee(22,23);

// Interval for nonblocking loop
#define TRANSMIT_INTERVAL 250
#define SENSOR_UPDATE_INTERVAL 50

#include "telemetry.h"              // telemetry structure
#include "sensors/bmp388.h"         // pressure sensor
#include "sensors/voltage.h"        // ADC voltage reader
#include "sensors/imu.h"            // camera position sensor
//#include "sensors/stabilizer.h"
#include "Servo_PID.h"              // stabilizer
#include <EEPROM.h>

PayloadTelemetry payload = PayloadTelemetry();

Sensor_BMP388 bmp = Sensor_BMP388(0x76);
Sensor_Voltage vsense = Sensor_Voltage(A6);
Sensor_IMU cam_imu = Sensor_IMU();

// 3-axis servo:
// Servo yaw is 360 deg, PWM control velocity instead of position
// Servo pitch and roll is 180 deg, just plug the degs
float yaw;
float pit;
float rol;
Servo_PID servo_yaw = Servo_PID(0.8, 0.0, 0.0, &yaw, 180.0, 1.0);
//Servo_PID servo_pit = Servo_PID(0.4, 0.0, 0.0, &pit, 0.0, 20.0);
//Servo_PID servo_rol = Servo_PID(0.8, 0.0, 0.0, &rol, 0.0, 1.0);
Servo servo_pit;
Servo servo_rol;

// important data
float groundLevelPressure = 101325.0;
bool TX_start = false;
byte lastPayloadState = PAYLOAD_STANDBY;

char transmit[256];
char received[256];
byte idx_tx = 0;
byte idx_rx = 0;

unsigned long int lastTransmit = 0;
unsigned long int lastSensorUpdate = 0;
unsigned long int release_time = 0;

void setup() {
    Serial.begin(115200);
    xbee.begin(115200);

    //EEPROM.get(0, groundLevelPressure);
    EEPROM.get(4, TX_start);
    EEPROM.get(5, lastPayloadState);
    bmp.setReferenceAltitude(groundLevelPressure);
    payload.softwareState = lastPayloadState;

    servo_yaw.attach(3);
    servo_pit.attach(4);
    servo_rol.attach(5);
    
    servo_yaw.setDirection(SERVO_CW);
//    servo_pit.setDirection(SERVO_CCW);
//    servo_rol.setDirection(SERVO_CW);

    servo_yaw.write(90);
    servo_pit.write(90);
    servo_rol.write(90);

    //Serial.println("Start!!");
    delay(1000);
}

void loop() {
    // listen to xbee and print to serial monitor
    if (xbee.available()) {
        do {
            received[idx_rx] = xbee.read();
            idx_rx++;
            if(received[idx_rx-1] == '\r') {
                Serial.print("Received: ");
                Serial.println(received);
                
                if(strncmp(received, "PL_ECHO", 7) == 0)
                    xbee.print("PL_ECHO\r");
                
                if(strncmp(received, "PL_CMD,RELEASED", 15) == 0) {
                    if (payload.softwareState == PAYLOAD_STANDBY) {
                        payload.softwareState = PAYLOAD_RELEASED;
                        TX_start = true;
                        EEPROM.put(4, TX_start);
                        release_time = millis();
                        lastTransmit = millis();
                        xbee.print(payload.constructMessage());
                    }
                }
                if(strncmp(received, "PL_CMD,CALIBRATE", 16) == 0) {
                    groundLevelPressure = bmp.getPressure();
                    bmp.setReferenceAltitude(groundLevelPressure);
                    //EEPROM.put(0, groundLevelPressure);
                }
                if(strncmp(received, "PL_CMD,RESET", 12) == 0) {
                    TX_start = false;
                    EEPROM.put(4, TX_start);
                    payload.softwareState = PAYLOAD_STANDBY;
                }

                while(idx_rx--)
                    received[idx_rx] = '\0';
                
                idx_rx = 0;
            }
        } while(xbee.available() > 0);
    }
    // listen to serial and transmit to xbee
//    if (Serial.available()) {
//        do {
//            transmit[idx_tx] = Serial.read();
//            idx_tx++;
//        }
//        while(Serial.available() > 0);
//        
//        Serial.print("Transmitting: "); Serial.println(transmit);
//        xbee.print(transmit);
//        xbee.print("\r");
//        
//        while(idx_tx--)
//            transmit[idx_tx] = '\0';
//        
//        idx_tx = 0;
//    }
    
    if (millis() - lastSensorUpdate > SENSOR_UPDATE_INTERVAL) {
        lastSensorUpdate = millis();
        sensor_update();
        determinePayloadSoftwareState();
    }

    // periodic transmission
    if (TX_start && (millis() - lastTransmit) >= TRANSMIT_INTERVAL) {
        lastTransmit = millis();
        xbee.print(payload.constructMessage());
    }
}

void sensor_update() {
    // update telemetry data
    bmp.read();
    payload.altitude = bmp.getAltitude();
    payload.temp = bmp.getTemperature();
    payload.voltage = vsense.readVoltage();

    cam_imu.update();
    payload.imu.gyroR = cam_imu.gyroR;
    payload.imu.gyroP = cam_imu.gyroP;
    payload.imu.gyroY = cam_imu.gyroY;
    payload.imu.accelR = cam_imu.accelR;
    payload.imu.accelP = cam_imu.accelP;
    payload.imu.accelY = cam_imu.accelY;
    payload.imu.magR = cam_imu.magR;
    payload.imu.magP = cam_imu.magP;
    payload.imu.magY = cam_imu.magY;
    payload.pointingError = abs(cam_imu.yaw - 180.0);

    yaw = cam_imu.yaw;
    pit = cam_imu.pitch;
    rol = cam_imu.pitch;
}

void determinePayloadSoftwareState() {
    switch(payload.softwareState){
        case PAYLOAD_STANDBY:
            servo_yaw.write(90);
            servo_pit.write(90);
            servo_rol.write(90);
            break;
        case PAYLOAD_RELEASED:
            payload.softwareState = payload.pointingError < 10.0
                                  ? PAYLOAD_TARGET_ACQUIRING
                                  : PAYLOAD_TARGET_POINTING;
            servo_yaw.start();
//            servo_pit.start();
//            servo_rol.start();
            break;
        case PAYLOAD_TARGET_ACQUIRING:
        case PAYLOAD_TARGET_POINTING:
            servo_yaw.update();
//            servo_pit.update();
//            servo_rol.update();
//            servo_pit.write(pit);
//            servo_pit.write(rol);
                   
            payload.softwareState = payload.pointingError < 10.0
                                  ? PAYLOAD_TARGET_ACQUIRING
                                  : PAYLOAD_TARGET_POINTING;
            if (payload.altitude <= 5.0) {
                payload.softwareState = PAYLOAD_LANDED;
            }
            break;
        case PAYLOAD_LANDED:
            servo_yaw.write(90);
            servo_pit.write(90);
            servo_rol.write(90);
            TX_start = false;
            EEPROM.put(4, TX_start);
            lastTransmit = millis();
            xbee.print(payload.constructMessage());
            break;
        default:
            break;
    }
    
//    if (lastPayloadState != payload.softwareState) {
//        lastPayloadState = payload.softwareState;
//        EEPROM.put(5, lastPayloadState);
//    }
}
