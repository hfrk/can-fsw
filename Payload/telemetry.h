/*
 * Library to provide CanSat telemetry data encapsulation
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "Arduino.h"

enum CT_SoftwareState {
    CONTAINER_STANDBY     =  0,
    CONTAINER_ASCENT      =  1,
    CONTAINER_SEPARATION  =  2,
    CONTAINER_DESCENT     =  3,
    CONTAINER_TP_RELEASE  =  4,
    CONTAINER_LANDED      =  5,
    CONTAINER_INVALID     = -1
};

enum TP_SoftwareState {
    PAYLOAD_STANDBY           =  0,
    PAYLOAD_RELEASED          =  1,
    PAYLOAD_TARGET_ACQUIRING  =  2,
    PAYLOAD_TARGET_POINTING   =  3,
    PAYLOAD_LANDED            =  4,
    PAYLOAD_INVALID           = -1
};

class ContainerTelemetry {
public:
  ContainerTelemetry(){};
  String constructMessage() {
    memset(buffer, 0, sizeof(buffer));
    snprintf(buffer, sizeof(buffer),
              "%c,%c,"
              "%.1f,%.1f,%.2f,"
              "%02d:%02d:%02d,"
              "%.4f,%.4f,%.1f,"
              "%ld,%s,%s\r",
              mode, tpReleased,
              altitude, temp, voltage,
              hhGPS, mmGPS, ssGPS,
              latitudeGPS, longitudeGPS, altitudeGPS,
              satsGPS, getSoftwareState(), cmdEcho.c_str()
            );
    return String(buffer);    
  }

  /* FSW */
  char mode = 'F';
  char tpReleased = 'N';
  
  /* Pressure sensor */
  float altitude;
  float temp;

  /* Voltage divider */
  float voltage;

  /* GPS */
  //String timeGPS;
  uint8_t hhGPS;
  uint8_t mmGPS;
  uint8_t ssGPS;
  float latitudeGPS;
  float longitudeGPS;
  float altitudeGPS;
  unsigned long satsGPS = 0;

  /* FSW */
  byte softwareState;
  String cmdEcho;

private:
    const char* getSoftwareState() {
        //int rand = random(0,4);
        switch(softwareState){
            case CONTAINER_STANDBY:
                return "STANDBY";
            case CONTAINER_ASCENT:
                return "ASCENT";
            case CONTAINER_SEPARATION:
                return "SEPARATION";
            case CONTAINER_DESCENT:
                return "DESCENT";
            case CONTAINER_TP_RELEASE:
                return "TP_RELEASE";
            case CONTAINER_LANDED:
                return "LANDED";
            default:
                return "INVALID";
        }
    }

    char buffer[256];
};

class PayloadTelemetry {
public:
  PayloadTelemetry(){};
  String constructMessage() {
    memset(buffer, 0, sizeof(buffer));
    snprintf(buffer, sizeof(buffer),
              "%.1f,%.1f,%.2f,"
              "%.1f,%.1f,%.1f,"
              "%.2f,%.2f,%.2f,"
              "%.3f,%.3f,%.3f,"
              "%.2f,%s\r",
              altitude, temp, voltage,
              imu.gyroR, imu.gyroP, imu.gyroY,
              imu.accelR, imu.accelP, imu.accelY,
              imu.magR, imu.magP, imu.magY,
              pointingError, getTpSoftwareState()
            );
    return String(buffer);
  }

  /* Pressure sensor */
  float altitude;
  float temp;

  /* Voltage divider */
  float voltage;
  
  /* IMU */
  struct IMU {
    float gyroR, gyroP, gyroY;
    float accelR, accelP, accelY;
    float magR, magP, magY;
  } imu;
  float pointingError = 0;
  
  /* FSW */
  byte softwareState = PAYLOAD_STANDBY;

private:
    const char* getTpSoftwareState() {
        //int rand = random(0,4);
        switch(softwareState){
            case PAYLOAD_STANDBY:
                return "STANDBY";
            case PAYLOAD_RELEASED:
                return "RELEASED";
            case PAYLOAD_TARGET_ACQUIRING:
                return "TARGET_ACQUIRING";
            case PAYLOAD_TARGET_POINTING:
                return "TARGET_POINTING";
            case PAYLOAD_LANDED:
                return "LANDED";
            default:
                return "INVALID";
        }
    }
  
    char buffer[256];
};

class Telemetry {
public:
  Telemetry(){};
  String constructContainerMessage() {
      return constructHeader('C') + container.constructMessage();
  }
  String constructPayloadMessage(char* data) {
      return constructHeader('T') + String(data);
  }
  void setPacketCount(unsigned long count) {
      packetCount = count;
  };
  ContainerTelemetry container;
  PayloadTelemetry payload;
  
private:
  String constructHeader(char packetType) {
    char header[32];
    snprintf(header, sizeof(header),
              "%s,"
              "%02d:%02d:%02d,"
              "%ld,%c,",
              teamID,
              container.hhGPS, container.mmGPS, container.ssGPS,
              ++packetCount, packetType
            );
    return String(header);
  }

  const char teamID[5] = "1025";
  //String timeString;
  unsigned long packetCount = 0;
  //String packetType = "C"
  char buffer[256];
};

#endif /* TELEMETRY_H */
