/*
 * Narantaka @ 2022 CanSat Competition
 * Firmware for cansat container
 * Using Teensy 4.0 microcontroller
 *
 * Communication:
 * * GCSLINK (container to GCS):
 *   2.4 GHz XBEE-PRO S2C using 115200 8-N-1
 *   Use Serial1 ports (0 and 1)
 * * PLINK (container to payload):
 *   900 MHz XBEE-PRO S3B using 115200 8-N-1
 *   Use Serial2 ports (7 and 8)
 *
 * - Ananda Hafidh R K (github.com/hfrk)
 */

// Serial communication
#define PLINK Serial1
#define GCSLINK Serial2
#define GPSPORT Serial3

// Pin declaration
#define BUZZER_PIN 2
#define SERVO_RELEASE_PIN 16
#define DEPLOY_MOTOR_PIN 6

// Interval for nonblocking loop
#define TRANSMIT_INTERVAL 1000
#define SENSOR_UPDATE_INTERVAL 250

// Servo positions (using 180 deg servo)
#define SERVO_RELEASE_STILL 180
#define SERVO_RELEASE_CHUTE 150
#define SERVO_RELEASE_PLOCK  75

#include "telemetry.h"          // telemetry structure
#include "sensors/bmp388.h"     // pressure sensor
#include "sensors/voltage.h"    // ADC voltage reader
#include <TinyGPS++.h>

#include "Servo.h"

// OV2640 camera
#include <SD.h>
#include <Wire.h>
#include "ArduCAM.h"
#include <SPI.h>
#include "memorysaver.h"
#include <EEPROM.h>

Telemetry telemetry = Telemetry();

Sensor_BMP388 bmp = Sensor_BMP388(0x76);
Sensor_Voltage vsense = Sensor_Voltage(A7);
TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;

Servo servo_release;

const int CS = 10;
const int SD_CS = BUILTIN_SDCARD;
ArduCAM myCAM(OV2640, CS);
File logFile;
File imageFile;

// important data
float groundLevelPressure = 101325.0;
unsigned long int packetCount = 0;
bool TX_start = false;
bool sim_mode = false;
bool tp_released = false;
byte lastContainerState = CONTAINER_STANDBY;
bool recording = false;

char transmit[256];
char received[256];
byte idx_tx = 0;
byte idx_rx = 0;

unsigned long int lastTransmit = 0;
unsigned long int lastSensorUpdate = 0;
unsigned long int release_time = 0;

void setup() {
    Serial.begin(115200);
    PLINK.begin(115200);
    GCSLINK.begin(115200);
    GPSPORT.begin(GPSBaud);
    
    EEPROM.get(0, packetCount);
    EEPROM.get(8, TX_start);
    EEPROM.get(9, tp_released);
    EEPROM.get(10, lastContainerState);
    EEPROM.get(11, recording);

    pinMode(CS,OUTPUT);
    SPI.begin();
    SD.begin(SD_CS);
    delay(1000);
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    if (myCAM.read_reg(ARDUCHIP_TEST1) != 0x55) {
        logger("SPI1 interface Error!");
    }

    // buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // release motor
    pinMode(DEPLOY_MOTOR_PIN, OUTPUT);
    analogWriteResolution(8);
    analogWrite(DEPLOY_MOTOR_PIN, 0);

    // release servo
    servo_release.attach(SERVO_RELEASE_PIN);
    servo_release.write(SERVO_RELEASE_STILL);
    bmp.setReferenceAltitude(groundLevelPressure);
    telemetry.setPacketCount(packetCount);
    telemetry.container.softwareState = lastContainerState;

    myCAM.set_format(JPEG);
    myCAM.InitCAM();
    myCAM.OV2640_set_JPEG_size(OV2640_640x480);
    // myCAM.wrSensorReg8_8(0xff, 0x01);
    // myCAM.wrSensorReg8_8(0x13, 0xe4);
    // myCAM.wrSensorReg8_8(0x45, 0x00);
    // myCAM.wrSensorReg8_8(0x10, 0x4e);
    // myCAM.wrSensorReg8_8(0x04, 0x00);
    // myCAM.wrSensorReg8_8(0xff, 0xff);
    myCAM.CS_HIGH();
    //Serial.println("Start!!");

}

void loop() {
    // receive from PLINK and forward to GCSLINK
    // listen to GCSLINK
    while (GCSLINK.available()) {
        received[idx_rx] = GCSLINK.read();
        idx_rx++;
        if(received[idx_rx-1] == '\r') {
            Serial.print("Received from GCS: ");
            Serial.println(received);
      
            if (strncmp(received, "ECHO", 4) == 0)
                GCSLINK.print("ECHO\r");
            if (strncmp(received, "PL_ECHO", 7) == 0)
                PLINK.print("PL_ECHO\r");
            if (strncmp(received, "CMD,1025,", 9) == 0)
                command_parse(received+9);

            while(idx_rx--)
                received[idx_rx] = '\0';
            
            idx_rx = 0;
        }
    }

    // listen to PLINK
    while (PLINK.available()) {
        transmit[idx_tx] = PLINK.read();
        idx_tx++;
        if(transmit[idx_tx-1] == '\r') {
            GCSprint('T');
            
            while(idx_tx--)
                transmit[idx_tx] = '\0';
            
            idx_tx = 0;
        }
    }

    // decode GPS data
    while (GPSPORT.available()) {
        gps.encode(GPSPORT.read());
    }

   if (millis() - lastSensorUpdate > SENSOR_UPDATE_INTERVAL) {
       lastSensorUpdate = millis();
       sensor_update();
       determineContainerSoftwareState();
   }
    // periodic transmission
    if ((millis() - lastTransmit) >= TRANSMIT_INTERVAL) {
        lastTransmit = millis();//sensor_update();
        GCSprint('C');
    }
    if (recording) {
        if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
            // capture done, restart capture
            myCAM.clear_fifo_flag();
            myCAM.start_capture();
        }
        else {
            // read 1024 byte
            myCAM.CS_LOW();
            myCAM.set_fifo_burst();
            byte buf[1024];
            for (int i = 0; i < 1024; i++) {
                buf[i] = SPI.transfer(0x00);
            }
            myCAM.CS_HIGH();
            imageFile = SD.open("image.jpg", O_WRITE | O_CREAT | O_TRUNC);
            imageFile.write(buf + 0, 256);
            imageFile.write(buf + 256, 256);
            imageFile.write(buf + 512, 256);
            imageFile.write(buf + 768, 256);
            imageFile.close();
        }
    }
}

void GCSprint(char packet_type) {
    // Transmit and log if commanded to start transmission
    if (!TX_start)
        return;

    String message;
    switch(packet_type) {
        case 'C':
            // container telemetry
            message = telemetry.constructContainerMessage();
            break;
        case 'T':
            // payload telemetry
            message = telemetry.constructPayloadMessage(transmit);
            break;
        default:
            break;
    }
    GCSLINK.print(message);
    logger(message);
}

void logger(String message) {
    // log message to SD card
    logFile = SD.open("cansat.log", O_WRITE | O_CREAT | O_TRUNC);

    if (logFile) {
        logFile.println(message);
        logFile.close();
    }
}

void command_parse(char* command) {
    // parse command (CMD,1025,command)
    // main command
    if (strncmp(command, "CX,", 3) == 0) {
        // Start/stop command
        if (strncmp(command+3, "ON", 2) == 0) {
            TX_start = true;
            telemetry.container.cmdEcho = "CXON";
        }
        else if (strncmp(command+3, "OFF", 3) == 0) {
            TX_start = false;
            telemetry.container.cmdEcho = "CXOFF";
        }
    }
    else if (strncmp(command, "ST,", 3) == 0) {
        // Set time
        // ST,hh:mm:ss
//        referenceTime.millis = millis();
//        referenceTime.hh = (*(command+3) - '0') * 10
//                         + (*(command+4) - '0');
//        referenceTime.mm = (*(command+6) - '0') * 10
//                         + (*(command+7) - '0');
//        referenceTime.ss = (*(command+9) - '0') * 10
//                         + (*(command+10) - '0');
    }
    else if (strncmp(command, "SIM,", 4) == 0) {
        // Simulation control
        // to be implemented
    }
    else if (strncmp(command, "SIMP,", 5) == 0) {
        // Simulation pressure
        // to be implemented
    }

    // auxillary command
    else if (strncmp(command, "RESET", 5) == 0) {
        // Reset data
        packetCount = 0;
        sim_mode = false;
        tp_released = false;
        lastContainerState = CONTAINER_STANDBY;
        recording = false;
        telemetry.setPacketCount(packetCount);
        telemetry.container.softwareState = lastContainerState;
        digitalWrite(BUZZER_PIN, LOW);
        PLINK.print("PL_CMD,RESET\r");
        telemetry.container.cmdEcho = "RESET";
    }
    else if (strncmp(command, "RELEASE", 7) == 0) {
        // Manual release
        // turn on motor for 10 sec
        tp_released = true;
        release_time = millis();
        PLINK.print("PL_CMD,RELEASED\r");
        telemetry.container.softwareState = CONTAINER_TP_RELEASE;
        telemetry.container.cmdEcho = "RELEASE";
    }
    else if (strncmp(command, "CALIBRATE", 9) == 0) {
        // Calibrate altimeter
        groundLevelPressure = bmp.getPressure();
        bmp.setReferenceAltitude(groundLevelPressure);
        PLINK.print("PL_CMD,CALIBRATE\r");
        telemetry.container.cmdEcho = "CALIBRATE";
    }
}

void sensor_update() {
    // update telemetry data
    telemetry.container.mode = sim_mode ? 'S' : 'F';
    telemetry.container.tpReleased = tp_released ? 'R' : 'N';

    bmp.read();
    telemetry.container.altitude = bmp.getAltitude();
    telemetry.container.temp = bmp.getTemperature();
    telemetry.container.voltage = vsense.readVoltage();

    if (gps.time.isValid()) {
        telemetry.container.hhGPS = gps.time.hour();
        telemetry.container.mmGPS = gps.time.minute();
        telemetry.container.ssGPS = gps.time.second();
    }
    else {
        // read from RTC (?)
    }
    if (gps.location.isValid()) {
        telemetry.container.latitudeGPS = gps.location.lat();
        telemetry.container.longitudeGPS = gps.location.lng();      
    }
    telemetry.container.altitudeGPS = gps.altitude.meters();
    telemetry.container.satsGPS = gps.satellites.value();
  
    // save important data to EEPROM
    EEPROM.put(0, packetCount);
    EEPROM.put(8, TX_start);
    EEPROM.put(9, tp_released);
    EEPROM.put(10, lastContainerState);
    EEPROM.put(11, recording);
}

void determineContainerSoftwareState() {
    switch(telemetry.container.softwareState) {
        case CONTAINER_STANDBY:
            if (telemetry.container.altitude >= 5.0)
                telemetry.container.softwareState = CONTAINER_ASCENT;
            break;
        case CONTAINER_ASCENT:
            if (telemetry.container.altitude >= 700.0)
                telemetry.container.softwareState = CONTAINER_SEPARATION;
            break;
        case CONTAINER_SEPARATION:
            if (telemetry.container.altitude <= 400.0) {
                servo_release.write(SERVO_RELEASE_CHUTE);
                telemetry.container.softwareState = CONTAINER_DESCENT;
            }
            break;
        case CONTAINER_DESCENT:
            if (telemetry.container.altitude <= 300.0) {
                // release payload
                servo_release.write(SERVO_RELEASE_PLOCK);
                tp_released = true;
                release_time = millis();
                // start container camera video recording
                myCAM.clear_fifo_flag();
                myCAM.start_capture();
                recording = true;
                telemetry.container.softwareState = CONTAINER_TP_RELEASE;
                PLINK.print("PL_CMD,RELEASED\r");
            }
            break;
        case CONTAINER_TP_RELEASE:
            if (tp_released && (millis() - release_time) < 20000) {
                // run tether motor for 20 sec
                if ((millis() - release_time) % 1000 < 666) {
                    analogWrite(DEPLOY_MOTOR_PIN, 256);
                }
                else {
                    analogWrite(DEPLOY_MOTOR_PIN, 0);
                }
            }
            else {
                analogWrite(DEPLOY_MOTOR_PIN, 0);
                if (telemetry.container.altitude <= 5.0)
                    telemetry.container.softwareState = CONTAINER_LANDED;
            }
            break;
        case CONTAINER_LANDED:
            digitalWrite(BUZZER_PIN, HIGH);
            TX_start = false;
            break;
        default:
            break;
    }
}
