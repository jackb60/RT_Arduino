#include <SPI.h>
#include <CC1200.h>
#include <baro.h>
#include <ADXL357.h>
#include <gyro.h>
#include <flash.h>
#include <cam.h>
#include <vtx.h>
#include <pyro.h>
#include <GPS.h>

#include <myTypes.h>

#define MOSI PD7
#define MISO PB4
#define SCLK PA5
#define MAG_CS PD0
#define GYRO_CS PD1
#define FLASH_CS PD4
#define ACCEL_CS PD2
#define BARO_CS  PD3
#define CC_CS PC12


HardwareSerial debugSer(PC7, PC6);
HardwareSerial gpsSer(PB12, PB13);
HardwareSerial CAM0_SER(PE7, PE8);
HardwareSerial CAM1_SER(PE0, PE1);
HardwareSerial CAM2_SER(PB15, PB14);
HardwareSerial VTX_SER(PA12);


SPIClass SPI_3(MOSI, MISO, SCLK, -1);
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

CC1200 cc(&SPI_3, settings, CC_CS);
ADXL357 accel(&SPI_3, settings, ACCEL_CS);
baro barometer(&SPI_3, settings, BARO_CS);
gyro mygyro(&SPI_3, settings, GYRO_CS);
flash flashMem(&SPI_3, settings, FLASH_CS);

cam cam0(&CAM0_SER);
cam cam1(&CAM1_SER);
vtx myVTX(&VTX_SER);
GPS gps(&gpsSer);

pyro pyros;

State currentState = GROUND_TESTING;

State recState = GROUND_TESTING;

uint32_t loopBegin;
uint32_t lastTelem;

int8_t rxrssi;
unsigned long lastRec;
uint16_t packetNum;
uint8_t badPackets;

HardwareTimer *myTim = new HardwareTimer(TIM4);

void setup() {
  // put your setup code here, to run once:
  pinMode(MAG_CS, OUTPUT); //Remove when mag library added
  digitalWrite(MAG_CS, 1); //Remove when mag library added
  pyros.begin();

  myTim->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PD12);
  myTim->setOverflow(20000, MICROSEC_FORMAT);
  myTim->setCaptureCompare(1, degToUs(0), MICROSEC_COMPARE_FORMAT);
  myTim->resume();

  SPI_3.begin();
  debugSer.begin(115200);

  delay(100);

  cc.begin();
  cc.simpleConfig();
  accel.setup();
  barometer.begin();
  mygyro.begin();
  mygyro.config();
  flashMem.begin();

  cam0.begin();
  cam1.begin();
  myVTX.begin();
  gps.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  loopBegin = millis();
  barometer.updateRawTemp();
  barometer.updateRawPress();
  accel.update();
  mygyro.update();
  gps.update();
  pyros.update(currentState);


  //to-do: state machine
  //to-do: flash logging handler
  //to-do: power board handler

  if (millis() - lastTelem > 50) {
    lastTelem = millis();
    constructTelemetryPacket();
    sendTelemetryPacket();
  }
  readTelem();

  while (millis() - loopBegin < 10) {}
}

///////////////////////////////////////////////////
//                    Telemetry                  //
///////////////////////////////////////////////////
uint8_t telemPkt[128];
void constructTelemetryPacket() {
  uint16_t pyrosStatus = pyros.getPyrosStatus();
  memcpy(&telemPkt[0], &pyrosStatus, 2);

  for (uint8_t i = 0; i < 6; i++) {
    telemPkt[26 + i] = pyros.resistance(i) * 10;
  }

  uint16_t gyrorawX = mygyro.getRawX();
  uint16_t gyrorawY = mygyro.getRawY();
  uint16_t gyrorawZ = mygyro.getRawZ();
  memcpy(&telemPkt[35], &gyrorawX, 2);
  memcpy(&telemPkt[37], &gyrorawY, 2);
  memcpy(&telemPkt[39], &gyrorawZ, 2);

  uint32_t barorawTemp = barometer.getRawTemp();
  memcpy(&telemPkt[23], &barorawTemp, 3);

  //GPS
  uint8_t gpsFix = gps.getFixType();
  uint32_t lat = gps.getLat();
  uint32_t lon = gps.getLon();
  uint32_t alt = gps.getHeight();
  uint32_t hACC = gps.getHAcc();
  uint32_t vACC = gps.getVAcc();

  memcpy(&telemPkt[41], &gpsFix, 1);
  memcpy(&telemPkt[42], &lat, 4);
  memcpy(&telemPkt[46], &lon, 4);
  memcpy(&telemPkt[50], &alt, 4);


  uint8_t armed_byte = 0;
  for(uint8_t i = 0; i < 6; i++) {
    armed_byte |= pyros.isArmed(i) << i;
  }
  memcpy(&telemPkt[103], &armed_byte, 1);

  uint8_t fired_byte = 0;
  for(uint8_t i = 0; i < 6; i++) {
    fired_byte |= pyros.isFired(i) << i;
  }
  memcpy(&telemPkt[104], &fired_byte, 1);
  
  telemPkt[127] = calcChecksum();
}

uint8_t calcChecksum() {
  uint8_t sum = 0;
  for(int i = 0; i < 127; i++) {
    sum += telemPkt[i];
  }
  return sum;
}

void sendTelemetryPacket() {
  if (cc.status() == 7) {
    cc.flushTx();
  }
  cc.Tx(telemPkt, 128);
}

void readTelem() {
  if(cc.status() == 0) {
    cc.Rx(16);
  }
  if(cc.avail() >= 16) {
    lastRec = millis();
    byte recBuf[16];
    cc.read(recBuf, 16);
    rxrssi = cc.rssi();
      if(recBuf[0] == 0xAA) {
      uint16_t chksum = 0;
      for (int i = 1; i < 14; i++) {
        chksum += recBuf[i];
      }

      if (chksum == (recBuf[14] << 8) + recBuf[15]) {
        bool recValid = true;
        switch (recBuf[13]) {
          case 0x01:
            for (uint8_t i = 1; i < 12; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
              for(uint8_t i = 0; i < 8; i++) {
                if ((recBuf[12] >> i) & 0b1) {
                  pyros.arm(i);
                }
              }
            }
            break;
          case 0x06:
            for (uint8_t i = 1; i < 12; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
              for(uint8_t i = 0; i < 8; i++) {
                if ((recBuf[12] >> i) & 0b1) {
                  pyros.fire(i);
                }
              }
            }
            break;
          case 0x03:
            for (uint8_t i = 1; i < 9; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
              float angle = *((float*) &recBuf[9]);
              myTim->setCaptureCompare(1, degToUs(angle), MICROSEC_COMPARE_FORMAT);
            }
            break;
        }
      } else {
        badPackets++;
      }
    } else {
      badPackets++;
    }
  }
}

uint16_t degToUs(float degrees) {
  return 1500.0 + (degrees / 60.0) * 500.0; 
}