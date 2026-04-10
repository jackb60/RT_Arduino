#include <SPI.h>
#include <CC1200.h>
#include <DUMMYbaro.h>
#include <DUMMYADXL357.h>
#include <DUMMYgyro.h>
#include <flash.h>
#include <cam.h>
#include <vtx.h>
#include <pyro.h>
#include <DUMMYGPS.h>
#include <power.h>
#include <airbrakes.h>
#include <rollcontrol.h>

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

#define AIRBRAKES_CLOSED_ANGLE -67.0f
#define AIRBRAKES_OPEN_ANGLE -117.0f

#define SERVO2OFFSET -0.5f
#define SERVO3OFFSET -15.0f

volatile bool airbrakesEnabled = false;
volatile bool rollControlEnabled = false;

volatile float airbrakesSetAngle = AIRBRAKES_CLOSED_ANGLE;
volatile float rollControlSetAngle = 0;

volatile uint16_t servo0us;
volatile uint16_t servo1us = 1500; //unused
volatile uint16_t servo2us;
volatile uint16_t servo3us;

bool loggingEnabled = true;

uint8_t telemPkt[128];

HardwareSerial debugSer(PC7, PC6);
HardwareSerial gpsSer(PB12, PB13);
HardwareSerial CAM0_SER(PE7, PE8);
HardwareSerial CAM1_SER(PE0, PE1);
HardwareSerial CAM2_SER(PB15, PB14);
HardwareSerial VTX_SER(PA12);
HardwareSerial pwrSer(PB11, PB10);


SPIClass SPI_3(MOSI, MISO, SCLK, -1);
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

CC1200 cc(&SPI_3, settings, CC_CS);
DUMMYADXL357 accel(&SPI_3, settings, ACCEL_CS);
DUMMYbaro barometer(&SPI_3, settings, BARO_CS);
DUMMYgyro mygyro(&SPI_3, settings, GYRO_CS);
flash flashMem(&SPI_3, settings, FLASH_CS);

cam cam0(&CAM0_SER);
cam cam1(&CAM1_SER);
vtx myVTX(&VTX_SER);
DUMMYGPS gps(&gpsSer);
power pwr(&pwrSer);

airbrakes myairbrakes;
rollcontrol myrollcontrol;

pyro pyros;

State currentState = GROUND_TESTING;

State recState = GROUND_TESTING;

bool simEnabled;
uint32_t simStartTime;
uint32_t simTime;

uint32_t FCtime;

uint32_t loopBegin;
uint32_t lastTelem;
uint32_t lastPowerPkt;

int8_t rxrssi;
unsigned long lastRec;
uint16_t packetNum;
uint8_t badPackets;

uint32_t flightBeginTime;

pwrBoardData powerPkt;
uint8_t tmp[sizeof(powerPkt)];

pwrCommands pwrCommand;

HardwareTimer *myTim = new HardwareTimer(TIM4);

void setup() {
  // put your setup code here, to run once:
  pinMode(MAG_CS, OUTPUT); //Remove when mag library added
  digitalWrite(MAG_CS, 1); //Remove when mag library added
  pyros.begin();

  myTim->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PD12);
  myTim->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PD14);
  myTim->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, PD15);
  myTim->setOverflow(20000, MICROSEC_FORMAT);
  myTim->setCaptureCompare(1, degToUsAirbrakes(AIRBRAKES_CLOSED_ANGLE), MICROSEC_COMPARE_FORMAT);
  myTim->setCaptureCompare(3, degToUsRollControl(SERVO2OFFSET), MICROSEC_COMPARE_FORMAT);
  myTim->setCaptureCompare(4, degToUsRollControl(SERVO3OFFSET), MICROSEC_COMPARE_FORMAT);
  myTim->attachInterrupt(Update_IT_callback);
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
  pwr.begin();

  myairbrakes.begin();
  myrollcontrol.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  loopBegin = millis();
  if (simTime < 208000 && simEnabled) {
    simTime = millis() - simStartTime;
  }
  barometer.updateAll(simTime);
  accel.update(currentState, simTime);
  mygyro.update(simTime);
  gps.update(simTime);
  pyros.update(currentState);
  pwr.update();

  FCtime = millis();
  handleState();
  if (rollControlEnabled) {
    updateRollControl();
  }
  
  if (airbrakesEnabled) {
    updateAirbrakes();
  }

  if (millis() - lastTelem > 50) {
    lastTelem = millis();
    constructTelemetryPacket();
    sendTelemetryPacket();
    handleLogging();
  }
  readTelem();

  if (millis() - lastPowerPkt > 100) {
    lastPowerPkt = millis();
    pwrSer.write(0xAA);
    pwrSer.write((uint8_t*) &pwrCommand, sizeof(pwrCommand));
    pwrSer.write(calcChecksumP((uint8_t*) &pwrCommand, sizeof(pwrCommand)));
  }

  while (millis() - loopBegin < 10) {}
}
///////////////////////////////////////////////////
//                  State Machine                //
///////////////////////////////////////////////////
#define ACCEL_FLIGHT_THRESHOLD   30    //m/s^2
#define T_APOGEE_LOCKOUT         27000 //msec
#define T_APOGEE_OVERRIDE        35000 //msec
#define T_BP_DEPLOY1             3000  //msec (PAST T_APOGEE)
#define T_BP_DEPLOY2             5000  //msec (PAST T_APOGEE)
#define T_MAIN_LOCKOUT           55000 //msec (PAST T_APOGEE)
#define APOGEE_DROP              20    //m
#define MAIN_DEPLOY_ALT          457   //m (AGL)

//uint32_t flightBeginTime;            //time flight mode entered; initialized earlier
uint32_t apogeeTime;                   //time apogee reached
//uint32_t FCtime;                     //time handleState called (msec); initialized earlier
//State recState;                      //state recieved from ground station if manually advanced; initialized earlier
//State currentState;                  //state rocket is in; initialized earlier
bool bpFired1 = false;
bool bpFired2 = false;
bool baroMaxAltReset = false;

void handleState() {
  switch(currentState) {
    case GROUND_TESTING:                                                                      //If we are in ground testing mode
      if (recState == PRE_FLIGHT) {                                                             //If we recieve the signal to enter preflight mode
        currentState = PRE_FLIGHT;                                                              //Enter into preflight mode
        allocateFlash();                                                                        //Clear some flash
        loggingEnabled = true;                                                                  //Begin logging
        mygyro.update(simTime);                                                                        //Prevent large dt on first update post zero

        mygyro.zeroRollPitchYaw();                                                              //Zero roll, pitch, yaw
        accel.update(currentState, simTime);

        accel.zeroIntegratedVelo();                                                             //Zero integrated velocity
        gps.zeroAlt();                                                                          //Zero alt
        barometer.zeroAlt();
        rollControlSetAngle = 0;                                                                //Zero roll control, airbrakes 
        airbrakesSetAngle = AIRBRAKES_CLOSED_ANGLE;
        airbrakesEnabled = false;
        rollControlEnabled = false;
        //pwrCommand.BMS.protectionsEnabled = false;                                              //Disable BMS protections and screw switch functionality
        //pwrCommand.BMS.screwSwitchEnabled = false;
        for (uint8_t i = 0; i < 6; i++) {                                                       //Enable all converters
          pwrCommand.convertersEnabled[i] = true;
        }
        simStartTime = millis();
        simEnabled = true;
      }
      break;
    case PRE_FLIGHT:                                                                          //If we are in preflight mode
      if ((recState == FLIGHT) ||                                                               //If we recieve the signal to enter 
      (accel.getVerticalAccelMinusGravity() > ACCEL_FLIGHT_THRESHOLD)) {                        //Or If we experience large acceleration
        currentState = FLIGHT;                                                                    //Enter into flight mode
        flightBeginTime = millis();                                                               //Mark the time we entered flight mode
        FCtime = millis();                                                                        //Update FCtime so it is never less than flightBeginTime
        mygyro.zeroRollPitchYaw();                                                                //Zero roll, pitch, yaw                                                  
        updateAirbrakes();                                                                        //Begin roll control, airbrakes
        updateRollControl();
        airbrakesEnabled = true;
        rollControlEnabled = true;
        myVTX.setPower(3);                                                                        //Set VTX to 8W power
      }
      break;
    case FLIGHT:                                                                              //If we are in flight state
      if (FCtime - flightBeginTime > T_APOGEE_LOCKOUT) {                                        //If we are past the apogee lockout time
        if (!baroMaxAltReset) {
          barometer.resetMaxAlt();
          baroMaxAltReset = true;
        }
        if (recState == APOGEE ||                                                                 //If we recieve the signal to enter apogee state
        ((gps.getFixType() == 3) && (gps.getMaxAlt() > gps.getHeight() + APOGEE_DROP)) ||         //Or If we have 3D GPS fix and have dropped an amount from GPS max alt
        (barometer.getMaxAlt() > barometer.getFilteredAltitude() + APOGEE_DROP)) {                //Or If we have dropped an amount as measured by barometer
          currentState = APOGEE;                                                                    //Advance to Apogee
        }
      }
      if (FCtime - flightBeginTime > T_APOGEE_OVERRIDE) {                                     //If we have passed apogee override time
        currentState = APOGEE;                                                                  //Advance to Apogee
      }
      if (currentState == APOGEE) {                                                           //If we are advancing to apogee
        apogeeTime = millis();                                                                  //Mark time we entered apogee
        FCtime = millis();                                                                      //Update FCtime so it is never less than apogeeTime
        for (uint8_t i = 0; i < 2; i++) {                                                       //Fire Pyros 0, 1
          pyros.arm(i);
          pyros.fire(i);
        }
        rollControlSetAngle = 0;                                                                //Zero roll control, airbrakes 
        airbrakesSetAngle = AIRBRAKES_CLOSED_ANGLE;
        airbrakesEnabled = false;
        rollControlEnabled = false;
      }
      break;
    case APOGEE:                                                                              //If we are in apogee state
      if (FCtime - apogeeTime > T_BP_DEPLOY1 && !bpFired1) {                                      //If we are more than some amount past apogee
        for (uint8_t i = 3; i < 5; i++) {                                                         //Fire Pyros 3, 4
          pyros.arm(i);
          pyros.fire(i);
        }
        bpFired1 = true;
      }
      if (FCtime - apogeeTime > T_BP_DEPLOY2 && !bpFired2) {                                      //If we are more than some amount past apogee
        pyros.arm(5);
        pyros.fire(5);
        bpFired2 = true;
      }
      if (FCtime - apogeeTime > T_MAIN_LOCKOUT) {                                               //If we are past main lockout time
        if (recState == MAIN ||                                                                   //If we recieve signal to enter main state
        (gps.getFixType() == 3 && gps.getHeight() < MAIN_DEPLOY_ALT) ||                           //Or If we have 3D GPS fix and are less than some amount AGL
        (barometer.getFilteredAltitude() < MAIN_DEPLOY_ALT)) {                                    //Or If the barometer says we are less than some amount AGL
          currentState = MAIN;                                                                    //Advance to main state
          pyros.arm(2);                                                                           //Fire Pyro 2
          pyros.fire(2);                                                                        
          disableRollControlServos();                                                             //Remove power and signal from roll control servos to prevent damage upon landing
        }
      }
      break;
    case MAIN:                                                                                //If we are in main state
      if (recState == END) {                                                                    //If we recieve signal to enter into end state
        currentState = GROUND_TESTING;                                                          //Go back to ground testing mode
        loggingEnabled = false;                                                                 //End logging
        //pwrCommand.BMS.protectionsEnabled = true;                                               //Enable BMS protections/screw switch
        //pwrCommand.BMS.screwSwitchEnabled = true;
      }
      break;
  }
}
///////////////////////////////////////////////////
//                      Flash                    //
///////////////////////////////////////////////////
uint8_t flashBuf[512];
uint8_t bufInd = 0;
uint32_t writeIndex = 0;
void allocateFlash() {
  for (uint32_t eraseAdr = 0; eraseAdr < 5000000; eraseAdr += 256000) {
    flashMem.sectorErase(eraseAdr);
    while (flashMem.isBusy()) {

    }
  }
}

void handleLogging() {
  if (loggingEnabled) {
    memcpy(flashBuf + bufInd * 128, telemPkt, 128);
    bufInd++;
    if (bufInd == 4) {
      flashMem.programPage(flashBuf, writeIndex);
      writeIndex += 512;
      bufInd = 0;
    }
  }

}
///////////////////////////////////////////////////
//                    Telemetry                  //
///////////////////////////////////////////////////
void constructTelemetryPacket() {
  packetNum++;
  //Pyros
  uint16_t pyrosStatus = pyros.getPyrosStatus();
  memcpy(&telemPkt[0], &pyrosStatus, 2);

  uint8_t armed_byte = 0;
  for(uint8_t i = 0; i < 6; i++) {
    armed_byte |= pyros.isArmed(i) << i;
  }
  memcpy(&telemPkt[2], &armed_byte, 1);

  uint8_t fired_byte = 0;
  for(uint8_t i = 0; i < 6; i++) {
    fired_byte |= pyros.isFired(i) << i;
  }
  memcpy(&telemPkt[3], &fired_byte, 1);

  for (uint8_t i = 0; i < 6; i++) {
    telemPkt[4 + i] = pyros.resistance(i) * 10;
  }

  //Servos
  telemPkt[10] = servo0us & 0xFF;
  telemPkt[11] = (servo0us >> 8) | ((servo1us & 0x0F) << 4);
  telemPkt[12] = (servo1us >> 4) & 0xFF;
  telemPkt[13] = servo2us & 0xFF;
  telemPkt[14] = (servo2us >> 8) | ((servo3us & 0x0F) << 4);
  telemPkt[15] = (servo3us >> 4) & 0xFF;

  //Accel
  int32_t accelRawX = accel.getRawX();
  int32_t accelRawY = accel.getRawY();
  int32_t accelRawZ = accel.getRawZ();
  memcpy(&telemPkt[16], &accelRawX, 3);
  memcpy(&telemPkt[19], &accelRawY, 3);
  memcpy(&telemPkt[22], &accelRawZ, 3);

  //Gyro
  uint16_t gyroRawX = mygyro.getRawX();
  uint16_t gyroRawY = mygyro.getRawY();
  uint16_t gyroRawZ = mygyro.getRawZ();
  memcpy(&telemPkt[25], &gyroRawX, 2);
  memcpy(&telemPkt[27], &gyroRawY, 2);
  memcpy(&telemPkt[29], &gyroRawZ, 2);

  //GPS
  uint8_t gpsFix = gps.getFixType();
  int32_t lat = gps.getLat();
  int32_t lon = gps.getLon();
  float alt = gps.getHeight();
  uint32_t hACC = gps.getHAcc();
  uint32_t vACC = gps.getVAcc();
  uint8_t numSat = gps.getNumSV();
  memcpy(&telemPkt[31], &gpsFix, 1);
  memcpy(&telemPkt[32], &lat, 4);
  memcpy(&telemPkt[36], &lon, 4);
  memcpy(&telemPkt[40], &alt, 4);
  memcpy(&telemPkt[44], &hACC, 4);
  memcpy(&telemPkt[48], &vACC, 4);
  memcpy(&telemPkt[52], &numSat, 1);

  //Baro
  uint32_t baroRawPress = barometer.getRawPress();
  uint32_t baroRawTemp = barometer.getRawTemp();
  float baroAltAvg = barometer.getFilteredAltitude();
  memcpy(&telemPkt[53], &baroRawPress, 3);
  memcpy(&telemPkt[56], &baroRawTemp, 3);
  memcpy(&telemPkt[59], &baroAltAvg, 4);

  //State
  telemPkt[63] = currentState;

  //Integrated Angles
  float roll = mygyro.getRoll();
  float pitch = mygyro.getPitch();
  float yaw = mygyro.getYaw();
  memcpy(&telemPkt[64], &roll, 4);
  memcpy(&telemPkt[68], &pitch, 4);
  memcpy(&telemPkt[72], &yaw, 4);

  //Max Altitudes
  uint16_t maxBaroAlt = barometer.getMaxAlt();
  uint16_t maxGPSAlt = gps.getMaxAlt();
  memcpy(&telemPkt[76], &maxBaroAlt, 2);
  memcpy(&telemPkt[78], &maxGPSAlt, 2);

  //Time
  memcpy(&telemPkt[80], &FCtime, 4);

  //Packet Number
  memcpy(&telemPkt[84], &packetNum, 2);

  //RX RSSI
  memcpy(&telemPkt[86], &rxrssi, 1);

  //Power
  uint16_t cell1 = pwr.getCell1Voltage();
  uint16_t cell2 = pwr.getCell2Voltage();
  uint16_t cell3 = pwr.getCell3Voltage();
  uint16_t totalCurrent = pwr.getTotalCurrent();
  memcpy(&telemPkt[87], &cell1, 2);
  memcpy(&telemPkt[89], &cell2, 2);
  memcpy(&telemPkt[91], &cell3, 2);
  memcpy(&telemPkt[93], &totalCurrent, 2);
  telemPkt[95] = pwr.getTemp() * 2.0;
  telemPkt[96] = pwr.getProtectionStatus();
  telemPkt[97] = pwr.getProtectionsEnabled();
  for (uint8_t i = 0; i < 6; i++) {
    uint16_t converterVoltage = pwr.getConverterVoltage(i);
    uint16_t converterCurrent = pwr.getConverterCurrent(i);
    memcpy(&telemPkt[98 + 2 * i], &converterVoltage, 2);
    memcpy(&telemPkt[110 + 2 * i], &converterCurrent, 2);
  }

  //Accel Integrated Velocity
  float accelIntegratedVelocity = accel.getIntegratedVelo();
  memcpy(&telemPkt[122], &accelIntegratedVelocity, 4);

  //Checksum
  telemPkt[127] = calcChecksum();
}

uint8_t calcChecksum() {
  uint8_t sum = 0;
  for(int i = 0; i < 127; i++) {
    sum += telemPkt[i];
  }
  return sum;
}

uint8_t calcChecksumP(uint8_t* p, uint8_t len) {
  uint8_t sum = 0;
  for(int i = 0; i < len; i++) {
    sum += *(p + i);
  }
  return sum;
}

void sendTelemetryPacket() {
  if (cc.status() == 7) {
    cc.flushTx();
  }
  cc.freq915();
  cc.Tx(telemPkt, 128);
}

void readTelem() {
  if(cc.status() == 0) {
    cc.freq920();
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
            if (recValid) {
              for(uint8_t i = 0; i < 8; i++) {
                if ((recBuf[12] >> i) & 0b1) {
                  pyros.arm(i);
                }
              }
            }
            break;
          case 0x02:
            for (uint8_t i = 1; i < 12; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid) {
              recState = static_cast<State>(recBuf[12]);
            }
            break;
          case 0x06:
            for (uint8_t i = 1; i < 12; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid) {
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
              airbrakesSetAngle = *((float*) &recBuf[9]);
              airbrakesEnabled = false;
            }
            break;
          case 0x05:
            for (uint8_t i = 1; i < 9; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
              rollControlSetAngle = *((float*) &recBuf[9]);
              rollControlEnabled = false;
            }
            break;
          case 0x04:
            for (uint8_t i = 1; i < 13; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
              updateRollControl();
              rollControlEnabled = true;
            }
            break;
          case 0x08:
            for (uint8_t i = 1; i < 13; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
              airbrakesSetAngle = AIRBRAKES_CLOSED_ANGLE;
              rollControlSetAngle = 0;
              airbrakesEnabled = false;
              rollControlEnabled = false;
            }
            break;
          case 0x09:
            for (uint8_t i = 1; i < 13; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
                mygyro.zeroRollPitchYaw();
            }
            break;
          case 0x0A:
            for (uint8_t i = 1; i < 13; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
                barometer.zeroAlt();
                gps.zeroAlt();
            }
            break;
          case 0x0B:
            for (uint8_t i = 1; i < 13; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
              accel.zeroIntegratedVelo();
            }
            break;
          case 0x10: //EMERGENCY PISTON
            for (uint8_t i = 1; i < 13; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid) {
              for (uint8_t i = 0; i < 2; i++) {                                                       //Fire Pyros 0, 1
                pyros.arm(i);
                pyros.fire(i);
              }
            }
            break;
          case 0x11: //EMERGENCY BP
            for (uint8_t i = 1; i < 13; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid) {
              for (uint8_t i = 3; i < 5; i++) {                                                         //Fire Pyros 3, 4
                pyros.arm(i);
                pyros.fire(i);
              }
            }
            break;
          case 0x12: //EMERGENCY TD
            for (uint8_t i = 1; i < 13; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid) {
              pyros.arm(2);
              pyros.fire(2);
            }
            break;
          case 0x13: //EMERGENCY ALL
            for (uint8_t i = 1; i < 13; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid) {
              for (uint8_t i = 0; i < 5; i++) {                                                       //Fire Pyros 0, 1
                pyros.arm(i);
                pyros.fire(i);
              }
            }
            break;
          case 0x14: //VTX PWR
            for (uint8_t i = 1; i < 12; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
              if (recBuf[12] > 3) {
                recBuf[12] = 3;
              }
              myVTX.setPower(recBuf[12]);
            }
            break;
          case 0x15: //Converters
            for (uint8_t i = 1; i < 12; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
              debugSer.println(recBuf[12], BIN);
              for (uint8_t i = 0; i < 6; i++) {
                pwrCommand.convertersEnabled[i] = (recBuf[12] >> i) & 0x01;
                /*if ((recBuf[12]>>i) & 0x01) {
                  debugSer.println(i);
                }*/
              }
            }
            break;
          case 0x16: //BMS PROTECTIONS
            for (uint8_t i = 1; i < 12; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
              pwrCommand.BMS.protectionsEnabled = recBuf[12];
              pwrCommand.BMS.screwSwitchEnabled = recBuf[12];
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

///////////////////////////////////////////////////
//                    Airbrakes                  //
///////////////////////////////////////////////////
/*
typedef struct {
  float altitude;
  float vel_z;
  float accel_z;
  bool apogeeReached;
} AirbrakesData;
*/

AirbrakesData sendToAirbrakes;
void updateAirbrakes() {
  sendToAirbrakes.altitude = barometer.getFilteredAltitude();
  sendToAirbrakes.vel_z = accel.getIntegratedVelo();
  sendToAirbrakes.accel_z = accel.getAccelZ();
  sendToAirbrakes.apogeeReached = currentState > FLIGHT;
  myairbrakes.update((FCtime - flightBeginTime)/1000, sendToAirbrakes);
}

void updateRollControl() {
  myrollcontrol.update((FCtime - flightBeginTime) / 1000.0, barometer.getFilteredAltitude(), 
                          accel.getIntegratedVelo(), mygyro.getRoll(), mygyro.getRollRate());
}

void disableRollControlServos() {
  myTim->setMode(3, TIMER_OUTPUT_COMPARE_FORCED_INACTIVE, PD14);
  myTim->setMode(4, TIMER_OUTPUT_COMPARE_FORCED_INACTIVE, PD15);
  pwrCommand.convertersEnabled[4] = false;
}

uint16_t degToUsAirbrakes(float degrees) {
  return 1500.0 + (degrees / 60.0) * 500.0; 
}

uint16_t degToUsRollControl(float degrees) {
  return 1500.0 + (degrees / 50.0) * 500.0;
}

float dpToDeg(float dp) {
  return (AIRBRAKES_OPEN_ANGLE - AIRBRAKES_CLOSED_ANGLE) * dp + AIRBRAKES_CLOSED_ANGLE;
}

void Update_IT_callback() {
  if (airbrakesEnabled) {
    myTim->setCaptureCompare(1, degToUsAirbrakes(dpToDeg(myairbrakes.getDeployment())), MICROSEC_COMPARE_FORMAT);
    servo0us = degToUsAirbrakes(dpToDeg(myairbrakes.getDeployment()));
  } else {
    myTim->setCaptureCompare(1, degToUsAirbrakes(airbrakesSetAngle), MICROSEC_COMPARE_FORMAT);
    servo0us = degToUsAirbrakes(airbrakesSetAngle);
  }
  if (rollControlEnabled) {
    myTim->setCaptureCompare(3, degToUsRollControl(myrollcontrol.getAngle() + SERVO2OFFSET), MICROSEC_COMPARE_FORMAT);
    myTim->setCaptureCompare(4, degToUsRollControl(myrollcontrol.getAngle() + SERVO3OFFSET), MICROSEC_COMPARE_FORMAT);
    servo2us = degToUsRollControl(myrollcontrol.getAngle() + SERVO2OFFSET);
    servo3us = degToUsRollControl(myrollcontrol.getAngle() + SERVO3OFFSET);
  } else {
    myTim->setCaptureCompare(3, degToUsRollControl(rollControlSetAngle + SERVO2OFFSET), MICROSEC_COMPARE_FORMAT);
    myTim->setCaptureCompare(4, degToUsRollControl(rollControlSetAngle + SERVO3OFFSET), MICROSEC_COMPARE_FORMAT);
    servo2us = degToUsRollControl(rollControlSetAngle + SERVO2OFFSET);
    servo3us = degToUsRollControl(rollControlSetAngle + SERVO3OFFSET);
  }
}