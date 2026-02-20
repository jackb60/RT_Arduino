#include <SPI.h>
#include <SoftwareSerial.h>
#include <CC1200.h>

#define MAVG 20
#define MAVG2 50

#define ACCELXOFFSET 0
#define ACCELYOFFSET 0
#define ACCELZOFFSET 0

#define GPS_SER Serial1
//#define FIN_SER FIN_SER
#define MOSI PD7
#define MISO PB4
#define SCLK PA5
#define FLASH_CS PB12
#define ACCEL_CS PB13
#define BARO_CS  PD3
#define CC_CS PC12
//BAROMETER CALIBRATION VALUES
#define C1 0xA579
#define C2 0x953A
#define C3 0x68AC
#define C4 0x6305
#define C5 0x8405
#define C6 0x6D91
//BAROMETER VALUES
float dT, TEMP, P;
int64_t OFF, SENS;
const float P0 = 1013.25;
HardwareSerial GPS_SER(PB12, PB13);
//HardwareSerial FIN_SER(PA3, PA2);
//HardwareSerial BLE_SER(PB11, PB10);
//SoftwareSerial mySerial(PB9, PB8);
SPIClass SPI_3(MOSI, MISO, SCLK, -1);
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);
SPISettings accelSettings(1000000, MSBFIRST, SPI_MODE3);
unsigned long last = millis();
unsigned long last2 = millis();
unsigned long last3 = millis();
String nmeaData = "";
float BARO_mavg[MAVG];

float vel_mavg[MAVG2];
unsigned long vel_times[MAVG2];

unsigned long fire_times[8];
bool fired[8];

bool armed[8];

uint8_t GPS_CMD[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5};

uint8_t GPS_CMD2[] = {0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22};

uint8_t telem_pkt[128];
int16_t pyroPins[8];// = {PA0, PA1, PA4, PA5, PA6, PA7, PB0, PB1};
int16_t firePins[8];// = {PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8};

volatile uint16_t pyroStatus;

float vbat;
bool prev;
uint32_t flightTime;
uint32_t flightBegin;

uint16_t packetNum;
uint8_t badPackets;

float maxGpsAlt;
float maxBaroAvgAlt;

float gpsAltOffset;
float baroAltOffset;

float integratedVelo;

float verticalAccelMinusGravity;

float rawGPS;
float rawBF;


unsigned long accelLast = millis();

CC1200 cc(&SPI_3, settings, CC_CS);

enum State {
  GROUND_TESTING,
  PRE_FLIGHT,
  FLIGHT,
  APOGEE,
  DISREEF,
  END
};

enum PyroStatus {
  PYRO_FAILURE,
  PYRO_UNCONNECTED,
  PYRO_CONNECTED,
  PYRO_SUCCESS
};

State currentState = GROUND_TESTING;

State recState = GROUND_TESTING;

struct downlink {
  //GPS
  bool gpsValid;
  float lat;
  float lon;
  float GPS_alt;
  float PDOP;
  float HDOP;
  float VDOP;
  //ACCEL
  int16_t rawAccelX;
  int16_t rawAccelY;
  int16_t rawAccelZ;
  float accelX; // m/s^2
  float accelY; // m/s^2
  float accelZ; // m/s^2
  //BARO
  uint32_t rawTemp;
  uint32_t rawPressure;
  float temp;
  float pressure;
  float BARO_alt;
  float BARO_alt_avg;
  float BARO_velo;
  uint16_t pyros;
};

struct fin {
  //MAG
  int32_t rawMagX;
  int32_t rawMagY;
  int32_t rawMagZ;
  float heading;
  //GYRO
  int16_t rawGyroX;
  int16_t rawGyroY;
  int16_t rawGyroZ;
  float dpsX;
  float dpsY;
  float dpsZ;
  float yaw;
  float pitch;
  float roll;
  //SERVO
  uint16_t us_on_6 = 1000;
  uint16_t us_on_7 = 1050;
};

enum Servo_State {
  SERVO_ZERO,
  SERVO_ANGLE,
  SERVO_PD
};

struct fin2 {
  Servo_State servoState;
  bool zeroRoll;
  float angle;
  float velo;
};

downlink pkt;
fin pkt2;
fin2 fintx;

int8_t rxrssi;
unsigned long lastRec;

unsigned long apogeeTime;

bool logging = false;

uint8_t finData [sizeof(fin) * 2 - 1];


float baroLast = 0.0;

void setup() {
  // put your setup code here, to run once:
  //BLE_SER.begin(115200);
  //mySerial.begin(9600);
  GPS_SER.begin(9600);
  //FIN_SER.begin(500000);
  /*pinMode(PB12, OUTPUT);
  digitalWrite(PB12, 1);
  pinMode(PB13, OUTPUT);
  digitalWrite(PB13, 1);
  pinMode(PB15, OUTPUT);
  digitalWrite(PB15, 1);
  pinMode(PB14, OUTPUT);
  digitalWrite(PB14, 1);
  pyroSetup();*/
  pinMode(BARO_CS, OUTPUT);
  digitalWrite(BARO_CS, 1);
  pinMode(PC12, OUTPUT);
  digitalWrite(PC12, 1);
  pinMode(PD0, 1);
  digitalWrite(PD0, 1);
  pinMode(PD1, OUTPUT);
  digitalWrite(PD1, 1);
  pinMode(PD2, OUTPUT);
  digitalWrite(PD2, 1);
  pinMode(PD4, OUTPUT);
  digitalWrite(PD4, 1);
  SPI_3.begin();
  SPI_3.beginTransaction(settings);
  cc.begin();
  cc.simpleConfig();
  SPI_3.endTransaction();
  //SPI_3.beginTransaction(accelSettings);
  //ACCEL_setup();
  //SPI_3.endTransaction();
  /*delay(1000);
  for (int i = 0; i < sizeof(GPS_CMD); i++) {
    GPS_SER.write(GPS_CMD[i]);
  }
  for (int i = 0; i < sizeof(GPS_CMD2); i++) {
    GPS_SER.write(GPS_CMD2[i]);
  }*/
  //GPS_SER.begin(115200);
}
void loop() {
  // put your main code here, to run repeatedly:
  GPS_updateString();
  GPS_parseWrap();
  //SPI_3.beginTransaction(accelSettings);
  //ACCEL_parse();
  //SPI_3.endTransaction();
  SPI_3.beginTransaction(settings);
  BARO_updateAll();

  //FIN_updateString();
  //FIN_parse();
  
  //pyroUpdate();
  //pyroMonitor();
  //updateBattery();

  flightTime = millis();

  //FINsendPkt();
  handleState();

  readTelem();
  if (millis() - last3 > 50) {
    last3 = millis();
    updateTelemPkt();
    sendTelemPkt();
    //handleLogging();
  }

  SPI_3.endTransaction();
  if(millis() - last2 > 10) {
    //mySerial.println("SLOW");
  }
  while (millis() - last2 < 10) {}
  last2 = millis();
}

///////////////////////////////////////////////////
//                  Logging                      //
///////////////////////////////////////////////////
bool flashEnabled = false;
uint32_t logAddress = 0;

bool isFlashBusy() {
  digitalWrite(FLASH_CS, LOW);
  SPI_3.transfer(0x05);
  uint8_t status = SPI_3.transfer(0x00);
  digitalWrite(FLASH_CS, HIGH);
  return (status & 0x01);
}

void writeEnable() {
  digitalWrite(FLASH_CS, 0);
  SPI_3.transfer(0x06);
  digitalWrite(FLASH_CS, 1);
}

void eraseFlash() {
  digitalWrite(FLASH_CS, 0);
  SPI_3.transfer(0xC7);
  digitalWrite(FLASH_CS, 1);
}

void handleLogging() {
  if (logging) {
    if (!flashEnabled) {
      writeEnable();
      eraseFlash();
      while (isFlashBusy()) {}
      flashEnabled = true;
    }
  writeEnable();
  digitalWrite(FLASH_CS, 0);
  SPI_3.transfer(0x02);
  SPI_3.transfer(logAddress >> 16);
  SPI_3.transfer(logAddress >> 8);
  SPI_3.transfer(logAddress);
  byte tp_cpy[128];
  memcpy(tp_cpy, telem_pkt, 128);
  SPI_3.transfer(tp_cpy, 128);
  digitalWrite(FLASH_CS, 1);
  logAddress += 128;
  if (logAddress == 8388608) {
    logging = false;
  }
  }
}


///////////////////////////////////////////////////
//                    Pyros                      //
///////////////////////////////////////////////////
void armPyro(uint8_t i) {
  armed[i] = true;
}

void disarmPyro(uint8_t i) {
  armed[i] = false;
}

void firePyro(uint8_t i) {
  if(armed[i]) {
    digitalWrite(firePins[i], 1);
    fired[i] = true;
    armed[i] = false;
    fire_times[i] = millis();
  }
}

void pyroMonitor() {
  for(int i = 0; i < 8; i++) {
    if (fired[i] && (millis() - fire_times[i]) > 1000) {
      digitalWrite(firePins[i], 0);
      if(!isPyroConnected(i) &&  getPyroStatus(i) != PYRO_FAILURE) {
        setPyroStatus(i, PYRO_SUCCESS);
      } else {
        setPyroStatus(i, PYRO_FAILURE);
      }
      fired[i] = false;
    }
  }
}

void pyroSetup() {
  for(byte i = 0; i < 8; i++) {
    pinMode(firePins[i], OUTPUT);
    digitalWrite(firePins[i], 0);
  }
}

void setPyroStatus(uint8_t pyro, PyroStatus val) {
  uint16_t mask = 0b11 << (pyro * 2);
  pyroStatus &= ~mask;
  pyroStatus |= (val & 0b11) << (pyro * 2);
}

uint8_t getPyroStatus(uint8_t pyro) {
  return (pyroStatus >> (pyro * 2)) & 0b11;
}

bool isPyroConnected(uint8_t pyro) {
  uint16_t analogVal = analogRead(pyroPins[pyro]);
  return analogVal * 3.3 / 1023.0 > 0.25;
}

void pyroUpdate() {
  for (uint8_t i = 0; i < 8; i++) {
    if (currentState == GROUND_TESTING) {
      //In ground testing, we only care if pyro is connected or disconnected
      if(isPyroConnected(i)) {
        setPyroStatus(i, PYRO_CONNECTED);
      } else {
        setPyroStatus(i, PYRO_UNCONNECTED);
      }
    } else {
      //Otherwise, an unconnected pyro is a failure, unless it has been successfuly fired
      if(!isPyroConnected(i) && getPyroStatus(i) != PYRO_SUCCESS && !fired[i]) {
        setPyroStatus(i, PYRO_FAILURE);
      }
    }
  }
  pkt.pyros = pyroStatus;
}
///////////////////////////////////////////////////
//            Telem Packet Contruction           //
///////////////////////////////////////////////////
//telem_pkt
void readTelem() {
  if(cc.status() == 0 && prev) {
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
                  armPyro(i);
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
                  firePyro(i);
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
          case 0x03:
            for (uint8_t i = 1; i < 9; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
              fintx.angle = *((float*) &recBuf[9]);
              fintx.servoState = SERVO_ANGLE;
            }
            break;
          case 0x04:
            for (uint8_t i = 1; i < 13; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
              fintx.servoState = SERVO_PD;
              fintx.zeroRoll = true;
            }
            break;
          case 0x07:
            for (uint8_t i = 1; i < 12; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
              for(uint8_t i = 0; i < 8; i++) {
                if ((recBuf[12] >> i) & 0b1) {
                  disarmPyro(i);
                }
              }
            }
            break;
          case 0x08:
            for (uint8_t i = 1; i < 13; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
            }
            if (recValid && currentState == GROUND_TESTING) {
              fintx.servoState = SERVO_ZERO;
            }
            break;
          case 0x09:
            for (uint8_t i = 1; i < 13; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
              if (recValid && currentState == GROUND_TESTING) {
                zeroRoll();
              }
            }
            break;
          case 0x0A:
            for (uint8_t i = 1; i < 13; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
              if (recValid && currentState == GROUND_TESTING) {
                zeroAlt();
              }
            }
            break;
          case 0x0B:
            for (uint8_t i = 1; i < 13; i++) {
              if (recBuf[i] != 0x00) {
                recValid = false;
              }
              if (recValid && currentState == GROUND_TESTING) {
                zeroVelo();
              }
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

void sendTelemPkt() {
  cc.Tx(telem_pkt, 128);
  prev = true;
}

void updateTelemPkt() {
  packetNum++;

  memcpy(&telem_pkt[0], &pkt.pyros, 2);

  telem_pkt[11] = pkt2.us_on_6;
  telem_pkt[12] = pkt2.us_on_6 >> 8;
  telem_pkt[12] += pkt2.us_on_7 << 4;
  telem_pkt[13] = pkt2.us_on_7 >> 4;

  memcpy(&telem_pkt[14], &pkt.rawAccelX, 2);
  memcpy(&telem_pkt[16], &pkt.rawAccelY, 2);
  memcpy(&telem_pkt[18], &pkt.rawAccelZ, 2);

  memcpy(&telem_pkt[20], &pkt.rawPressure, 3);
  memcpy(&telem_pkt[23], &pkt.rawTemp, 3);

  memcpy(&telem_pkt[26], &pkt2.rawMagX, 3);
  memcpy(&telem_pkt[29], &pkt2.rawMagY, 3);
  memcpy(&telem_pkt[32], &pkt2.rawMagZ, 3);

  memcpy(&telem_pkt[35], &pkt2.rawGyroX, 2);
  memcpy(&telem_pkt[37], &pkt2.rawGyroY, 2);
  memcpy(&telem_pkt[39], &pkt2.rawGyroZ, 2);

  memcpy(&telem_pkt[41], &pkt.gpsValid, 1);
  memcpy(&telem_pkt[42], &pkt.lat, 4);
  memcpy(&telem_pkt[46], &pkt.lon, 4);
  memcpy(&telem_pkt[50], &pkt.GPS_alt, 4);
  memcpy(&telem_pkt[54], &pkt.PDOP, 4);
  memcpy(&telem_pkt[58], &pkt.HDOP, 4);
  memcpy(&telem_pkt[62], &pkt.VDOP, 4);

  memcpy(&telem_pkt[66], &flightTime, 4);
  unsigned long lastRecNew = millis() - lastRec;
  memcpy(&telem_pkt[70], &lastRecNew, 4);

  memcpy(&telem_pkt[74], &pkt2.yaw, 4);
  memcpy(&telem_pkt[78], &pkt2.pitch, 4);
  memcpy(&telem_pkt[82], &pkt2.roll, 4);
  memcpy(&telem_pkt[86], &pkt2.heading, 4);

  memcpy(&telem_pkt[90], &vbat, 4);

  memcpy(&telem_pkt[94], &currentState, 3);

  memcpy(&telem_pkt[95], &pkt.BARO_alt_avg, 4);
  memcpy(&telem_pkt[99], &pkt.BARO_velo, 4);

  uint8_t armed_byte = 0;
  for(uint8_t i = 0; i < 8; i++) {
    armed_byte |= armed[i] << i;
  }
  memcpy(&telem_pkt[103], &armed_byte, 1);


  uint8_t fired_byte = 0;
  for(uint8_t i = 0; i < 8; i++) {
    fired_byte |= fired[i] << i;
  }
  memcpy(&telem_pkt[104], &fired_byte, 1);

  telem_pkt[105] = badPackets;
  telem_pkt[106] = rxrssi;

  memcpy(&telem_pkt[107], &integratedVelo, 4);
  memcpy(&telem_pkt[111], &maxBaroAvgAlt, 4);
  memcpy(&telem_pkt[115], &maxGpsAlt, 4);

  memcpy(&telem_pkt[125], &packetNum, 2);

  telem_pkt[127] = calcChecksum();
}

uint8_t calcChecksum() {
  uint8_t sum = 0;
  for(int i = 0; i < 127; i++) {
    sum += telem_pkt[i];
  }
  return sum;
}

///////////////////////////////////////////////////
//                  State Machine                //
///////////////////////////////////////////////////
/*
  enum State {
  GROUND_TESTING,
  PRE_FLIGHT,
  FLIGHT,
  APOGEE,
  DISREEF
};
*/

#define ACCEL_FLIGHT_THS   30    //m/s^2
#define T_APOGEE_LOCKOUT   7000  //msec
#define T_APOGEE_OVERRIDE  13000 //msec
#define T_DISREEF_LOCKOUT  2000 //msec (PAST T_APOGEE)
#define T_DISREEF_OVERRIDE 2000 //msec (PAST T_APOGEE)
#define APOGEE_DROP        20    //m
#define DISREEF_DROP       150   //m

void handleState() {
  switch(currentState) {
    case GROUND_TESTING:                                                                            //If we are in ground testing mode
      if (recState == PRE_FLIGHT) {                                                                 //If we recieve signal to advance to preflight mode
        currentState = PRE_FLIGHT;                                                                  //Advance to preflight mode
        zeroRoll();                                                                                 //Zero roll, altitude, and velocity
        zeroAlt();
        zeroVelo();
        logging = true;                                                                             //Begin logging
        fintx.servoState = SERVO_ZERO;                                                              //Zero servos
      }
      break;
    case PRE_FLIGHT:                                                                                //If we are in preflight mode
      if ((recState == FLIGHT) || (verticalAccelMinusGravity > ACCEL_FLIGHT_THS)) {                 //If recieve signal to advance to flight mode, or accel - gravity > threshold
        currentState = FLIGHT;                                                                      //Advance to flight mode
        fintx.servoState = SERVO_PD;                                                                //Turn on PD control
        flightBegin = millis();                                                                     //Mark time flight mode entered
        flightTime = millis();
        zeroRoll();                                                                                 //Zero roll
      }
      break;
    case FLIGHT:                                                                                    //If we are in flight mode
      if (flightTime - flightBegin > T_APOGEE_LOCKOUT) {                                            //If we are past the apogee lockout time
        if (recState == APOGEE || (pkt.gpsValid && (maxGpsAlt - pkt.GPS_alt > APOGEE_DROP)) ||      //If we recieve signal to enter into apogee mode OR gps is valid and has alt has dropped APOGEE_DROP meters
        (maxBaroAvgAlt - pkt.BARO_alt_avg > APOGEE_DROP)) {                                         //OR barometer alt has dropped APOGEE_DROP meters
          currentState = APOGEE;                                                                    //Advance to Apogee mode
        }
      }
      if (flightTime - flightBegin > T_APOGEE_OVERRIDE) {                                           //If we are past the apogee override time
        currentState = APOGEE;                                                                      //Advance to Apogee mode
      }
      if (currentState == APOGEE) {                                                                 //If we advanced to apogee
        apogeeTime = millis();                                                                      //Mark time apogee mode entered
        flightTime = millis();
        for (uint8_t i = 0; i < 6; i++) {                                                           //Fire pyros at headers 0, 1, 2, 3, 4, 5
          armPyro(i);
          firePyro(i);
        }
        fintx.servoState = SERVO_ZERO;                                                              //Turn off fin control
      }
      break;
    case APOGEE:                                                                                    //If we are in apogee mode
      if (flightTime - apogeeTime > T_DISREEF_LOCKOUT) {                                            //If we are past the Disreef lockout time
        if (recState == DISREEF || (pkt.gpsValid && (maxGpsAlt - pkt.GPS_alt > DISREEF_DROP)) ||    //If we recieve signal to enter into disreef mode OR gps is valid and has alt has dropped DISREEF_DROP meters
        (maxBaroAvgAlt - pkt.BARO_alt_avg > DISREEF_DROP)) {                                        //OR barometer alt has dropped DISREEF_DROP meters
          currentState = DISREEF;                                                                   //Advance to Disreef mode
        }
      }
      if (flightTime - apogeeTime > T_DISREEF_OVERRIDE) {                                           //If we are past the Disreef override time
        currentState = DISREEF;                                                                     //Advance to Disreef mode
      }
      if (currentState == DISREEF) {                                                                //If we advanced to Disreef
        for (uint8_t i = 6; i < 8; i++) {                                                           //Fire pyros at headers 6, 7
          armPyro(i);
          firePyro(i);
        }
      }
      break;
    case DISREEF:
      if (recState == END) {
        logging = false;
        currentState = GROUND_TESTING;
      }
      break;
  }
}

///////////////////////////////////////////////////
//                 Calibration                   //
///////////////////////////////////////////////////

void zeroRoll() {
  fintx.zeroRoll = true;
}

void zeroAlt() {
  gpsAltOffset = rawGPS; //Note this only offsets if there is a fix, which is logical
  baroAltOffset = rawBF;
  maxGpsAlt = 0;
  maxBaroAvgAlt = 0;
}

void zeroVelo() {
  integratedVelo = 0;
}


///////////////////////////////////////////////////
//                Fin Control uC                 //
///////////////////////////////////////////////////
/*void FINsendPkt() {
  FIN_SER.print("aaaa");
  FIN_SER.write((uint8_t*) &fintx, sizeof(fintx));
  FIN_SER.write(FINcalcChecksum());
  if(fintx.zeroRoll) {
    fintx.zeroRoll = false;
  }
}

uint8_t FINcalcChecksum() {
  uint8_t sum;
  for(int i = 0; i < sizeof(fintx); i++) {
    sum += *((uint8_t*) &fintx + i);
  }
  return sum;
}

void FIN_updateString() {
  uint16_t avail = FIN_SER.available();
  while (avail > sizeof(finData)) {
    FIN_SER.read();
    avail--;
  }
  if (avail) {
    //Shift existing data to the left
    for(uint8_t i = avail; i < sizeof(finData); i++) {
      finData[i - avail] = finData[i];
    }
    //Read new data into the end
    for (uint8_t i = avail; i > 0; i--) {
      finData[sizeof(finData) - i] = FIN_SER.read();
    }
  }
}

void FIN_parse() {
  for (uint8_t i = 0; i < sizeof(fin) + 1; i++) {
    if (finData[i] == 0x61 && finData[i + 1] == 0x61 && finData[i + 2] == 0x61 && finData[i + 3] == 0x61) {
        uint8_t sum = 0;
        for (uint8_t x = 0; x < sizeof(fin); x++) {
          sum += finData[i + 4 + x];
        }
        if (sum == finData[i + 4 + sizeof(fin)]) {
          memcpy(&pkt2, &(finData[i + 4]), sizeof(fin));
        }
    }
  }
}*/
///////////////////////////////////////////////////
//                   BARO                        //
///////////////////////////////////////////////////
void BARO_updateAll() {
  pkt.rawTemp = rawTemp();
  pkt.rawPressure = rawPress();
  pkt.temp = getTemperature();
  pkt.pressure = getPressure();
  float rawBaro = getAltitude();
  pkt.BARO_alt = rawBaro - baroAltOffset;
  for(int8_t i = MAVG - 2; i > - 1; i--) {
    BARO_mavg[i+1] = BARO_mavg[i];
  }
  BARO_mavg[0] = rawBaro;
  pkt.BARO_alt_avg = 0;
  for(uint8_t i = 0; i < MAVG; i++) {
    pkt.BARO_alt_avg += BARO_mavg[i];
  }
  rawBF = pkt.BARO_alt_avg;
  pkt.BARO_alt_avg /= MAVG;
  pkt.BARO_alt_avg -= baroAltOffset;
  rawBF /= MAVG;

  if (pkt.BARO_alt_avg > maxBaroAvgAlt) {
    maxBaroAvgAlt = pkt.BARO_alt_avg;
  }

  for(uint8_t i = 1; i < MAVG2; i++) {
    vel_mavg[i - 1] = vel_mavg[i];
    vel_times[i - 1] = vel_times[i];
  }
  vel_mavg[MAVG2 - 1] = pkt.BARO_alt_avg;
  vel_times[MAVG2 - 1] = millis();
  pkt.BARO_velo = (vel_mavg[MAVG2 - 1] - vel_mavg[0]) / (vel_times[MAVG2 - 1] - vel_times[0]) * 1000.0;

}
float getPressure(){
  dT = (float)pkt.rawTemp - ((float)C5)*((int)1<<8);
  TEMP = 2000.0 + dT * ((float)C6)/(float)((long)1<<23);
  OFF = (((int64_t)C2)*((long)1<<17)) + dT * ((float)C4)/((int)1<<6);
  SENS = ((float)C1)*((long)1<<16) + dT * ((float)C3)/((int)1<<7);
  float pa = (float)((float)pkt.rawPressure/((long)1<<15));
  float pb = (float)(SENS/((float)((long)1<<21)));
  float pc = pa*pb;
  float pd = (float)(OFF/((float)((long)1<<15)));
  P = pc - pd;
  return P/100;
}
float getTemperature(){
  dT = (float)pkt.rawTemp - ((float)C5)*((int)1<<8);
  TEMP = 2000.0 + dT * ((float)C6)/(float)((long)1<<23);
  return TEMP/100 ;
}
float getAltitude(){
  float h,t,p;
  t = pkt.temp;
  p = pkt.pressure;
  p = P0/p;
  h = 153.84615*(pow(p,0.19) - 1)*(t+273.15);
  return h;
}
uint32_t rawTemp() {
  digitalWrite(BARO_CS, 0);
  SPI_3.transfer(0x54);
  digitalWrite(BARO_CS, 1);
  delay(3);
  digitalWrite(BARO_CS, 0);
  uint32_t res = 0;
  SPI_3.transfer(0x00);
  res += (SPI_3.transfer16(0x00)) << 8;
  res += SPI_3.transfer(0x00);
  digitalWrite(BARO_CS, 1);
  return res;
}
uint32_t rawPress() {
  digitalWrite(BARO_CS, 0);
  SPI_3.transfer(0x44);
  digitalWrite(BARO_CS, 1);
  delay(3);
  digitalWrite(BARO_CS, 0);
  uint32_t res = 0;
  SPI_3.transfer(0x00);
  res +=(SPI_3.transfer16(0x00)) << 8;
  res += SPI_3.transfer(0x00);
  digitalWrite(BARO_CS, 1);
  return res;
}
///////////////////////////////////////////////////
//                  BATTERY                      //
///////////////////////////////////////////////////
void updateBattery() {
  vbat = 4.0 * analogRead(PC0) * 3.3 * 1.77 / 1023.0;
}

///////////////////////////////////////////////////
//                   ACCEL                       //
///////////////////////////////////////////////////
void _writeReg(uint8_t reg, uint8_t value) {
  digitalWrite(ACCEL_CS, 0);
  SPI_3.transfer(reg);
  SPI_3.transfer(value);
  digitalWrite(ACCEL_CS, 1);
}

void _readData(int16_t* x, int16_t* y, int16_t* z) {
  digitalWrite(ACCEL_CS, 0);
  SPI_3.transfer(0x80 | 0x40 | 0x32); // Read multiple starting at DATAX0
  uint8_t data[6];
  SPI_3.transfer(data, 6);
  digitalWrite(ACCEL_CS, 1);
  *x = (int16_t) (data[1] << 8 | data[0]) - ACCELXOFFSET;
  *y = (int16_t) (data[3] << 8 | data[2]) - ACCELYOFFSET;
  *z = (int16_t) (data[5] << 8 | data[4]) - ACCELZOFFSET;
}

void ACCEL_setup() {
    _writeReg(0x31, 0b00001011); // Set data format to full res, +/-16g
    _writeReg(0x2D, 0b00001000); // Enable measurements
    //defaults to 100Hz
}


#define ACCEL_INT_THS 10
void ACCEL_parse() {
  _readData(&pkt.rawAccelX, &pkt.rawAccelY, &pkt.rawAccelZ);
  pkt.accelX = pkt.rawAccelX * 0.00390625 * 9.80665;
  pkt.accelY = pkt.rawAccelY * 0.00390625 * 9.80665;
  pkt.accelZ = pkt.rawAccelZ * 0.00390625 * 9.80665;

  verticalAccelMinusGravity = pkt.accelX - 9.81;
  //Only integrate if above threshold in pre-flight
  if (currentState == PRE_FLIGHT) {
    if (abs(verticalAccelMinusGravity) >= ACCEL_INT_THS) {
      integratedVelo += verticalAccelMinusGravity * (millis() - accelLast) / 1000.0;
    } 
  } else {
    integratedVelo += verticalAccelMinusGravity * (millis() - accelLast) / 1000.0;
  }
  fintx.velo = integratedVelo;
  //FIX FROM FLIGHTS 1/2: accelLast should be updated regardless of aceeleration value
  accelLast = millis();
}


///////////////////////////////////////////////////
//                    GPS                        //
///////////////////////////////////////////////////
void GPS_updateString() {
  while (GPS_SER.available()) {
    nmeaData += (char) GPS_SER.read();
  }
}
void GPS_parseWrap() {
  if (nmeaData.indexOf("$GNGLL") != -1) {
    int endOfBlock = nmeaData.lastIndexOf("$GNGLL");
    int endLine = nmeaData.indexOf('\n', endOfBlock);
    if (endLine != -1) {
      String toParse = nmeaData.substring(0, endLine + 1);
      parseNMEA(toParse);
      nmeaData.remove(0, endLine + 1);
    }
  }
}
void parseNMEA(String data) {
    /*pkt.gpsValid = false;
    pkt.lat = 0.0;
    pkt.lon = 0.0;
    pkt.GPS_alt = 0.0;
    pkt.PDOP = 0.0;
    pkt.HDOP = 0.0;
    pkt.VDOP = 0.0;*/
  // Split into lines
  int start = 0;
  while (start < data.length()) {
    int end = data.indexOf('\n', start);
    if (end == -1) end = data.length();
    String line = data.substring(start, end);
    start = end + 1;
    if(validateChecksum(line)) {
      if (line.startsWith("$GNRMC")) {
        int firstComma = line.indexOf(',');
        int statusIdx = nthIndexOf(line, ',', 2);
        if (statusIdx != -1) {
          char status = line.charAt(statusIdx + 1);
          pkt.gpsValid = (status == 'A');
        }
        String latStr = getField(line, 4);
        String latHem = getField(line, 5);
        String lonStr = getField(line, 6);
        String lonHem = getField(line, 7);
        pkt.lat = convertToDecimal(latStr.toFloat(), latHem);
        pkt.lon = convertToDecimal(lonStr.toFloat(), lonHem);
      }
      else if (line.startsWith("$GNGGA")) {
        float alt = getField(line, 10).toFloat();
        float geoid = getField(line, 12).toFloat();
        rawGPS = alt + geoid;
        pkt.GPS_alt = alt + geoid - gpsAltOffset;

        if (pkt.gpsValid && pkt.GPS_alt > maxGpsAlt) {
          maxGpsAlt = pkt.GPS_alt;
        }
      }
      else if (line.startsWith("$GNGSA")) {
        pkt.PDOP = getField(line, 16).toFloat();
        pkt.HDOP = getField(line, 17).toFloat();
        pkt.VDOP = getField(line, 18).toFloat();
      }
    }
  }
}
// --- Helper functions ---
String getField(String data, int index) {
  int start = 0, end = -1;
  for (int i = 0; i < index; i++) {
    start = end + 1;
    end = data.indexOf(',', start);
    if (end == -1) end = data.length();
  }
  return data.substring(start, end);
}
int nthIndexOf(String str, char ch, int n) {
  int pos = -1;
  while (n-- > 0) {
    pos = str.indexOf(ch, pos + 1);
    if (pos == -1) break;
  }
  return pos;
}
float convertToDecimal(float raw, String hemi) {
  int deg = int(raw / 100);
  float minutes = raw - (deg * 100);
  float dec = deg + (minutes / 60.0);
  if (hemi == "S" || hemi == "W") dec = -dec;
  return dec;
}

bool validateChecksum(String nmea) {
  // Find the asterisk that marks the start of checksum
  int asteriskIndex = nmea.indexOf('*');
  if (asteriskIndex == -1 || asteriskIndex < 1) return false;

  // Compute checksum from characters between '$' and '*'
  byte checksum = 0;
  for (int i = 1; i < asteriskIndex; i++) {
    checksum ^= nmea.charAt(i);
  }

  // Extract transmitted checksum (2 hex chars after '*')
  String checksumStr = nmea.substring(asteriskIndex + 1, asteriskIndex + 3);
  byte transmitted = strtol(checksumStr.c_str(), NULL, 16);

  return (checksum == transmitted);
}
