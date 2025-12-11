#include <SPI.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <IAM20380.h>
#define DEBUG_SER Serial2
#define MAIN_SER Serial1
#define MOSI PA7
#define MISO PA6
#define SCLK PA5
#define MAG_CS PA0
#define GYRO_CS PA1
//MAG CALIBRATION
#define MAG_XOFFSET 133039.0
#define MAG_YOFFSET 132740.0
#define MAG_ZOFFSET 130707.0
#define XSCALE 1.00
#define YSCALE 1.03192
#define ZSCALE 1.02375
//GYRO CALIBRATION
#define GYRO_XOFFSET -26
#define GYRO_YOFFSET -14
#define GYRO_ZOFFSET -16

#define OFFSET1 7 //CHANNEL 1 (PA8, HEADER 7, RED WIRE)
#define OFFSET2 -2 //CHANNEL 2 (PA9, HEADER 6, ALL BLACK WIRES)

#define setpoint 0.0

#define kP 0.1887
#define kD 0.03774

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

fin2 rxpkt;

uint8_t rxData [sizeof(rxpkt) * 2 - 1];

struct downlink {
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
downlink pkt;
SPIClass SPI_3(MOSI, MISO, SCLK, -1);
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);
HardwareSerial DEBUG_SER(PA3, PA2);
HardwareSerial MAIN_SER(PB7, PB6);
SFE_MMC5983MA myMag;
IAM20380 gyro(&SPI_3, settings, GYRO_CS);
unsigned long last = millis();
unsigned long last2 = millis();
unsigned long gyro_last;

HardwareTimer *myTim = new HardwareTimer(TIM1);

void setup() {
  // put your setup code here, to run once:
  SPI_3.begin();
  SPI_3.beginTransaction(settings);
  pinMode(MAG_CS, OUTPUT);
  digitalWrite(MAG_CS, 1);
  pinMode(GYRO_CS, OUTPUT);
  digitalWrite(GYRO_CS, 1);
  DEBUG_SER.begin(115200);
  MAIN_SER.begin(500000);
  myMag.begin(MAG_CS);
  myMag.softReset();
  myMag.setFilterBandwidth(400);
  gyro.config();
  myTim->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PA8);
  myTim->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PA9);
  myTim->setOverflow(20000, MICROSEC_FORMAT);
  myTim->setCaptureCompare(1, degToUs(OFFSET1), MICROSEC_COMPARE_FORMAT);
  myTim->setCaptureCompare(2, degToUs(OFFSET2), MICROSEC_COMPARE_FORMAT);
  myTim->attachInterrupt(Update_IT_callback);
  pkt.us_on_6 = degToUs(OFFSET1);
  pkt.us_on_7 = degToUs(OFFSET2);
  myTim->resume();
}
void loop() {
  // put your main code here, to run repeatedly:
  /*if(millis() - last > 1000) {
    DEBUG_SER.println("---- Parsed Data ----");
    DEBUG_SER.print("rawMagX: "); DEBUG_SER.println(pkt.rawMagX);
    DEBUG_SER.print("rawMagY: "); DEBUG_SER.println(pkt.rawMagY);
    DEBUG_SER.print("rawMagZ: "); DEBUG_SER.println(pkt.rawMagZ);
    DEBUG_SER.print("heading: "); DEBUG_SER.println(pkt.heading, 2);
    DEBUG_SER.print("rawGyroX: "); DEBUG_SER.println(pkt.rawGyroX);
    DEBUG_SER.print("rawGyroY: "); DEBUG_SER.println(pkt.rawGyroY);
    DEBUG_SER.print("rawGyroZ: "); DEBUG_SER.println(pkt.rawGyroZ);
    DEBUG_SER.print("dpsX: "); DEBUG_SER.println(pkt.dpsX, 2);
    DEBUG_SER.print("dpsY: "); DEBUG_SER.println(pkt.dpsY, 2);
    DEBUG_SER.print("dpsZ: "); DEBUG_SER.println(pkt.dpsZ, 2);
    DEBUG_SER.print("yaw: "); DEBUG_SER.println(pkt.yaw, 2);
    DEBUG_SER.print("pitch: "); DEBUG_SER.println(pkt.pitch, 2);
    DEBUG_SER.print("roll: "); DEBUG_SER.println(pkt.roll, 2);
    last = millis();
  }*/
  magupdate();
  gyroupdate();
  
  sendPkt();
  
  updatePacket();
  parsePacket();

  servoHandle();

  if (rxpkt.zeroRoll) {
    pkt.yaw = 0.0;
    pkt.pitch = 0.0;
    pkt.roll = 0.0;
    rxpkt.zeroRoll = false;
  }

  if (millis() - last2 > 4) {
    //DEBUG_SER.println("SLOW");
  }
  while (millis() - last2 < 4) {}//4
  last2 = millis();
}

void updatePacket() {
  uint16_t avail = MAIN_SER.available();
  while (avail > sizeof(rxData)) {
    MAIN_SER.read();
    avail--;
  }
  if (avail) {
    //Shift existing data to the left
    for(uint8_t i = avail; i < sizeof(rxData); i++) {
      rxData[i - avail] = rxData[i];
    }
    //Read new data into the end
    for (uint8_t i = avail; i > 0; i--) {
      rxData[sizeof(rxData) - i] = MAIN_SER.read();
    }
  }
}

void parsePacket() {
  for (uint8_t i = 0; i < sizeof(rxpkt) + 1; i++) {
    if (rxData[i] == 0x61 && rxData[i + 1] == 0x61 && rxData[i + 2] == 0x61 && rxData[i + 3] == 0x61) {
        uint8_t sum = 0;
        for (uint8_t x = 0; x < sizeof(rxpkt); x++) {
          sum += rxData[i + 4 + x];
        }
        if (sum == rxData[i + 4 + sizeof(rxpkt)]) {
          memcpy(&rxpkt, &(rxData[i + 4]), sizeof(rxpkt));
        }
    }
  }
}

void servoHandle() {
  switch (rxpkt.servoState) {
    case SERVO_ZERO:
      setServos(0.0);
      break;
    case SERVO_ANGLE:
      setServos(rxpkt.angle);
      break;
  }
}

void Update_IT_callback() {
  if (rxpkt.servoState == SERVO_PD) {
    pidUpdate();
  }
}

void setServos(float degrees) {
  myTim->setCaptureCompare(1, degToUs(degrees + OFFSET1), MICROSEC_COMPARE_FORMAT);
  myTim->setCaptureCompare(2, degToUs(degrees + OFFSET2), MICROSEC_COMPARE_FORMAT);
  pkt.us_on_6 = degToUs(degrees + OFFSET1);
  pkt.us_on_7 = degToUs(degrees + OFFSET2);
}

uint16_t degToUs(float degrees) {
  return 1500.0 + (degrees / 60.0) * 500.0; 
}

void sendPkt() {
  MAIN_SER.print("aaaa");
  MAIN_SER.write((uint8_t*) &pkt, sizeof(pkt));
  MAIN_SER.write(calcChecksum());
}

uint8_t calcChecksum() {
  uint8_t sum;
  for(int i = 0; i < sizeof(pkt); i++) {
    sum += *((uint8_t*) &pkt + i);
  }
  return sum;
}
void gyroupdate() {
  pkt.rawGyroX = gyro.gyro_x() - GYRO_XOFFSET;
  pkt.rawGyroY = gyro.gyro_y() - GYRO_YOFFSET;
  pkt.rawGyroZ = gyro.gyro_z() - GYRO_ZOFFSET;
  pkt.dpsX = pkt.rawGyroX * 0.03051757812;
  pkt.dpsY = pkt.rawGyroY * -0.03051757812; //So roll is +CW like magnetometer
  pkt.dpsZ = pkt.rawGyroZ * 0.03051757812;
  unsigned long new_time = micros();
  if (gyro_last != 0) {
    float elapsed = (new_time - gyro_last);
    pkt.yaw += ((float) pkt.dpsX) * elapsed / 1000000.0;
    pkt.pitch += ((float) pkt.dpsZ) * elapsed / 1000000.0;
    pkt.roll += ((float) pkt.dpsY) * elapsed / 1000000.0;
  }
  gyro_last = new_time;
}
void magupdate() {
  uint32_t rawValueX = 0;
  uint32_t rawValueY = 0;
  uint32_t rawValueZ = 0;
  double scaledX = 0;
  double scaledY = 0;
  double scaledZ = 0;
  double heading = 0;
  float finalHeading = 0;
  myMag.getMeasurementXYZ(&rawValueX, &rawValueY, &rawValueZ);
  pkt.rawMagX = rawValueX - MAG_XOFFSET;
  pkt.rawMagY = rawValueY - MAG_YOFFSET;
  pkt.rawMagZ = rawValueZ - MAG_ZOFFSET;
  scaledX = (double)rawValueX - MAG_XOFFSET;
  scaledX /= (131072.0 * XSCALE);
  scaledY = (double)rawValueY - MAG_YOFFSET;
  scaledY /= (131072.0 * YSCALE);
  scaledZ = (double)rawValueZ - MAG_ZOFFSET;
  scaledZ /= (131072.0 * ZSCALE);
  heading = atan2(scaledX, 0 - scaledZ);
  heading /= PI;
  heading *= 180;
  heading += 180;
  pkt.heading = heading;
}

void pidUpdate() {
  /*
  kP, kD, velocity (m/s), setpoint (deg), roll (deg), dpsY (deg/s) are global vars (type float) that get updated elsewhere.
  Note we use dpsY because the y axis on the gyro is alligned with the roll axis of rocket.
  */
  float velocity_pid = (rxpkt.velo < 20.0) ? 20 : rxpkt.velo;
  //SAFEGUARD: Protect against absurdly high velocitites
  float velocity_pid = (rxpkt.velo > 300.0) ? 300 : rxpkt.velo;
  float scaled_kP = kP * 10000 / pow(velocity_pid, 2);
  float scaled_kD = kD * 10000 / pow(velocity_pid, 2);
  /*
  Roll is positive CW, and a positive angle on the sevos rolls the rocket CW.
  Therefore, we don't need to negate PD signs.
  PD control output = kP * e(t) + kD * de/dt
  e(t) = setpoint - measured = setpoint - roll
  de/dt = d(setpoint - roll)/dt = -dpsY
  As a sanity check, rocket at roll = 10 deg and zero angular velo,
  then pdOutput will be negative, so servos will drive CCW, causing rocket to roll CCW.
  */
  //FIX: DO NOT SCALE scaled_kD by PI/180
  float pdOutput = scaled_kP * (setpoint - pkt.roll) + scaled_kD * -pkt.dpsY;

  //Clip to -10 and 10 deg
  if (pdOutput > 10) {
    pdOutput = 10.0;
  } else if (pdOutput < -10) {
    pdOutput = -10.0;
  }
  setServos(pdOutput);
}
