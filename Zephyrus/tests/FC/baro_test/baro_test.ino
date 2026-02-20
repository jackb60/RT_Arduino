HardwareSerial debugSer(PC7, PC6);

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

#include "SPI.h"
#include "ADXL357.h"

#define MOSI PD7
#define MISO PB4
#define SCLK PA5
#define BARO_CS PD3

SPIClass SPI_3(MOSI, MISO, SCLK, -1);
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

void setup() {
  // put your setup code here, to run once:
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
  pinMode(PD3, OUTPUT);
  SPI_3.begin();
  debugSer.begin(115200);
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  debugSer.println(rawTemp());
  debugSer.println(rawPress());
  debugSer.println(getTemperature());
  debugSer.println(getAltitude());
  debugSer.println();

  delay(1000);
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

float getPressure(){
  dT = (float)rawTemp() - ((float)C5)*((int)1<<8);
  TEMP = 2000.0 + dT * ((float)C6)/(float)((long)1<<23);
  OFF = (((int64_t)C2)*((long)1<<17)) + dT * ((float)C4)/((int)1<<6);
  SENS = ((float)C1)*((long)1<<16) + dT * ((float)C3)/((int)1<<7);
  float pa = (float)((float)rawPress()/((long)1<<15));
  float pb = (float)(SENS/((float)((long)1<<21)));
  float pc = pa*pb;
  float pd = (float)(OFF/((float)((long)1<<15)));
  P = pc - pd;
  return P/100;
}
float getTemperature(){
  dT = (float)rawTemp() - ((float)C5)*((int)1<<8);
  TEMP = 2000.0 + dT * ((float)C6)/(float)((long)1<<23);
  return TEMP/100 ;
}
float getAltitude(){
  float h,t,p;
  t = getTemperature();
  p = getPressure();
  p = P0/p;
  h = 153.84615*(pow(p,0.19) - 1)*(t+273.15);
  return h;
}