#include "SPI.h"
#include "CC1200.h"
#include "GPS.h"

#define MOSI PD7
#define MISO PB4
#define SCLK PA5
#define CS   PA2

SPIClass SPI_3(MOSI, MISO, SCLK, -1);
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

CC1200 cc(&SPI_3, settings, CS);

HardwareSerial gpsSer(PB12, PB13);
HardwareSerial debugSer(PC7, PC6);
byte recBuf[128];
byte txBuf[16];

byte state = 0;
byte pyros = 0;

GPS gps(&gpsSer);

void setup() {
  // put your setup code here, to run once:
  pinMode(PA0, OUTPUT);
  pinMode(PA1, OUTPUT);
  delay(1000);
  gps.begin();
  Serial.begin(115200);
  debugSer.begin(115200);
  debugSer.println("BOOT");
  cc.begin();
  debugSer.println(cc.partnum());
  debugSer.println(cc.status());
  if(cc.status() != 7) {
    digitalWrite(PA0, 1);
  }
  cc.simpleConfig();
  cc.Rx(128);
}
  
void loop() {
  gps.update();
  if(cc.status() == 6) {
    cc.flushRx();
    cc.Rx(128);
  }

  if(cc.avail() == 128) {
    Serial.write(0xAB);
    Serial.write(0xAB);
    cc.read(recBuf, 128);
    Serial.write(recBuf, 128);
    Serial.write(cc.rssi());
    Serial.write(gps.getFixType());
    int32_t lat = gps.getLat();
    int32_t lon = gps.getLon();
    uint32_t alt = gps.getHeight();
    Serial.write((uint8_t*) &lat, 4);
    Serial.write((uint8_t*) &lon, 4);
    Serial.write((uint8_t*) &alt, 4);
    digitalWrite(PA1, 1);
    delay(20);
    digitalWrite(PA1, 0);
    cc.Rx(128);   
  }
}
  
