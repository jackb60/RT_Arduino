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
    cc.freq915();
    cc.Rx(128);
  }

  //Serial.println(cc.status());
  //Serial.println(cc.avail());
  //Serial.println();

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
    delay(10);
    digitalWrite(PA1, 0);
    txBuf[0] = 0xAA;

    if(Serial.available() >= 16 && Serial.read() == 0xAA) {
      Serial.readBytes((char*) txBuf + 1, 15);
      uint16_t chksum = 0;
      
      for (uint8_t i = 1; i < 14; i++) {
        chksum += txBuf[i];
      }
      if (chksum == ((txBuf[14] << 8) + txBuf[15])) {

      } else {
        
        for (uint8_t i = 1; i < 16; i++) {
          txBuf[i] = 0;
        }
      }

    } else {
      txBuf[0] = 0xAA;
      for (uint8_t i = 1; i < 16; i++) {
        txBuf[i] = 0;
      }
    }
    cc.freq917();
    cc.Tx(txBuf, 16);
    pyros = 0;
    while (cc.status() != 0) {}
    cc.freq915();
    cc.Rx(128);   
  }
}
  
