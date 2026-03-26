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

byte recBuf[128];
byte txBuf[16];

byte state = 0;
byte pyros = 0;

GPS gps(&gpsSer);

void setup() {
  // put your setup code here, to run once:
  delay(1000);
  gps.begin();
  Serial.begin(115200);
  //Serial.println("BOOT");
  cc.begin();
  cc.simpleConfig();
  cc.Rx(128);
}
  
void loop() {
  gps.update();
  if(cc.status() == 6) {
    cc.flushRx();
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
    Serial.write(gps.getLat());
    Serial.write(gps.getLon());
    Serial.write(gps.getHeight());
    delay(10);
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
    cc.Tx(txBuf, 16);
    pyros = 0;
    while (cc.status() != 0) {}
    cc.Rx(128);   
  }
}
  
