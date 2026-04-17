HardwareSerial debugSer(PC7, PC6);

#include "SPI.h"
#include "flash.h"

#define MOSI PD7
#define MISO PB4
#define SCLK PA5
#define FLASH_CS PD4

#define PACKETS 12000

uint8_t testReadPage[512];

SPIClass SPI_3(MOSI, MISO, SCLK, -1);
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

flash myFlash(&SPI_3,settings, FLASH_CS);
int counter = 0;
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
  pinMode(PD3, OUTPUT);
  digitalWrite(PD3, 1);
  SPI_3.begin();
  debugSer.begin(115200);
  delay(1000);
  myFlash.begin();

}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0; i < PACKETS; i+=4) {
    while (myFlash.isBusy()) {}
    myFlash.readPage((uint8_t*) testReadPage, i * 512);
    for(int x = 0; x < 4; x++) {
      debugSer.write(0xAB);
      debugSer.write(0xAB);
      debugSer.write(testReadPage + x * 128, 128);
      debugSer.write((uint8_t) 0);
      debugSer.write((uint8_t) 0);
      int32_t lat = 0;
      int32_t lon = 0;
      uint32_t alt = 0;
      debugSer.write((uint8_t*) &lat, 4);
      debugSer.write((uint8_t*) &lon, 4);
      debugSer.write((uint8_t*) &alt, 4);
      delay(10);
    }
  }
}