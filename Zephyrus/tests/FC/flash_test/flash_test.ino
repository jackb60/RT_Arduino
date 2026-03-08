HardwareSerial debugSer(PC7, PC6);

#include "SPI.h"
#include "flash.h"

#define MOSI PD7
#define MISO PB4
#define SCLK PA5
#define FLASH_CS PD4

uint8_t testWritePage[512];
uint8_t testReadPage[512];

SPIClass SPI_3(MOSI, MISO, SCLK, -1);
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

flash myFlash(&SPI_3,settings, FLASH_CS);

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
  delay(100);
  myFlash.begin();
  debugSer.println("Erasing flash sector 0");
  myFlash.sectorErase(0);
    while(myFlash.isBusy()) {
        debugSer.print(".");
        delay(100);
    }
  debugSer.println("Programming flash");
  for (int i = 0; i < 512; i++) {
    testWritePage[i] = i;
  }
  myFlash.programPage((uint8_t*) testWritePage, 0);
  while(myFlash.isBusy()) {}
  debugSer.println("Reading flash");
  myFlash.readPage((uint8_t*) testReadPage, 0);
  debugSer.println("Data read from flash:");
  for (int i = 0; i < 512; i++) {
    debugSer.print(testReadPage[i], HEX);
    debugSer.print(" ");
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}