HardwareSerial debugSer(PC7, PC6);

#include "SPI.h"
#include "ADXL357.h"

#define MOSI PD7
#define MISO PB4
#define SCLK PA5
#define CS PD2

SPIClass SPI_3(MOSI, MISO, SCLK, -1);
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

ADXL357 accel(&SPI_3, settings, CS);

void setup() {
  delay(100);
  debugSer.begin(115200);
  SPI_3.begin();
  debugSer.println("BOOT");
  pinMode(PC14, OUTPUT);
  digitalWrite(PC14, 1);
  pinMode(PD1, OUTPUT);
  digitalWrite(PD1, 1);
  pinMode(PD2, OUTPUT);
  digitalWrite(PD2, 1);
  pinMode(PD4, OUTPUT);
  digitalWrite(PD4, 1);
  accel.setup();
}

void loop() {
  debugSer.println("X: " + String(accel.getAccelX()) + " // Y: " + String(accel.getAccelY()) + " // Z: " + String(accel.getAccelZ()));
  accel.update();
  delay(1000);
}
