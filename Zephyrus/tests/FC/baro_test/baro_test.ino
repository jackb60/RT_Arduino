HardwareSerial debugSer(PC7, PC6);

#include "SPI.h"
#include "baro.h"

#define MOSI PD7
#define MISO PB4
#define SCLK PA5
#define BARO_CS PD3

SPIClass SPI_3(MOSI, MISO, SCLK, -1);
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

baro myBaro(&SPI_3,settings, BARO_CS);

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
  pinMode(PD4, OUTPUT);
  digitalWrite(PD4, 1);
  SPI_3.begin();
  myBaro.begin();
  delay(100);
  debugSer.begin(115200);
  debugSer.println("Calibration Values:");
  for (int i = 1; i <= 6; i++) {
    debugSer.print("C");
    debugSer.print(i);
    debugSer.print(": ");
    debugSer.println(myBaro.getCalibrationConstant(i), HEX);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  debugSer.println("\n--------------\n");
  debugSer.print("Data Packet #");
  debugSer.println(counter);
  counter += 1;

  myBaro.updateRawTemp();
  myBaro.updateRawPress();


  debugSer.print("Raw Temperature: ");
  debugSer.println(myBaro.getRawTemp());
  debugSer.print("Raw Pressure: ");
  debugSer.println(myBaro.getRawPress());
  debugSer.print("Temperature (C): ");
  debugSer.println(myBaro.getTemperature());
  debugSer.print("Altitude (m): ");
  debugSer.println(myBaro.getAltitude());
  debugSer.println("Calibration Values:");
  for (int i = 1; i <= 6; i++) {
    debugSer.print("C");
    debugSer.print(i);
    debugSer.print(": ");
    debugSer.println(myBaro.getCalibrationConstant(i), HEX);
  }
  delay(2000);
}