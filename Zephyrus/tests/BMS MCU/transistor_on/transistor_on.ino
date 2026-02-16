#include "BQ76922.h"
#include "Wire.h"

BQ76922 bms;

void setup() {
  // put your setup code here, to run once:
  Serial.setTx(PA2);
  Serial.setRx(PA3);
  Serial.begin(115200);
  Serial.println("BOOT");
  Wire.setSCL(PA7);
  Wire.setSDA(PA6);
  bms.begin();
  delay(100);
  Serial.println("BMS BEGIN");
  int16_t dummy = bms.cellVoltage(0); //Chip locks up without a read first
  bms.enterConfigMode();
  Serial.println("ENTER CONFIG");
  bms.disableProtections();
  bms.enableFet();
  bms.chargePumpEnable();
  bms.exitConfigMode();
}

void loop() {
  // put your main code here, to run repeatedly:
  for(byte i = 1; i <= 5; i++) {
    Serial.print("Voltage at pin ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(bms.cellVoltage(i));
    Serial.println(" mV");
  }

  Serial.print("Stack voltage: ");
  Serial.print(bms.stackVoltage());
  Serial.println(" userV (default cV)");

  Serial.print("Current: ");
  Serial.print(bms.current());
  Serial.println(" userA (defualt mA)");

  Serial.print("Temperature: ");
  Serial.print(bms.temp());
  Serial.println(" C");

  Serial.print("FET: ");
  Serial.println(bms.fetStatus(), HEX);
  delay(2000);
}
