#include <INA232.h>
#include <Wire.h>

#define EN0 PB0
#define EN1 PB1
#define EN2 PB3
#define EN3 PB4
#define EN4 PB5

int16_t enables[] = {EN0, EN1, EN2, EN3, EN4};


TwoWire Wire1(PA6,PA7);
TwoWire Wire2(PC14,PB6);


INA232 monitor1(&Wire2, 0b1000001);
INA232 monitor2(&Wire2, 0b1000000);
INA232 monitor3(&Wire2, 0b1000010);
INA232 monitor4(&Wire1, 0b1000001);
INA232 monitor5(&Wire1, 0b1000000);
INA232 monitor6(&Wire1, 0b1000010);

/*
Monitor Key:
1: 3.3V
2: 3V
3: 5V
4: 7.4V
5: 28V
6: 8.4V 
*/


void setup() {
  // put your setup code here, to run once:
  Serial.setTx(PA2);
  Serial.setRx(PA3);
  Serial.begin(115200);
  Serial.println("BOOT");

  for (int i = 0; i < 5; i++) {
    pinMode(enables[i], OUTPUT);
    digitalWrite(enables[i], 1);
  }
  /*Wire.setSDA(PA6);
  Wire.setSCL(PA7);
  Wire.begin();*/

  Wire1.begin();
  Wire2.begin();

  monitor1.begin();
  monitor2.begin();
  monitor3.begin();
  monitor4.begin();
  monitor5.begin();
  monitor6.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Monitor 1: ");
  Serial.println(monitor1.current());
  Serial.println(monitor1.voltage());
  Serial.println("Monitor 2: ");
  Serial.println(monitor2.current());
  Serial.println(monitor2.voltage());
  Serial.println("Monitor 3: ");
  Serial.println(monitor3.current());
  Serial.println(monitor3.voltage());
  Serial.println("Monitor 4: ");
  Serial.println(monitor4.current());
  Serial.println(monitor4.voltage());
  Serial.println("Monitor 5: ");
  Serial.println(monitor5.current());
  Serial.println(monitor5.voltage());
  Serial.println("Monitor 6: ");
  Serial.println(monitor6.current());
  Serial.println(monitor6.voltage());
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  delay(1000);
}
