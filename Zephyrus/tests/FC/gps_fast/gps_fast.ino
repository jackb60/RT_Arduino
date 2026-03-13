#include <GPS.h>

HardwareSerial debugSer(PC7, PC6);
HardwareSerial gpsSer(PB12, PB13);

GPS gps(&gpsSer);

void setup() {
  // put your setup code here, to run once:
  debugSer.begin(115200);
  delay(100);
  debugSer.println("BOOT");
  gps.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  gps.update();
  debugSer.println(gps.getITOW());
  debugSer.println(gps.getLon());
  debugSer.println(gps.getLat());
  debugSer.println();
  delay(100);
}
