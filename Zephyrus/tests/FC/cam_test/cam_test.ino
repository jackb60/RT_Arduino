#include <cam.h>

#define USED_SER CAM0_SER

HardwareSerial CAM0_SER(PE7, PE8);
HardwareSerial CAM1_SER(PE0, PE1);
HardwareSerial CAM2_SER(PB15, PB14);
HardwareSerial debugSer(PC7, PC6);


cam mycam(&USED_SER);

void setup() {
  debugSer.begin(115200);
  debugSer.println("Starting CAM Test");
  mycam.begin();
}

void loop() {
    mycam.getInfo();
    delay(1000);
    while(USED_SER.available()) {
      debugSer.print(USED_SER.read(), HEX);
      debugSer.print(" ");
    }
}
