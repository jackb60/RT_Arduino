#include <pyro.h>

HardwareSerial debugSer(PC7, PC6);


pyro myPyro;

int counter = 0;

void setup() {
  debugSer.begin(115200);
  debugSer.println("Starting Pyro Test");
  myPyro.begin();
  for(int i = 0; i < 6; i++) {
    myPyro.off(i);
    //myPyro.arm(i);
    //myPyro.fire(i);
  }
  debugSer.println("Yee Haw");
  delay(500);
  for(int i = 0; i < 6; i++) {
    myPyro.off(i);
    //myPyro.arm(i);
    //myPyro.fire(i);
  }  
    debugSer.println("all set");
}

void loop() {
  debugSer.println(counter);
  counter += 1;
    for (int i = 0; i < 6; i++) {
        debugSer.print("Pyro ");
        debugSer.print(i);
        debugSer.print(" Connected: ");
        debugSer.print(myPyro.connected(i));
        debugSer.print(" Resistance: ");
        debugSer.println(myPyro.resistance(i));
        
    }
    debugSer.println();
    delay(500);
}
