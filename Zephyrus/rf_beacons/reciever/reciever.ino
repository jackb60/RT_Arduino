HardwareSerial debugSer(PA3, PA2);
HardwareSerial draSer(PA10, PA9);

char cfg[] = "AT+DMOSETGROUP=0,147.4500,147.4500,0000,1,0000\r\n";

void setup() {
  // put your setup code here, to run once:
  debugSer.begin(115200);
  draSer.begin(9600);
  pinMode(PB3, OUTPUT);
  digitalWrite(PB3, 1);

  delay(500);
  draSer.write(cfg, sizeof(cfg));
}

void loop() {
  // put your main code here, to run repeatedly:
}