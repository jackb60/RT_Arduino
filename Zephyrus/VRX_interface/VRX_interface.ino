HardwareSerial fusionSerial (PA9);
void setup() {
  Serial.begin(115200);
  fusionSerial.begin(115200);
}
void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()){
    fusionSerial.write(Serial.read());
  }
  fusionSerial.enableHalfDuplexRx();
  while(fusionSerial.available()){
    Serial.write(fusionSerial.read());
  }
}