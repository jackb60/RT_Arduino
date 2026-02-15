HardwareSerial BMS_Serial(PA1, PA0);

void setup() {
  // put your setup code here, to run once:
  Serial.setTx(PA2);
  Serial.setRx(PA3);
  Serial.begin(115200);
  Serial.println("BOOT");

  BMS_Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  BMS_Serial.println("hello world boi");
  while (BMS_Serial.available()) {
    Serial.write(BMS_Serial.read());
  }
  delay(1000);

}
