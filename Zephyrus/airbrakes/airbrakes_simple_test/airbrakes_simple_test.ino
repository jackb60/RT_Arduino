

int pos = 0;
int mode = 0;
float airbrakesCtrlStartTime = 0;
float A0_req = 0.5;

float servoAngle = 0;

HardwareTimer *myTim = new HardwareTimer(TIM1);

void setAirbrakesServo(float fraction) {
  servoAngle = (-70.0f*fraction);
}

uint16_t degToUs(float deg) {
  return 1500 + (1.0 / .15) * deg;
}

void Update_IT_callback() {
  myTim->setCaptureCompare(1, degToUs(servoAngle), MICROSEC_COMPARE_FORMAT);
}


void setup() {
  // put your setup code here, to run once:
  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(115200);

  myTim->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PA8);
  myTim->setOverflow(20000, MICROSEC_FORMAT);
  myTim->setCaptureCompare(1, degToUs(0), MICROSEC_COMPARE_FORMAT);
  myTim->attachInterrupt(Update_IT_callback);
  myTim->resume();

  Serial.println("Angle pls");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (mode == 1) {
    float deployedFraction = 2.0f * A0_req * (millis()/1000.0f - airbrakesCtrlStartTime);
    Serial.print("Ramping to ");
    Serial.println(deployedFraction);
    Serial.print("Time so far: ");
    Serial.print(millis()/1000.0f - airbrakesCtrlStartTime);
    Serial.println(" s");
    if (deployedFraction >= A0_req) {
      mode = 0;
      Serial.print("Back to manual.");
      return;
    }
    setAirbrakesServo(deployedFraction);
  }
  else {
    //Serial.println(servoAngle);
    if (Serial.available()) {
      int angle = Serial.parseInt();
      while (Serial.available()) {
        Serial.read();
      }
      if(angle >= -180 && angle <= 180) {
        servoAngle = (angle);
        Serial.print("Servo moved to: ");
        Serial.println(angle);
      } else {
        Serial.println("Invalid input ! Enter diff bruh.");
      }
      if (angle > 200) {
        mode = 1;
        A0_req = ((float) angle - 200.0f)/100.0f;
        Serial.println("Preparing Ramp...");
        setAirbrakesServo(0);
        delay(1500);
        airbrakesCtrlStartTime = millis()/1000.0f;
        Serial.print("Starting Ramp at ");
        Serial.println(airbrakesCtrlStartTime);
      }
    }
  }

}