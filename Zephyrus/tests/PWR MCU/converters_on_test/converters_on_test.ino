#define EN0 PB0
#define EN1 PB1
#define EN2 PB3
#define EN3 PB4
#define EN4 PB5

int16_t enables[] = {EN0, EN1, EN2, EN3, EN4};

void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < 5; i++) {
    pinMode(enables[i], OUTPUT);
    digitalWrite(enables[i], 1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
