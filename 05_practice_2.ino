#define PIN_LED 7

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 0);
  delay(1000);
}

void loop() {
    int i =0;
    while(i<5) {
      digitalWrite(PIN_LED, 1);
      delay(200);
      digitalWrite(PIN_LED, 0);
      delay(200);
      i++;
    }
    digitalWrite(PIN_LED, 1);
    while(1) {}
}
