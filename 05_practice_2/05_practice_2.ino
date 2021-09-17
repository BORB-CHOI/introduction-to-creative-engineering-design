#define PIN_LED 7
unsigned int toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  digitalWrite(PIN_LED, LOW);
  delay(1000);
  
  for (int i=0; i<10; i++){
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
    delay(100);
  }
  digitalWrite(PIN_LED, HIGH);
  exit(0);
}

int toggle_state(int toggle) {
  toggle = !toggle;
  return toggle;
}
