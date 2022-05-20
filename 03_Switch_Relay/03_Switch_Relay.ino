const byte switch_pin = 30;
const byte relay_pin = 22;
bool high = false;

void setup() {
  pinMode(switch_pin, INPUT_PULLUP);
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, LOW);
  Serial.begin(9600);
}

void loop() {
  if (digitalRead(switch_pin) == HIGH && !high) {
    digitalWrite(relay_pin, HIGH);
    Serial.println(digitalRead(switch_pin));
    high = true;
  }
  else if (digitalRead(switch_pin) == LOW && high) {
    digitalWrite(relay_pin, LOW);
    Serial.println(digitalRead(switch_pin));
    high = false;
  }
  delay(100);
}
