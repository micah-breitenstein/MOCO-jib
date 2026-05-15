// Test sketch to verify DIR pin toggles
// Upload this to the Nano, then measure voltage on pin 2

void setup() {
  pinMode(2, OUTPUT);  // DIR pin
  pinMode(13, OUTPUT); // LED for visual confirmation
}

void loop() {
  // Toggle DIR pin every second
  digitalWrite(2, HIGH);
  digitalWrite(13, HIGH);
  delay(1000);
  
  digitalWrite(2, LOW);
  digitalWrite(13, LOW);
  delay(1000);
}
