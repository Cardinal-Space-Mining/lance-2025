void setup() {
  Serial.begin(9600);       // Start serial communication at 9600 baud
  pinMode(13, OUTPUT);      // Set pin 13 as an output (usually the built-in LED)
}

void loop() {
  if (Serial.available() > 0) {       // Check if data is available
    char command = Serial.read();     // Read a single byte/character

    if (command == '1') {
      digitalWrite(13, HIGH);         // Turn LED on
      Serial.println("LED ON");
    } else if (command == '0') {
      digitalWrite(13, LOW);          // Turn LED off
      Serial.println("LED OFF");
    } else {
      Serial.println("Unknown command");
    }
  }
}