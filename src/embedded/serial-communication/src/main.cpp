#include <Arduino.h>

uint8_t buffer[256];

void setup() {
  Serial.begin(921600);
  Serial.println("Hello World");
}

void loop() {
  if (Serial.available() > 0) {
    Serial.readBytesUntil('\n', buffer, 256);
  }
}
