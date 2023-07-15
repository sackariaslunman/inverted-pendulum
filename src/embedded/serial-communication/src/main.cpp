#include <Arduino.h>

#define LED 12

long last_update = 0;
int dt = 10;

float state[] = {1,2,3,4,5,6};
float control[14400];

void setup() {
  Serial.begin(576000);
  Serial.setTimeout(10*1000);
  Serial.write("start");
  pinMode(LED,OUTPUT);
  last_update = millis();
}

void loop() {
  if (millis() < last_update + dt)
    return;

  last_update = (millis() / dt) * dt;

  Serial.write("msg");
  auto time = (float)(last_update / 1000.0);
  Serial.write((uint8_t*)&time, sizeof(time));
  Serial.write((uint8_t*)state, sizeof(state));

  if (Serial.available()) {
    if (Serial.read() == 's') {
      digitalWrite(LED, HIGH);
      Serial.readBytes((uint8_t*)control, sizeof(control));
      delay(50);
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, HIGH);
    }
  } else {
    digitalWrite(LED, LOW);
  }
}