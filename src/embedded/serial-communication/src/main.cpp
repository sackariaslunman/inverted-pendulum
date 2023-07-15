#include <Arduino.h>

long last_update = 0;
int dt = 10;

float state[] = {0,0,0,0,0,0};
float control[] = {0};

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
}

void loop() {
  if (millis() - last_update < dt) return;

  last_update += dt;

  Serial.write((uint8_t*)state, sizeof(state));

  while (!Serial.available()) {
    if (millis() - last_update > dt) {
      return;
    }
  }

  Serial.readBytes((char*)control, sizeof(control));
  state[0] = control[0];
}