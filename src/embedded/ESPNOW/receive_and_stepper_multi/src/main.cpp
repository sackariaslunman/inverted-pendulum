#include <Arduino.h>

/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

#include "encoder.h"

// Define number of pendulums
// #define ONE_PENDULUM
#define TWO_PENDULUMS


// Time
unsigned long startMillis = 0;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int16_t encoder_pos;
    byte sender_id;
} struct_message;

// Create a struct_message called recivedData
struct_message recivedData;
struct_message pendulumData_0;
struct_message pendulumData_1;
struct_message cartData;

// Encoder objects
RotaryEncoder pendulum_0;
RotaryEncoder pendulum_1;
RotaryEncoder cart;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&recivedData, incomingData, sizeof(recivedData));

  // Update struct with corresponding id
  if (recivedData.sender_id == 0){
    // pendulumData_0 = recivedData;
    pendulum_0.update(recivedData.encoder_pos);
  }
  else if (recivedData.sender_id == 1){
    // pendulumData_1 = recivedData;
    pendulum_1.update(recivedData.encoder_pos);
  }
  else if (recivedData.sender_id == 4){
    // cartData = recivedData;
    cart.update(recivedData.encoder_pos);
  }
  else{
    // Serial.println("Error: Invalid sender ID");
  }
}


// Stepper
#include "AccelStepper.h"

#define dirPin 22
#define stepPin 21
 
int maxSpeed = 9000;
int accel = 50000;

int targetPos = 0;

#define steps_per_revolution 1000
#define wheel_circumference 140*0.002 // m
const double length_per_step = wheel_circumference / steps_per_revolution;

#define encoded_steps_per_revolution 4096
const double angle_per_step = 2*PI / encoded_steps_per_revolution; // rad

#define rail_length 1.2 // m

#define maxPos rail_length / length_per_step / 2 
#define minPos -rail_length / length_per_step / 2 

AccelStepper stepper(1, stepPin, dirPin); 

double prev_pos_0 = 0;
double prev_pos_1 = 0;


long last_update = 0;
long last_loop = 0;
// int dt = 10000;
int dt = 5000;

// State  = pos (m), vel (m/s), angle (rad), angle_vel (rad/sec)
#ifdef ONE_PENDULUM
double state[] = {0,0,0,0};
#endif
#ifdef TWO_PENDULUMS
double state[] = {0,0,0,0,0,0};
#endif
// Control = Linear acceleration (m/s^2) of the cart
double control[] = {0};
double control_stepperSpeed  = 0; // m/s


void setup() {
  // Initialize Serial Monitor
  Serial.begin(500000);
  Serial.setTimeout(dt);

  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    // Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // Setup stepper
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(accel);
  stepper.setCurrentPosition(0);

  // Setup led pin as output
  pinMode(5, OUTPUT);
}
 
void loop() {

  if (micros() - last_update > dt) 
  {
    last_update = (micros() / dt) * dt;
    // Send updated states for each id
    Serial.write('x');
    Serial.write('s');
    Serial.write('t');

    auto pendulum_0_pos = pendulum_0.pos;
    // auto pendulum_0_vel = pendulum_0.getVel();
    auto pendulum_0_vel = atan2(sin(pendulum_0_pos - prev_pos_0), cos(pendulum_0_pos - prev_pos_0)) / dt * 1000000;
    
    state[0] = stepper.currentPosition() * length_per_step;
    state[1] = stepper.speed() * length_per_step;
    state[2] = pendulum_0_pos;
    state[3] = pendulum_0_vel;
    prev_pos_0 = pendulum_0_pos;

    #ifdef TWO_PENDULUMS
    auto pendulum_1_pos = pendulum_1.pos;
    // auto pendulum_1_vel = pendulum_1.getVel();
    auto pendulum_1_vel = atan2(sin(pendulum_1_pos - prev_pos_1), cos(pendulum_1_pos - prev_pos_1)) / dt * 1000000;
    state[4] = pendulum_1_pos + pendulum_0_pos + PI; 
    state[5] = pendulum_1_vel + pendulum_0_vel;
    prev_pos_1 = pendulum_1_pos;
    #endif

    Serial.write((uint8_t*)state, sizeof(state));

    // Control
    while (!Serial.available()) {

    }

    Serial.readBytes((uint8_t*)control, sizeof(control));
  }

  // Control
  // Calculate speed based on control acceleration
  if (micros() - last_loop > 1000)
  {
    
    int time_since_last_loop = micros() - last_loop;

    // Acceleration update
    control_stepperSpeed += control[0] * (double(time_since_last_loop) / 1000000);
    // Limit speed
    if (control_stepperSpeed > maxSpeed)
    {
      control_stepperSpeed = maxSpeed;
    }
    else if (control_stepperSpeed < -maxSpeed)
    {
      control_stepperSpeed = -maxSpeed;
    }
    stepper.setSpeed(control_stepperSpeed / length_per_step);

    last_loop = micros();
  }

  stepper.runSpeed();

}