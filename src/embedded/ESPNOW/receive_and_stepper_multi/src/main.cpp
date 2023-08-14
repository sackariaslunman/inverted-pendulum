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
// struct_message 2Data;
// struct_message 3Data;
struct_message cartData;


// Encoder objects
RotaryEncoder pendulum_0;
RotaryEncoder pendulum_1;
RotaryEncoder cart;

// array for counting espnow updates per id
int update_count[5] = {0,0,0,0,0};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&recivedData, incomingData, sizeof(recivedData));
  // Serial.print("Bytes received: ");
  // Serial.print(len);
  // Serial.print("  Encoder_pos: ");
  // Serial.print(recivedData.encoder_pos);
  // Serial.print("  Sender ID: ");
  // Serial.print(recivedData.sender_id);  
  // Serial.println();

  // Update struct with corresponding id
  if (recivedData.sender_id == 0){
    // pendulumData_0 = recivedData;
    pendulum_0.update(recivedData.encoder_pos);
    update_count[0] += 1;
  }
  else if (recivedData.sender_id == 1){
    // pendulumData_1 = recivedData;
    pendulum_1.update(recivedData.encoder_pos);
    update_count[1] += 1;
  }
  else if (recivedData.sender_id == 2){
    // 2Data = recivedData;
  }
  else if (recivedData.sender_id == 3){
    // 3Data = recivedData;
  }
  else if (recivedData.sender_id == 4){
    // cartData = recivedData;
    cart.update(recivedData.encoder_pos);
    update_count[4] += 1;
  }
  else{
    Serial.println("Error: Invalid sender ID");
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



long last_update = 0;
long last_loop = 0;
int dt = 10000;

// State  = pos (m), vel (m/s), angle (rad), angle_vel (rad/sec)
double state[] = {0,0,2,0,0,0};
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
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
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

    auto pendulum_0_pos = -pendulum_0.getPos();
    auto pendulum_1_pos = pendulum_1.getPos();
    auto pendulum_0_vel = pendulum_0.getVel();
    auto pendulum_1_vel = -pendulum_1.getVel();

    state[0] = stepper.currentPosition() * length_per_step;
    state[1] = stepper.speed() * length_per_step;
    // state[2] = pendulum_0.getPos();
    // state[3] = pendulum_0.getVel();
    // state[4] = pendulum_1.getPos() + pendulum_0.getPos(); 
    // state[5] = pendulum_1.getVel() + pendulum_0.getVel();

    state[2] = pendulum_0_pos;
    state[3] = pendulum_0_vel;
    state[4] = pendulum_1_pos + pendulum_0_pos + PI; 
    state[5] = pendulum_1_vel + pendulum_0_vel;

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
      control_stepperSpeed = maxSpeed-1;
    }
    else if (control_stepperSpeed < -maxSpeed)
    {
      control_stepperSpeed = -maxSpeed+1;
    }
    stepper.setSpeed(control_stepperSpeed / length_per_step);

    last_loop = micros();
  }

  stepper.runSpeed();



// See if 100 ms has passed
  // if (millis() - startMillis > 1000)  {
  //   // Reset the timer
  //   startMillis = millis();

  //   // Print encoder pos for each id
  //   Serial.print("Pendulum 0: ");
  //   Serial.print(pendulum_0.getPos());
  //   Serial.print("  Pendulum 1: ");
  //   Serial.print(pendulum_1.getPos());
  //   Serial.print("  Cart: ");
  //   Serial.print(cart.getPos());

  //   // Velocity for each id
  //   Serial.print("  Pendulum 0: ");
  //   Serial.print(pendulum_0.getVel());
  //   Serial.print("  Pendulum 1: ");
  //   Serial.print(pendulum_1.getVel());
  //   Serial.print("  Cart: ");
  //   Serial.print(cart.getVel());


  //   // Print update count for each id
  //   // Serial.print("  Pendulum 0: ");
  //   // Serial.print(update_count[0]);
  //   // Serial.print("  Pendulum 1: ");
  //   // Serial.print(update_count[1]);
  //   // Serial.print("  Cart: ");
  //   // Serial.print(update_count[4]);

  //   // clear update count
  //   update_count[0] = 0;
  //   update_count[1] = 0;
  //   update_count[4] = 0;


  //   Serial.println();
  // }

}