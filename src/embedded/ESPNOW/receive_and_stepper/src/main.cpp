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

// Timer to measure time between messages
// last time
long startTime = 0;
// Current time
long stopTime = 0;

int main_loop_i = 0;

// Time
unsigned long startMillis = 0;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    uint16_t b;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  // Serial.print("b: ");
  // Serial.println(myData.b); 
}




// Stepper
#include "AccelStepper.h"

#define dirPin 22
#define stepPin 21

AccelStepper stepper(1, stepPin, dirPin);

int maxSpeed = 10000;
int accel = 50000;

int targetPos = 0;
int targetAngle = 2048;
bool stop = false;
bool demo = false;
bool p_reg = false;

double p_reg_pos = 0.1;
double i_reg_pos = 0.01;

double error = 0;
double error_sum = 0;
double error_last = 0;



#define steps_per_revolution 1000
#define wheel_circumference 140*0.002 // m
const double length_per_step = wheel_circumference / steps_per_revolution;

#define encoded_steps_per_revolution 4096
const double angle_per_step = 2*PI / encoded_steps_per_revolution; // rad

#define rail_length 1.2 // m

#define maxPos rail_length / length_per_step / 2 
#define minPos -rail_length / length_per_step / 2 


// serial-communication\src\main.cpp
long last_update = 0;
long last_loop = 0;
int dt = 5000;

// State  = pos (m), vel (m/s), angle (rad), angle_vel (rad/sec)
double state[] = {0,0,2,0};
// Control = Linear acceleration (m/s^2) of the cart
double control[] = {0};

double old_angle = 0;

double control_stepperSpeed  = 0; // m/s


// double array for low pass of angle velocity
#define angle_vel_array_size 5
double angle_vel_array[angle_vel_array_size] = {0};
int angle_vel_index = 0;
 
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

  // Setup led pin as output
  pinMode(5, OUTPUT);

  // Setup stepper
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(accel);
  stepper.setCurrentPosition(0);

}
 
void loop() {

  if (micros() - last_update > dt)
  {
    last_update = (micros() / dt) * dt;

    Serial.write('x');
    Serial.write('s');
    Serial.write('t');
    // States
    double pos_state = stepper.currentPosition() * length_per_step;
    double vel_state = stepper.speed() * length_per_step;
    // Modulo
    double angle_state = (myData.b/double(encoded_steps_per_revolution))*2*PI + PI;
    // double angle_state = (myData.b % encoded_steps_per_revolution)/encoded_steps_per_revolution *2*PI  + PI; 
    // double angle_vel_state = (angle_state - old_angle) / dt * 1000;
    // double angle_vel_state = atan2(sin(angle_state - old_angle), cos(angle_state - old_angle)) / dt * 1000;



    double average_angle_vel = 0;
    for (int i = 0; i < angle_vel_array_size; i++)
    {
      average_angle_vel += angle_vel_array[i];
    }
    average_angle_vel /= angle_vel_array_size;
    old_angle = angle_state;
    state[0] = pos_state;
    state[1] = vel_state;
    state[2] = angle_state;
    state[3] = average_angle_vel;
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

    control_stepperSpeed += control[0] * (double(time_since_last_loop) / 1000000);
    // control_stepperSpeed += control[0];
    stepper.setSpeed(control_stepperSpeed / length_per_step);

    // calculate velocity
    double angle_state = (myData.b/double(encoded_steps_per_revolution))*2*PI + PI;
    double angle_vel = atan2(sin(angle_state - old_angle), cos(angle_state - old_angle)) / time_since_last_loop * 1000000;
    // double angle_vel = (angle_state - old_angle) / time_since_last_loop * 1000;
    old_angle = angle_state;

    // Append to circular array
    angle_vel_array[angle_vel_index] = angle_vel;
    angle_vel_index = (angle_vel_index + 1) % angle_vel_array_size;
    last_loop = micros();
  }




  


  stepper.runSpeed();


}