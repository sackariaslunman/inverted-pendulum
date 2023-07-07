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
    // char a[32];
    uint16_t b;
    // float c;
    // bool d;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  // Serial.print("Char: ");
  // Serial.println(myData.a);
  // Serial.print("Int: ");
  // Serial.print(myData.b); 
  // Serial.print("\t");
  // Serial.print(millis() - startMillis);
  // Serial.print("Float: ");
  // Serial.println(myData.c);
  // Serial.print("Bool: ");
  // Serial.println(myData.d);
  // Serial.println();

  // if (myData.b == 0) {
  //   startTime = millis();
  // }

  // if (myData.b == 999) {
  //   stopTime = millis();
  //   Serial.print("Time between messages: ");
  //   Serial.print(stopTime - startTime);
  //   Serial.println(" ms");
  // }
  
}




// Stepper
#include "AccelStepper.h"

#define dirPin 22
#define stepPin 21

AccelStepper stepper(1, stepPin, dirPin);

int maxSpeed = 1000;
int accel = 1000;

int targetPos = 0;
int targetAngle = 2048;
bool stop = false;
bool demo = false;
bool p_reg = false;

float p_reg_pos = 0.1;
float i_reg_pos = 0.01;

float error = 0;
float error_sum = 0;
float error_last = 0;


#define minPos 0
#define maxPos 40*100

// Print string
String printString = "";



 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
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
  Serial.begin(115200);
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(accel);
  stepper.setCurrentPosition(0);

}
 
void loop() {
  // Read serial for time sync
  if (startMillis == 0){
    if (Serial.available() > 0) {
      startMillis = millis();    
      Serial.read();
    }
  }

  // Read serial input:
  if (Serial.available()) {
    char inChar = Serial.read();

    printString = "";

    printString += "Target position: ";
    printString += targetPos;
    printString += "\tCurrent position: ";
    printString += stepper.currentPosition();

    if (inChar == 'd') {
      targetPos += 100;
      if (targetPos > maxPos) {
        targetPos = maxPos;
      }
      stepper.moveTo(targetPos);
    }
    if (inChar == 'D') {
      targetPos += 1000;
      if (targetPos > maxPos) {
        targetPos = maxPos;
      }
      stepper.moveTo(targetPos);
    }
    if (inChar == 'c') {
      targetPos += 1;
      // if (targetPos > maxPos) {
      //   targetPos = maxPos;
      // }
      stepper.moveTo(targetPos);
    }
    if (inChar == 'a') {
      targetPos -= 100;
      if (targetPos < minPos) {
        targetPos = minPos;
      }
      stepper.moveTo(targetPos);
    }
    if (inChar == 'A') {
      targetPos -= 1000;
      if (targetPos < minPos) {
        targetPos = minPos;
      }
      stepper.moveTo(targetPos);
    }
    if (inChar == 'z') {
      targetPos -= 1;
      // if (targetPos < minPos) {
      //   targetPos = minPos;
      // }
      stepper.moveTo(targetPos);
    }
    if (inChar == 'w') {
      stepper.stop();
    }
    if (inChar == 's') {
      stop = !stop;
    }
    if (inChar == 'q') {
      stepper.setCurrentPosition(0);
      targetPos = 0;
    }
    if (inChar == 'e') {
      stepper.setCurrentPosition(maxPos);
      targetPos = maxPos;
    }
    if (inChar == '+') {
      maxSpeed *= 1.1;
      stepper.setMaxSpeed(maxSpeed);
      printString += "\tMax speed: ";
      printString += maxSpeed;

      // stepper.setSpeed(maxSpeed);
    }
    if (inChar == '-') {
      maxSpeed /= 1.1;
      stepper.setMaxSpeed(maxSpeed);
      printString += "\tMax speed: ";
      printString += maxSpeed;

      // stepper.setSpeed(maxSpeed);
    }
    if (inChar == '*') {
      accel *= 1.1;
      stepper.setAcceleration(accel);
      printString += "\tAcceleration: ";
      printString += accel;
    }
    if (inChar == '/') {
      accel /= 1.1;
      stepper.setAcceleration(accel);
      printString += "\tAcceleration: ";
      printString += accel;
    }
    if (inChar == 'p') {
      printString += "\tMax speed: ";
      printString += maxSpeed;
      printString += "\tAcceleration: ";
      printString += accel;
    }
    if (inChar == 'l') {
      demo = !demo;
    }
    if (inChar == 'o')
    {
      p_reg = !p_reg;
    }
    if ( inChar == 'k')
    {
      // decrease p
      p_reg_pos /= 1.1;
    }
    if ( inChar == 'K')
    {
      // increase p
      p_reg_pos *= 1.1;
    }
    if ( inChar == 'i')
    {
      // decrease i
      i_reg_pos /= 1.1;
    }
    if ( inChar == 'I')
    {
      // increase i
      i_reg_pos *= 1.1;
    }
    if ( inChar == 't')
    {
      // Use current angle as target
      targetAngle = myData.b;
    }
    

    Serial.println(printString);

  }


  if (demo) {
    // Demo:
    #define safeDist 100
    if (stepper.distanceToGo() == 0) {
      if (stepper.currentPosition() == minPos + safeDist) {
        stepper.moveTo(maxPos - safeDist);
      } else {
        stepper.moveTo(minPos + safeDist);
      }
    }
  }

  if (p_reg) {
    // PI-regulator:
    float p = p_reg_pos;
    float i = i_reg_pos;
    #define safeDist 100
    int input = myData.b;
    int min_input = targetAngle - 512;
    int max_input = targetAngle + 512;
    bool enable;
    if (input > min_input && input < max_input) {
      enable = true;
    } else {
      enable = false;
    }

    if (enable) {
      int current_pos = stepper.currentPosition();

      error = input - targetAngle;
      #define max_error_sum 10000
      error_sum += error;
      if (error_sum > max_error_sum) {
        error_sum = max_error_sum;
      }
      if (error_sum < -max_error_sum) {
        error_sum = -max_error_sum;
      }
      int target = current_pos + p * error + i * error_sum;



      if (target > maxPos - safeDist) {
        target = maxPos - safeDist;
        error_sum = 0;
      }
      if (target < minPos + safeDist) {
        target = minPos + safeDist;
        error_sum = 0;
      }
      stepper.moveTo(target);
    }


  }

  if (main_loop_i >= 1000) {
    // print
    main_loop_i = 0;
    Serial.print("\tError: ");
    Serial.print(error);
    Serial.print("\tError sum: ");
    Serial.print(error_sum);
    Serial.print("\tInput: ");
    Serial.print(myData.b);
    Serial.print("\tP_gain: ");
    Serial.print(p_reg_pos, 5);
    Serial.print("\tI_gain: ");
    Serial.print(i_reg_pos, 5);
    Serial.print("\tMax speed: ");
    Serial.print(maxSpeed);
    Serial.print("\tAcceleration: ");
    Serial.print(accel);
    Serial.println();

    //  Error: 502.00   Error sum: 10000.00     Input: 65535    P_gain: 0.06830 I_gain: 0.00102 Max speed: 10794        Acceleration: 96753
  }
  main_loop_i++;

  // Update stepper motor:
  int i = 0;
  if (!stop) {
    for (i = 0; i < 1000; i++) {
      stepper.run();
      // stepper.runSpeed();
    }
  }
  else
  {
    stepper.setCurrentPosition(targetPos);
    demo = false;
  }



}