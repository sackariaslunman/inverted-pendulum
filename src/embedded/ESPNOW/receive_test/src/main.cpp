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


// Time
unsigned long startMillis = 0;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    uint16_t encoder_pos;
    byte sender_id;
} struct_message;

// Create a struct_message called recivedData
struct_message recivedData;
struct_message pendulumData_0;
struct_message pendulumData_1;
// struct_message 2Data;
// struct_message 3Data;
struct_message cartData;

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
    pendulumData_0 = recivedData;
  }
  else if (recivedData.sender_id == 1){
    pendulumData_1 = recivedData;
  }
  else if (recivedData.sender_id == 2){
    // 2Data = recivedData;
  }
  else if (recivedData.sender_id == 3){
    // 3Data = recivedData;
  }
  else if (recivedData.sender_id == 4){
    cartData = recivedData;
  }
  else{
    Serial.println("Error: Invalid sender ID");
  }
}




 
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
}
 
void loop() {
// See if 100 ms has passed
  if (millis() - startMillis > 100)  {
    // Reset the timer
    startMillis = millis();

    // Print encoder pos for each id
    Serial.print("Pendulum 0: ");
    Serial.print(pendulumData_0.encoder_pos);
    Serial.print("  Pendulum 1: ");
    Serial.print(pendulumData_1.encoder_pos);

    Serial.print("  Cart: ");
    Serial.print(cartData.encoder_pos);

    Serial.println();
  }

}