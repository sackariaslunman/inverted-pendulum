#include <Arduino.h>

int led_pin = 5;

// Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/


// Första ESP MAC = 58:BF:25:37:BF:C0 COM 12
// Andra ESP MAC =  58:BF:25:37:F8:C8 COM 14
// Nandanas ESP = 58:BF:25:36:FC:28 COM 19

#include "WiFi.h"
#include "esp_now.h"

uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x37, 0xF8, 0xC8};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  // char a[32];
  uint16_t b;
  // float c;
  // bool d;
} struct_message;

uint16_t i = 0;
int loop_size = 1000;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}




void setup(){
  Serial.begin(115200);
  Serial.println("Starting");
  WiFi.mode(WIFI_STA);
  Serial.println("MAC: " + WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0; // pick a channel
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
    
}

void loop() {



  // // Wait for message to be sent from serial monitor
  // while (Serial.available() == 0) {
  //   delay(1);
  // }

  // // Read message from serial monitor
  // String message = Serial.readStringUntil('\n');
  // Serial.print("Message received: ");
  // Serial.println(message);



  // Set values to send
  // strcpy(myData.a, "THIS IS A CHAR");
  // strcpy(myData.a, message.c_str());
  myData.b = i;
  i++;
  // myData.c = 1.2;
  // myData.d = false;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    // Serial.println("Sent with success");
  }
  else {
    // Serial.println("Error sending the data");
    Serial.println(result);
  }

  if (i == loop_size) {
    i = 0;
    delay(1000);
  }
  // delay(2000);
  delay(2);
  // delayMicroseconds(1500);
}
    

