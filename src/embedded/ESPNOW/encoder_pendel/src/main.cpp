#include <Arduino.h>

// Print debug messages
#define DEBUG 1

// Pin definitions
#define LED_PIN 5
bool ledState = 0;

// Encoder
#define ENCODER_PIN_A 21
#define ENCODER_PIN_B 22
#define ENCODER_PIN_X 19

#define REVERSE_ENCODER 1

// Endstop
#define NEG_ENDSTOP_PIN 33
#define POS_ENDSTOP_PIN 32

// Global variables
volatile byte encoderAState = 0;
volatile byte encoderBState = 0;
volatile byte encoderXState = 0;

volatile byte encoderAStateOld = 0;
volatile byte encoderBStateOld = 0;
volatile byte encoderXStateOld = 0;

volatile int encoderPosition = 0;

volatile bool endstopNegState = 0;
volatile bool endstopPosState = 0;

// 4 value long list to store last 4 encoder positions. Circular buffer
#define ENCODER_BUFFER_SIZE 4
int posBuffer[ENCODER_BUFFER_SIZE] = {0, 0, 0, 0};
int posBufferIndex = 0;


// Function declarations
void encoderACall();
void encoderBCall();
void encoderXCall();

void encoderAUpdate();
void encoderBUpdate();

void writePosBuffer(int pos);
int readPosBuffer(int index);
void updateBuffer();

float calculateSpeed();

void endstopNegCall();
void endstopPosCall();

// ESPNOW
#include "WiFi.h"
#include "esp_now.h"


// Ny esp med headers : 58:BF:25:38:02:FC
// Esp stepper encoder : 58:BF:25:37:F8:C8
// Ny test esp utan headers : C0:49:EF:F0:93:34
// uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x38, 0x02, 0xFC};
uint8_t broadcastAddress[] = {0xC0, 0x49, 0xEF, 0xF0, 0x93, 0x34};
byte sender_ID = 0;

typedef struct struct_message {
  int16_t encoder_pos;
  byte id;
} struct_message;

// Create a struct_message called myData
struct_message myData;
esp_now_peer_info_t peerInfo;

esp_err_t result;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Time variables
unsigned long startMillis = 0;

bool enableEspNow = true;


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(115200);
  

  // Encoder
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);
  pinMode(ENCODER_PIN_X, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderACall, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoderBCall, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_X), encoderXCall, CHANGE);
  // Serial.println("Encoder setup done");

  // Endstop
  pinMode(NEG_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(POS_ENDSTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(NEG_ENDSTOP_PIN), endstopNegCall, CHANGE);
  attachInterrupt(digitalPinToInterrupt(POS_ENDSTOP_PIN), endstopPosCall, CHANGE);


  // ESPNOW
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

  // Set id
  myData.id = sender_ID;

}

void loop() {
  // put your main code here, to run repeatedly:

  // Read serial for time sync
  if (startMillis == 0){
    if (Serial.available() > 0) {
      startMillis = millis();    
      // Serial.read();
      if (Serial.read() == 't') {
      }
    }
  }
  
  myData.encoder_pos = encoderPosition % 4096;
  if (enableEspNow) {
    result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }
  else
  {
    result = 0;
  }
  
  Serial.print(encoderPosition);
  Serial.print("\t");
  Serial.print(millis() - startMillis);  
  Serial.print("\t");

  // Send message via ESP-NOW
  if (result == ESP_OK) {
    // Serial.println("Sent with success");
  }
  else {
    // Serial.println("Error sending the data");
    Serial.print(result);
  }

  Serial.println();

  delay(5);

  // Blink led if millis modelu 1000 is less than 500
  if ((millis() % 1000) < 500)
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }

}




// put function definitions here:
// Ring buffer functions
void writePosBuffer(int value){
  posBuffer[posBufferIndex] = value;
  posBufferIndex++;
  if (posBufferIndex >= ENCODER_BUFFER_SIZE){
    posBufferIndex = 0;
  }
}

// Denna funkar nog inte helt korrekt? Behöver lägga till något med nuvarnade posBufferIndex??
int readPosBuffer(int index){
  index = (ENCODER_BUFFER_SIZE + index + posBufferIndex) % ENCODER_BUFFER_SIZE;
  int value = posBuffer[index];
  return value;
}

void updateBuffer(){
  writePosBuffer(encoderPosition);
}


// Encoder functions
void encoderACall(){
  encoderAStateOld = encoderAState;
  encoderAState = digitalRead(ENCODER_PIN_A);
  encoderAUpdate();
}

void encoderBCall(){
  encoderBStateOld = encoderBState;
  encoderBState = digitalRead(ENCODER_PIN_B);
  // encoderUpdate();
  // encoderBUpdate();
}

void encoderXCall(){
  encoderXStateOld = encoderXState;
  encoderXState = digitalRead(ENCODER_PIN_X);
  if (encoderXState == HIGH){
    // Reset encoder to nearest multiple of 4096
    encoderPosition = static_cast<int>(std::round(encoderPosition / 4096.0)) * 4096;
  }
  updateBuffer();
}

void encoderAUpdate(){
  #ifndef REVERSE_ENCODER
    if (encoderAState == encoderBState){
      encoderPosition++; // CW
    } 
    else{
      encoderPosition--; // CCW
    }
  #else
    if (encoderAState == encoderBState){
      encoderPosition--; // CW
    } 
    else{
      encoderPosition++; // CCW
    }
  #endif
  updateBuffer();
}

void encoderBUpdate(){
  #ifndef REVERSE_ENCODER
    if (encoderBState == encoderAState){
      encoderPosition--; // CW
    } 
    else{
      encoderPosition++; // CCW
    }
  #else
    if (encoderBState == encoderAState){
      encoderPosition++; // CW
    } 
    else{
      encoderPosition--; // CCW
    }
  #endif
  updateBuffer();
}


float calculateSpeed(){
  int pos1 = readPosBuffer(0);
  int pos2 = readPosBuffer(-1);
  int pos3 = readPosBuffer(-2);
  int pos4 = readPosBuffer(-3);

  float dt = 0.001;

  float speed = (-2*pos4 + 9*pos3 - 18*pos2 + 11*pos1)/(6*dt);

  return speed;

}


// Endstop functions
void endstopNegCall(){
  endstopNegState = digitalRead(NEG_ENDSTOP_PIN);
  if (endstopNegState == HIGH){
    encoderPosition = 0;
  }
}

void endstopPosCall(){
  endstopPosState = digitalRead(POS_ENDSTOP_PIN);
  if (endstopPosState == HIGH){
    encoderPosition = 15860;
  }
}