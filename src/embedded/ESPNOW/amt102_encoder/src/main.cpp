#include <Arduino.h>

// Print debug messages
#define DEBUG 1

// Pin definitions
#define LED_PIN 5

// Encoder
#define ENCODER_PIN_A 21
#define ENCODER_PIN_B 22
#define ENCODER_PIN_X 19

// Global variables
volatile byte encoderAState = 0;
volatile byte encoderBState = 0;
volatile byte encoderXState = 0;

volatile byte encoderAStateOld = 0;
volatile byte encoderBStateOld = 0;
volatile byte encoderXStateOld = 0;

volatile int encoderPosition = 0;

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

// ESPNOW
#include "WiFi.h"
#include "esp_now.h"


// Ny esp med headers : 58:BF:25:38:02:FC
// uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x37, 0xF8, 0xC8};
uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x38, 0x02, 0xFC};

typedef struct struct_message {
  // char a[32];
  uint16_t b;
  // float c;
  // bool d;
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

}

void loop() {
  // put your main code here, to run repeatedly:

  // encoderXState = digitalRead(ENCODER_PIN_X);

  // digitalWrite(LED_PIN, encoderXState);


  // Read serial for time sync
  if (startMillis == 0){
    if (Serial.available() > 0) {
      startMillis = millis();    
      // Serial.read();
      if (Serial.read() == 't') {
        
      }
    }
  }
  myData.b = encoderPosition % 4096;
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


  // Serial.print("\t");
  // Serial.print(calculateSpeed());
  // Print arry
  // for (int i = 0; i < ENCODER_BUFFER_SIZE; i++){
  //   Serial.print("\t");
  //   Serial.print(readPosBuffer(i));
  // }
  Serial.println();

  delay(5);

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
}

void encoderXCall(){
  encoderXStateOld = encoderXState;
  encoderXState = digitalRead(ENCODER_PIN_X);
  if (encoderXState == HIGH){
    encoderPosition = 0;
  }
  updateBuffer();
}

void encoderAUpdate(){
  if (encoderAState == encoderBState){
    encoderPosition++; // CW
  } 
  else{
    encoderPosition--; // CCW
  }
  updateBuffer();
}

void encoderBUpdate(){
  if (encoderBState == encoderAState){
    encoderPosition--; // CW
  } 
  else{
    encoderPosition++; // CCW
  }
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