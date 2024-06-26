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


// Endstop
#define NEG_ENDSTOP_PIN 33
#define POS_ENDSTOP_PIN 32

// Global variables
volatile bool encoderAState = 0;
volatile bool encoderBState = 0;
volatile bool encoderXState = 0;

volatile bool encoderAStateOld = 0;
volatile bool encoderBStateOld = 0;
volatile bool encoderXStateOld = 0;

volatile int encoderPosition = 0;
// int encoderPositionOld = 0;

double encoderRad = 0;
double encoderRadOld = 0;
double encoderVel = 0;

unsigned long lastLoopTime = 0;

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

void encoderUpdate();
void encoderAUpdate();
void encoderAUpdateOld();
void encoderBUpdate();

void writePosBuffer(int pos);
int readPosBuffer(int index);
void updateBuffer();

void endstopNegCall();
void endstopPosCall();

// ESPNOW
#include "WiFi.h"
#include "esp_now.h"


// Ny esp med headers : 58:BF:25:38:02:FC
// Esp stepper encoder : 58:BF:25:37:F8:C8
// Ny test esp utan headers : C0:49:EF:F0:93:34
uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x38, 0x02, 0xFC};
// uint8_t broadcastAddress[] = {0xC0, 0x49, 0xEF, 0xF0, 0x93, 0x34};


byte sender_ID = 0;
const bool REVERSE_ENCODER = false;


typedef struct struct_message {
  double encoder_pos;
  double encoder_velocity;
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

unsigned long last_changed_time = 0;


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
  delay(5000);
}

int loop_encoder_position = 0;

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
  
  int dt = micros() - lastLoopTime;

  encoderRad = ((double)encoderPosition) * 2 * PI / 4096.0 - PI;

  double encoderVel = atan2(sin(encoderRad - encoderRadOld), cos(encoderRad - encoderRadOld)) / dt * 1000000;

  lastLoopTime = micros();
  
  
  myData.encoder_pos = encoderRad;
  myData.encoder_velocity = encoderVel;
  if (enableEspNow) {
    result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }
  else
  {
    result = 0;
  }
  
  Serial.print("Pos: ");
  // Serial.print(encoderPosition);
  Serial.print(encoderRad, 5);
  Serial.print("\tOld: ");
  Serial.print(encoderRadOld, 5);
  Serial.print("\tVel: ");
  Serial.print(encoderVel, 5);
  Serial.print("\tTime: ");
  Serial.print(millis() - startMillis); 
  Serial.print("\t dt: ");
  Serial.print(dt);

  Serial.print("\t");

  encoderRadOld = encoderRad;


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

  // Blink led if millis modulo 1000 is less than 500
  if ((millis() % 1000) < 500)
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }

  // if (((loop_encoder_position-encoderPosition)+4096)%4096 > 2 || ((encoderPosition-loop_encoder_position)+4096)%4096 > 2) {
  //   last_changed_time = micros();
  // }

  // // If pos hasn't changed for 5 seconds
  // if (micros() > last_changed_time + 5*1000*1000) {
  //   if (encoderPosition < 10 || encoderPosition > 4096-10) {
  //     encoderPosition = 0;
  //   }
  // }
  // loop_encoder_position = encoderPosition;
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

// Denna funkar nog inte helt korrekt? Behöver lägga till något med nuvarande posBufferIndex??
int readPosBuffer(int index){
  index = (ENCODER_BUFFER_SIZE + index + posBufferIndex) % ENCODER_BUFFER_SIZE;
  int value = posBuffer[index];
  return value;
}

void updateBuffer(){
  if (encoderPosition < 0){
    encoderPosition = encoderPosition + 4096;
  }
  encoderPosition = encoderPosition % 4096;
  last_changed_time = micros();
  // writePosBuffer(encoderPosition);
}


// Encoder functions
void encoderACall(){
  encoderAStateOld = encoderAState;
  encoderAState = digitalRead(ENCODER_PIN_A);
  encoderAUpdateOld();
}

void encoderBCall(){
  encoderBStateOld = encoderBState;
  encoderBState = digitalRead(ENCODER_PIN_B);
}

void encoderXCall(){
  // encoderXStateOld = encoderXState;
  // encoderXState = digitalRead(ENCODER_PIN_X);
  // if (encoderXState == HIGH){
  //   // Reset encoder to nearest multiple of 4096
  //   encoderPosition = static_cast<int>(std::round(encoderPosition / 4096.0)) * 4096;
  // }
  // updateBuffer();
}

void encoderAUpdate(){
  int encoderChange = 0;

  if (encoderAState != encoderAStateOld) {
    if (encoderBState != encoderAState) {
      encoderChange++;
    } else {
      encoderChange--;
    }
  }
  if (REVERSE_ENCODER) {
    encoderChange = -encoderChange;
  }
  encoderPosition += encoderChange;
  updateBuffer();
}

// Old code, maybe works?
void encoderAUpdateOld(){
  if (REVERSE_ENCODER) {
    if (encoderAState == encoderBState){
      encoderPosition++; // CW
    } 
    else{
      encoderPosition--; // CCW
    }
  }
  else {
    if (encoderAState == encoderBState){
      encoderPosition--; // CW
    } 
    else{
      encoderPosition++; // CCW
    }
  }
  updateBuffer();
}

void encoderBUpdate(){
  if (REVERSE_ENCODER) {
    if (encoderBState == encoderAState){
      encoderPosition--; // CW
    } 
    else{
      encoderPosition++; // CCW
    }
  }
  else {
    if (encoderBState == encoderAState){
      encoderPosition++; // CW
    } 
    else{
      encoderPosition--; // CCW
    }
  }
  updateBuffer();
}


// Endstop functions
void endstopNegCall(){
  // endstopNegState = digitalRead(NEG_ENDSTOP_PIN);
  // if (endstopNegState == HIGH){
  //   encoderPosition = 0;
  // }
}

void endstopPosCall(){
//   endstopPosState = digitalRead(POS_ENDSTOP_PIN);
//   if (endstopPosState == HIGH){
//     encoderPosition = 15860;
//   }
}