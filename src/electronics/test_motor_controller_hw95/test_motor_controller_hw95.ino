#define in1 6
#define in2 7
#define enA 9

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);

  Serial.begin(9600);
  Serial.println("Begin");
}

void loop() {
  // Set initial rotation direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 255);
}