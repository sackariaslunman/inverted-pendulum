#include <Arduino.h>
#include <Servo.h>

Servo esc; // create a Servo object
int pwm_duty_min = 800; // set PWM duty cycle for full reverse (in microseconds)
int pwm_duty_max = 2200; // set PWM duty cycle for full forward (in microseconds)
int pwm_duty_neutral = 1500; // set PWM duty cycle for neutral/stop (in microseconds)

void setup() {
  esc.attach(9); // attach ESC signal to pin 9
  esc.writeMicroseconds(pwm_duty_neutral); // set to neutral/stop position
}

void loop() {
  // drive motor forward at 50% speed for 3 seconds
  int pwm_duty = map(90, 0, 100, pwm_duty_neutral, pwm_duty_max); // calculate duty cycle value from percentage
  esc.writeMicroseconds(pwm_duty); // set ESC signal to desired duty cycle
  delay(500);

  // drive motor backward at 50% speed for 3 seconds
  pwm_duty = map(10, 0, 100, pwm_duty_min, pwm_duty_neutral); // calculate duty cycle value from percentage
  esc.writeMicroseconds(pwm_duty); // set ESC signal to desired duty cycle
  delay(500);

  // stop motor for 1 second
  esc.writeMicroseconds(pwm_duty_neutral); // set to neutral/stop position
  delay(1000);
}