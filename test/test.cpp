#include <Arduino.h>
#include <ESP32Servo.h>

Servo servo;
int servopin = 0; // change pin
int angle = 0;

void setup() {
  servo.attach(servopin);
  servo.write(90);
  Serial.begin(9600);
  Serial.println("enter angle: ");
}

void loop() {
  if (Serial.available()) {
    int angle = Serial.parseInt();
    servo.write(angle);
    delay (20) ;
  }
}