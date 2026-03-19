#include <ESP32Servo.h>
#include "camber_position.h"

// setup
Servo wingServo;
const int SERVO_PIN = 0;  // change pin
const int neutralPosition = 90; // neutralservo position

void setup() {
  Serial.begin(9600);
  
  wingServo.attach(SERVO_PIN); //attach servo to pin
  
  Serial.println("neutral position");
  wingServo.write(neutralPosition);
  delay(1000);
}

void loop() {
  int i = 0; // index for lookup table
  
  // current angle and position from lookup table
  float currentAngle = lookupTable[i].angleOfAttack;
  int targetPosition = lookupTable[i].servoPosition;
  
  Serial.print("index: ");
  Serial.print(i);
  Serial.print(" | angle of Attack: ");
  Serial.print(currentAngle);
  Serial.print("degrees | servo Position: ");
  Serial.print(targetPosition);
  Serial.println("degrees");
  
  // move servo to target position
  wingServo.write(targetPosition);
  
  delay(2000);
  
  // move to next angle
  i = (i + 1) % NUM_SHAPES;
  
}











/*
// func to map angle to closest servo position
int mapAngleToServo(float angle) {
  // if angle is outside range
  if (angle <= lookupTable[0].angleOfAttack) {
    return lookupTable[0].servoPosition;
  }
  if (angle >= lookupTable[NUM_SHAPES - 1].angleOfAttack) {
    return lookupTable[NUM_SHAPES - 1].servoPosition;
  }
  
  // Find the two closest entries for interpolation
  for (int i = 0; i < NUM_SHAPES - 1; i++) {
    if (angle >= lookupTable[i].angleOfAttack && 
        angle <= lookupTable[i + 1].angleOfAttack) {
      
      // Linear interpolation
      float angleDiff = lookupTable[i + 1].angleOfAttack - lookupTable[i].angleOfAttack;
      float posDiff = lookupTable[i + 1].servoPosition - lookupTable[i].servoPosition;
      float ratio = (angle - lookupTable[i].angleOfAttack) / angleDiff;
      
      int interpolatedPos = lookupTable[i].servoPosition + (ratio * posDiff);
      return interpolatedPos;
    }
  }
      
  return neutralPosition; // Fallback
}

  */