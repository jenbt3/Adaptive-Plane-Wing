#include <ESP32Servo.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include "camber_position.h"

// Pinout constants
const int SERVO_PINS[] = {15, 16, 17, 18, 8 , 3};  //1, 2, 3, 4, 5, 6 respectively
const int NUM_SERVOS   = sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]);
const int SDA_PIN      = 4;
const int SCL_PIN      = 5;

// Servo constants
const int NEUTRAL_POS  = 90;
const int MAX_HERTZ    = 50;

const int SERVO_TRIMS[] = { 
    0,
    0,
    -5,
    0,
    0,
    -10
};


// Filter constant
const float ALPHA      = 0.98;

// Global objects
Servo wingServos[NUM_SERVOS];
Adafruit_MPU6050 mpu;

float pitch             = 0.0;
float calibrationOffset = 0.0;
unsigned long lastTime  = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("booting...");

  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize all servos
  for (int i = 0; i < NUM_SERVOS; i++) {
    wingServos[i].attach(SERVO_PINS[i]);
    wingServos[i].setPeriodHertz(MAX_HERTZ);
    wingServos[i].write(NEUTRAL_POS);
  }

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found...");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Calibrating — keep wing flat...");
  float offsetSum = 0.0;
  for (int i = 0; i < 200; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    offsetSum += atan2(a.acceleration.x, a.acceleration.y) * 180.0 / M_PI;
    delay(5);
  }

  calibrationOffset = offsetSum / 200.0;
  pitch = 0.0;

  Serial.print("Calibration offset: ");
  Serial.println(calibrationOffset, 2);

  lastTime = millis();
}

int getServoPosition(float angle) {
  int roundedAngle = (int)round(angle);
  for (int i = 0; i < NUM_SHAPES; i++) {
    if (lookupTable[i].angleOfAttack == roundedAngle) {
      return lookupTable[i].servoPosition;
    }
  }

  if (angle <= lookupTable[0].angleOfAttack)
    return lookupTable[0].servoPosition;

  if (angle >= lookupTable[NUM_SHAPES - 1].angleOfAttack)
    return lookupTable[NUM_SHAPES - 1].servoPosition;

    
  return 0;
}

//write the same position to all servos
void writeAllServos(int position) {
    for (int i = 0; i < NUM_SERVOS; i++) {
        int val = constrain(position + SERVO_TRIMS[i], 0, 180);
        wingServos[i].write(val);
    }
}



//loop------

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float accelPitch = atan2(a.acceleration.x, a.acceleration.y)
                     * 180.0 / M_PI - calibrationOffset;

  pitch = ALPHA * (pitch + g.gyro.z * dt * 180.0 / M_PI)
        + (1.0 - ALPHA) * accelPitch;

  int rawPosition    = getServoPosition(pitch);
  int targetPosition = constrain(rawPosition + NEUTRAL_POS, 0, 180);

  writeAllServos(targetPosition);

  Serial.print("AoA: ");
  Serial.print(pitch, 2);
  Serial.print(" deg  |  Servo: ");
  Serial.println(targetPosition);

  delay(10);
}