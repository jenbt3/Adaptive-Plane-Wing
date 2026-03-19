#include <ESP32Servo.h>
#include <math.h>
#include <Wire.h> //I2C library for MPU6050
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include "camber_position.h"

//pinout constants
const int SERVO_PINS[] = {16, 17, 18};
const int NUM_SERVOS   = sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]);
const int SDA_PIN      = 4;
const int SCL_PIN      = 5;

//servo constants
const int NEUTRAL_POS  = 90;
const int MAX_HERTZ    = 50;

//filter constant
const float ALPHA      = 0.98;

//global onjects
Servo wingServo;
Adafruit_MPU6050 mpu;

//global variables
float pitch             = 0; // = AoA
float calibrationOffset = 0; // offset for flat position
unsigned long lastTime  = 0; // time in loop


void setup() {
    Serial.begin(115200); //start serial for debugging
    delay(2000);          //wait 2 seconds for serial to initialize
    Serial.println("Booting...");

    Wire.begin(SDA_PIN, SCL_PIN);//initialize I2C and MPU6050

    //initialize servo
    for (int i = 0; i < NUM_SERVOS; i++) {
        wingServo.attach(SERVO_PINS[i]); //attach servo to pin
        wingServo.setPeriodHertz(MAX_HERTZ); //set PWM frequency for servo
        wingServo.write(NEUTRAL_POS);
    }

    //initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("MPU6050 not found...");
        while (1);
    }

    // set sensor ranges
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G); //accelerometer range = +/-2g = better sensitivity
    mpu.setGyroRange(MPU6050_RANGE_250_DEG); //gyro range = +/-250 deg/s = better sensitivity
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    //calibrate at flat position; takes 200 readings in 1s
    Serial.println("Calibrating: keep wing flat...");
    float offsetSum = 0;
    for (int i = 0; i < 200; i++) {
        sensors_event_t a, g, temp; //create event objects for accelerometer, gyro, and temperature
        mpu.getEvent(&a, &g, &temp); //read sensor data into events
        offsetSum += atan2(a.acceleration.x, a.acceleration.y) * 180.0 / M_PI; //calculate angle from accelerometer and add to sum
        delay(5); //wait 5ms between readings
    }
    calibrationOffset = offsetSum / 200.0; //calculate average offset
    pitch = 0; //reset pitch to 0 after calibration

    Serial.print("Calibration offset: ");
    Serial.println(calibrationOffset, 2); //calibration offset with 2 decimal places

    lastTime = millis(); //record start time
}


void loop() {
    sensors_event_t a, g, temp; //create event objects
    mpu.getEvent(&a, &g, &temp); //read sensor data

    unsigned long currentTime = millis(); //get current time
    float dt = (currentTime - lastTime) / 1000.0; //calculate how long last loop took in seconds
    lastTime = currentTime; //update last time

    //calculate pitch (angle of attack) from accelerometer data
    float accelAngle = atan2(a.acceleration.x, a.acceleration.y) * 180.0 / M_PI; //convert to degrees
    pitch = ALPHA * (pitch + g.gyro.z * dt) + (1 - ALPHA) * accelAngle; //complementary filter to combine gyro and accelerometer data
    pitch -= calibrationOffset; //subtract calibration offset

}






//FUNCTIONS---------------------

//returns servo position based on angle of attack using lookup table
int getServoPosition(float angle) {
    int roundedAngle = (int)round(angle);
    for (int i = 0; i < NUM_SHAPES; i++) {
        if (lookupTable[i].angleOfAttack == roundedAngle) {
            return lookupTable[i].servoPosition;
        }
    }

    //if angle is outside of lookup table range, return closest position
    if (angle <= lookupTable[0].angleOfAttack)
        return lookupTable[0].servoPosition;

    if (angle >= lookupTable[NUM_SHAPES - 1].angleOfAttack)
        return lookupTable[NUM_SHAPES - 1].servoPosition;

    return NEUTRAL_POS; //default to neutral if something goes wrong
}

//write the same position to all servos
void writeAllServos(int position) {
    for (int i = 0; i < NUM_SERVOS; i++) {
        wingServo.write(position);
    }
}