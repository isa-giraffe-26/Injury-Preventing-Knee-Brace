#include <I2Cdev.h>
#include "MPU6050.h"
#include <Wire.h>


// MPU6050 instance
MPU6050 accelgyro;


// Accelerometer scale for ±8g (4096 LSB/g)
const float scale = 4096.0;


// Pins
#define SWITCH_PIN 7
#define HAPTIC_PIN 9


// Accelerometer data
int16_t ax, ay;


bool isAccelerometerActive = false;


void setup() {
    Serial.begin(19200);  // Make sure Serial Monitor matches this baud rate
    Wire.begin();


    pinMode(SWITCH_PIN, INPUT_PULLUP);
    pinMode(HAPTIC_PIN, OUTPUT);


    // Initialize before testing connection
    accelgyro.initialize();


    Serial.println("Testing accelerometer connection...");
    if (accelgyro.testConnection()) {
        Serial.println("MPU6050 connection successful");
        // Set range to ±8g
        accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    } else {
        Serial.println("MPU6050 connection failed");
        return;
    }
}


void loop() {
    // Handle accelerometer activation via switch
    if (digitalRead(SWITCH_PIN) == LOW && !isAccelerometerActive) {
        accelgyro.setSleepEnabled(false);  // Wake up sensor
        isAccelerometerActive = true;
    } else if (digitalRead(SWITCH_PIN) == HIGH && isAccelerometerActive) {
        accelgyro.setSleepEnabled(true);   // Sleep mode
        isAccelerometerActive = false;
        digitalWrite(HAPTIC_PIN, LOW);     // Turn off haptic
    }


    // Read and respond to X/Y-axis acceleration
    if (isAccelerometerActive) {
        accelgyro.getAcceleration(&ax, &ay, nullptr);  // Read X and Y axes
        float ax_in_g = (float)ax / scale;
        float ay_in_g = (float)ay / scale;

        
        if (ay_in_g <= -4.0) {
            Serial.print(millis());
            Serial.print("Y Acceleration: ");
            Serial.print(ay_in_g);
            Serial.println(" G");


            Serial.print(millis());
            Serial.print("X Acceleration: ");
            Serial.print(ax_in_g);
            Serial.println(" G");
        }

        
        if (abs(ay_in_g) >= 4.0) {
            digitalWrite(HAPTIC_PIN, HIGH);
        } else {
            digitalWrite(HAPTIC_PIN, LOW);
        }
    }


    delay(15); // Sampling interval
}


