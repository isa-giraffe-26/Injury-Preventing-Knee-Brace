#include <I2Cdev.h>
#include "MPU6050.h"


// MPU6050 instance
MPU6050 accelgyro;


// Scale for accelerometer (default sensitivity for ±2g = 16384)
int scale = 16384;


// Accelerometer data variables
int16_t ax, ay, az;


#define LED_PIN 13
bool blinkState = false;


void setup() {
    // Initialize serial communication
    Serial.begin(9600);


    // Initialize the accelerometer (MPU6050)
    Serial.println("Initializing accelerometer...");
    accelgyro.initialize();
    accelgyro.setSleepEnabled(false); // Ensure the accelerometer is awake


    // Verify connection to the MPU6050
    Serial.println("Testing accelerometer connection...");
    if (accelgyro.testConnection()) {
        Serial.println("MPU6050 connection successful");
    } else {
        Serial.println("MPU6050 connection failed");
        return;
    }


    // Configure Arduino LED for activity indication
    pinMode(LED_PIN, OUTPUT);
}


void loop() {
    // Read raw accelerometer measurements
    accelgyro.getAcceleration(&ax, &ay, &az);


    // Display acceleration values (in g's) on Serial Monitor
    Serial.print("acceleration:\t");
    Serial.print((float)ax / scale); Serial.print("\t");
    Serial.print((float)ay / scale); Serial.print("\t");
    Serial.println((float)az / scale);


    // Blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);


    // Delay to control the logging frequency (adjust as needed)
    delay(500); // 500 ms delay between readings
}

    