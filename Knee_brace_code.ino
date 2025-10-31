#include <I2Cdev.h>
#include "MPU6050.h"


// MPU6050 instance
MPU6050 accelgyro;


// Scale for accelerometer (default sensitivity for ±2g = 16384)
int scale = 16384;


// Accelerometer data variables
int16_t ax, ay, az;


// Pins
#define LED_PIN 13          // Pin to blink the LED
#define SWITCH_PIN 7        // Pin to control accelerometer on/off
#define HAPTIC_PIN 9        // Pin to control haptic motor (vibration motor)


bool blinkState = false;
bool isAccelerometerActive = false;


void setup() {
    // Initialize serial communication
    Serial.begin(9600);


    // Initialize pins
    pinMode(LED_PIN, OUTPUT);
    pinMode(SWITCH_PIN, INPUT_PULLUP); // Use internal pull-up resistor
    pinMode(HAPTIC_PIN, OUTPUT);       // Pin for controlling haptic motor


    // Start with accelerometer inactive
    isAccelerometerActive = false;


    // Verify connection to the MPU6050
    Serial.println("Testing accelerometer connection...");
    if (accelgyro.testConnection()) {
        Serial.println("MPU6050 connection successful");
    } else {
        Serial.println("MPU6050 connection failed");
        return;
    }
}


void loop() {
    // Check switch state
    if (digitalRead(SWITCH_PIN) == LOW) {
        // Switch pressed, activate the accelerometer
        if (!isAccelerometerActive) {
            Serial.println("Activating accelerometer...");
            accelgyro.initialize();
            accelgyro.setSleepEnabled(false); // Wake up the accelerometer
            isAccelerometerActive = true;
        }
    } else {
        // Switch not pressed, deactivate the accelerometer
        if (isAccelerometerActive) {
            Serial.println("Deactivating accelerometer...");
            accelgyro.setSleepEnabled(true);  // Put the accelerometer to sleep
            isAccelerometerActive = false;
        }
    }


    if (isAccelerometerActive) {
        // Read raw accelerometer measurements
        accelgyro.getAcceleration(&ax, &ay, &az);


        // Print raw accelerometer values for debugging
        Serial.print("Raw Accelerometer Values: X = "); Serial.print(ax);
        Serial.print("\tY = "); Serial.print(ay);
        Serial.print("\tZ = "); Serial.println(az);


        // Convert to Gs
        float ax_in_g = (float)ax / scale;
        float ay_in_g = (float)ay / scale;
       
        // Print the G values for debugging
        Serial.print("In G's: X = "); Serial.print(ax_in_g);
        Serial.print("\tY = "); Serial.println(ay_in_g);


        // Trigger the haptic motor if X < -1 and Y > 1 (with some tolerance)
        if (ax_in_g < -0.50 && ay_in_g > 1.00) {
            digitalWrite(HAPTIC_PIN, HIGH);  // Turn on haptic motor
            Serial.println("Haptic ON!");    // Debug message
        } else {
            digitalWrite(HAPTIC_PIN, LOW);   // Turn off haptic motor
            Serial.println("Haptic OFF");    // Debug message
        }


        // Blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    } else {
        // If accelerometer is off, turn off haptic motor and LED
        digitalWrite(HAPTIC_PIN, LOW);   // Ensure haptic motor is off
        digitalWrite(LED_PIN, LOW);      // Ensure LED is off
    }


    // Delay to control the logging frequency (adjust as needed)
    delay(500); // 500 ms delay between readings
}

    // Delay to control the logging frequency (adjust as needed)
    delay(500); // 500 ms delay between readings
}

    
