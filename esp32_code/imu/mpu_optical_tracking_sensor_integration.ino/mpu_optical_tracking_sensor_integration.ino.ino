/*
 * Dual Sensor Test - OTOS + MPU6050 on Same I2C Bus
 * 
 * This sketch tests reading from both the OTOS sensor and MPU6050 IMU
 * connected to the same I2C bus (GPIO 21/22 on ESP32)
 * 
 * Expected I2C Addresses:
 * - OTOS (PAA5160E1): 0x17 (23 decimal)
 * - MPU6050: 0x68 (104 decimal)
 * 
 * Upload this, open Serial Monitor at 115200 baud
 */

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

QwiicOTOS myOtos;
Adafruit_MPU6050 mpu;

void setup() {
    Serial.begin(115200);
    delay(2000);  // Wait for serial monitor
    
    Serial.println("======================================");
    Serial.println("Dual Sensor I2C Test");
    Serial.println("======================================");
    Serial.println();
    
    // Initialize I2C on default pins
    Wire.begin(21, 22);  // SDA=21, SCL=22
    delay(100);
    
    // Scan I2C bus
    Serial.println("Scanning I2C bus...");
    scanI2C();
    Serial.println();
    
    // Initialize OTOS
    Serial.println("Initializing OTOS...");
    if (!myOtos.begin()) {
        Serial.println("❌ OTOS not found! Check wiring.");
        Serial.println("   Expected at address 0x17");
        while(1) {
            delay(1000);
            Serial.println("   Retrying OTOS...");
            if (myOtos.begin()) {
                Serial.println("✓ OTOS found!");
                break;
            }
        }
    } else {
        Serial.println("✓ OTOS initialized successfully!");
    }
    
    // Calibrate OTOS
    Serial.println("  Calibrating OTOS (keep flat and still)...");
    delay(2000);
    myOtos.calibrateImu();
    myOtos.resetTracking();
    Serial.println("  OTOS calibration complete!");
    Serial.println();
    
    // Initialize MPU6050
    Serial.println("Initializing MPU6050...");
    if (!mpu.begin(0x68, &Wire)) {
        Serial.println("❌ MPU6050 not found! Check wiring.");
        Serial.println("   Expected at address 0x68");
        while(1) {
            delay(1000);
            Serial.println("   Retrying MPU6050...");
            if (mpu.begin(0x68, &Wire)) {
                Serial.println("✓ MPU6050 found!");
                break;
            }
        }
    } else {
        Serial.println("✓ MPU6050 initialized successfully!");
    }
    
    // Configure MPU6050
    Serial.println("  Configuring MPU6050...");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("  MPU6050 configuration complete!");
    Serial.println();
    
    Serial.println("======================================");
    Serial.println("Both sensors ready!");
    Serial.println("Starting continuous readings...");
    Serial.println("======================================");
    Serial.println();
    
    delay(1000);
}

void loop() {
    // Read from OTOS
    sfe_otos_pose2d_t otos_pos;
    myOtos.getPosition(otos_pos);
    
    // Read from MPU6050
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    
    // Calculate roll and pitch from accelerometer
    float roll = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
    float pitch = atan2(-accel.acceleration.x, 
                        sqrt(accel.acceleration.y * accel.acceleration.y + 
                             accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;
    
    // Print header every 20 readings
    static int count = 0;
    if (count % 20 == 0) {
        Serial.println();
        Serial.println("─────────────────────────────────────────────────────────────────────────────");
        Serial.println("OTOS Position (inches)       | MPU6050 Accel (m/s²)    | MPU6050 Gyro (rad/s)");
        Serial.println("─────────────────────────────────────────────────────────────────────────────");
    }
    count++;
    
    // Print data in tabular format
    Serial.printf("X:%6.2f Y:%6.2f H:%6.1f° | ", 
                  otos_pos.x, otos_pos.y, otos_pos.h);
    Serial.printf("X:%5.2f Y:%5.2f Z:%5.2f | ",
                  accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
    Serial.printf("X:%5.2f Y:%5.2f Z:%5.2f\n",
                  gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);