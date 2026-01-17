/*
 * Dual Sensor MQTT Publisher - OTOS + MPU6050
 * 
 * Reads from both OTOS position sensor and MPU6050 IMU
 * Publishes combined data via MQTT
 * 
 * I2C Bus (GPIO 21/22):
 * - OTOS (PAA5160E1): 0x17 - Position (X, Y, Heading)
 * - MPU6050: 0x68 - Orientation (Roll, Pitch, Accel, Gyro)
 */

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Sensor objects
QwiicOTOS myOtos;
Adafruit_MPU6050 mpu;

// WiFi credentials
const char* ssid = "Verizon_D4SHQL";
const char* password = "hurl-ranch9-raw";

// MQTT Broker
const char* mqtt_server = "192.168.1.224";
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32/dual_sensors";

WiFiClient espClient;
PubSubClient client(espClient);

// Calibration offsets for MPU6050
float gyro_offset_x = 0.0;
float gyro_offset_y = 0.0;
float gyro_offset_z = 0.0;

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("======================================");
    Serial.println("Dual Sensor MQTT Publisher");
    Serial.println("======================================");
    Serial.println();
    
    // Initialize I2C
    Wire.begin(21, 22);
    delay(100);
    
    // Scan I2C bus
    Serial.println("Scanning I2C bus...");
    scanI2C();
    Serial.println();
    
    // Initialize OTOS
    Serial.println("Initializing OTOS...");
    if (!myOtos.begin()) {
        Serial.println("❌ OTOS not found!");
        while(1) delay(1000);
    }
    Serial.println("✓ OTOS initialized!");
    
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
        Serial.println("❌ MPU6050 not found!");
        while(1) delay(1000);
    }
    Serial.println("✓ MPU6050 initialized!");
    
    // Configure MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("  MPU6050 configured!");
    
    // Calibrate MPU6050 gyroscope
    Serial.println("  Calibrating MPU6050 gyro (keep still)...");
    calibrateGyro();
    Serial.println("  MPU6050 calibration complete!");
    Serial.println();
    
    // Connect to WiFi
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("✓ WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();
    
    // Setup MQTT with larger buffer
    client.setServer(mqtt_server, mqtt_port);
    client.setBufferSize(512);  // Increase MQTT buffer size
    
    Serial.println("======================================");
    Serial.println("System ready! Publishing data...");
    Serial.println("======================================");
    Serial.println();
}

void loop() {
    // Maintain MQTT connection
    if (!client.connected()) {
        reconnectMQTT();
    }
    client.loop();
    
    // Read from OTOS
    sfe_otos_pose2d_t otos_pos;
    myOtos.getPosition(otos_pos);
    
    // Read from MPU6050
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    
    // Apply gyro calibration
    float gyro_x = gyro.gyro.x - gyro_offset_x;
    float gyro_y = gyro.gyro.y - gyro_offset_y;
    float gyro_z = gyro.gyro.z - gyro_offset_z;
    
    // Calculate roll and pitch from accelerometer
    float roll = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
    float pitch = atan2(-accel.acceleration.x, 
                        sqrt(accel.acceleration.y * accel.acceleration.y + 
                             accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;
    
    // Create JSON document (larger size)
    StaticJsonDocument<512> doc;
    
    // OTOS position data
    JsonObject otos = doc.createNestedObject("otos");
    otos["x"] = round(otos_pos.x * 100) / 100.0;      // 2 decimal places
    otos["y"] = round(otos_pos.y * 100) / 100.0;
    otos["h"] = round(otos_pos.h * 10) / 10.0;        // 1 decimal place
    
    // Orientation (roll, pitch from IMU, yaw from OTOS)
    JsonObject orient = doc.createNestedObject("orient");
    orient["r"] = round(roll * 10) / 10.0;            // 1 decimal place
    orient["p"] = round(pitch * 10) / 10.0;
    orient["y"] = round(otos_pos.h * 10) / 10.0;
    
    // MPU6050 accelerometer (optional - remove if still too large)
    JsonObject acc = doc.createNestedObject("accel");
    acc["x"] = round(accel.acceleration.x * 100) / 100.0;
    acc["y"] = round(accel.acceleration.y * 100) / 100.0;
    acc["z"] = round(accel.acceleration.z * 100) / 100.0;
    
    // MPU6050 gyroscope (optional - remove if still too large)
    JsonObject gyr = doc.createNestedObject("gyro");
    gyr["x"] = round(gyro_x * 100) / 100.0;
    gyr["y"] = round(gyro_y * 100) / 100.0;
    gyr["z"] = round(gyro_z * 100) / 100.0;
    
    doc["ts"] = millis();
    
    // Serialize to JSON string
    char jsonBuffer[512];
    size_t len = serializeJson(doc, jsonBuffer);
    
    Serial.printf("JSON size: %d bytes\n", len);
    
    // Publish to MQTT
    if (client.publish(mqtt_topic, jsonBuffer)) {
        Serial.println("✓ Published:");
        Serial.println(jsonBuffer);
    } else {
        Serial.println("❌ Failed to publish");
        Serial.printf("Buffer size: %d, Message size: %d\n", client.getBufferSize(), len);
    }
    
    delay(100);  // 10Hz update rate
}

void calibrateGyro() {
    const int samples = 100;
    float sum_x = 0, sum_y = 0, sum_z = 0;
    
    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, t;
        mpu.getEvent(&a, &g, &t);
        sum_x += g.gyro.x;
        sum_y += g.gyro.y;
        sum_z += g.gyro.z;
        delay(10);
    }
    
    gyro_offset_x = sum_x / samples;
    gyro_offset_y = sum_y / samples;
    gyro_offset_z = sum_z / samples;
    
    Serial.printf("  Gyro offsets: X=%.4f Y=%.4f Z=%.4f\n", 
                  gyro_offset_x, gyro_offset_y, gyro_offset_z);
}

void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        
        String clientId = "ESP32_DualSensor-";
        clientId += String(random(0xffff), HEX);
        
        if (client.connect(clientId.c_str())) {
            Serial.println(" connected!");
        } else {
            Serial.print(" failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in 5 seconds");
            delay(5000);
        }
    }
}

void scanI2C() {
    byte error, address;
    int deviceCount = 0;
    
    Serial.println("  Address  Device");
    Serial.println("  -------  ------");
    
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.printf("  0x%02X     ", address);
            
            if (address == 0x17) {
                Serial.println("OTOS ✓");
            } else if (address == 0x68) {
                Serial.println("MPU6050 ✓");
            } else {
                Serial.println("Unknown");
            }
            
            deviceCount++;
        }
    }
    
    Serial.println("  -------  ------");
    Serial.printf("  Found %d device(s)\n", deviceCount);
}