#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>

Adafruit_MPU6050 mpu;

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Raspberry Pi IP address
const char* serverName = "http://192.168.1.XXX:5000/data";

float angleX = 0.0, angleY = 0.0, angleZ = 0.0;
unsigned long lastTime = 0;

void setup(void) {
  Serial.begin(115200);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate angles
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float wx = g.gyro.x + 0.03;
  float wy = g.gyro.y - 0.02;
  float wz = g.gyro.z - 0.01;

  angleX += wx * dt;
  angleY += wy * dt;
  angleZ += wz * dt;

  // Send data to Raspberry Pi
  if(WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");
    
    // Create JSON payload
    String jsonData = "{";
    jsonData += "\"accel_x\":" + String(a.acceleration.x) + ",";
    jsonData += "\"accel_y\":" + String(a.acceleration.y) + ",";
    jsonData += "\"accel_z\":" + String(a.acceleration.z) + ",";
    jsonData += "\"gyro_x\":" + String(wx) + ",";
    jsonData += "\"gyro_y\":" + String(wy) + ",";
    jsonData += "\"gyro_z\":" + String(wz) + ",";
    jsonData += "\"angle_x\":" + String(angleX * 180.0 / PI) + ",";
    jsonData += "\"angle_y\":" + String(angleY * 180.0 / PI) + ",";
    jsonData += "\"angle_z\":" + String(angleZ * 180.0 / PI);
    jsonData += "}";
    
    int httpResponseCode = http.POST(jsonData);
    
    if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  }
  
  delay(500);
}