#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

Adafruit_MPU6050 mpu;

// WiFi credentials
const char* ssid = "Verizon_D4SHQL";
const char* password = "hurl-ranch9-raw";

// MQTT Broker (Raspberry Pi IP address)
const char* mqtt_server = "192.168.1.224";  
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32/mpu6050";

WiFiClient espClient;
PubSubClient client(espClient);

float angleX = 0.0, angleY = 0.0, angleZ = 0.0;
unsigned long lastTime = 0;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup(void) {
  Serial.begin(115200);
  
  // Connect to WiFi
  setup_wifi();
  
  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("Setup complete!");
  delay(100);
}

void loop() {
  // Maintain MQTT connection
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Get sensor data
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

  // Create JSON document
  StaticJsonDocument<300> doc;
  
  JsonObject accel = doc.createNestedObject("acceleration");
  accel["x"] = a.acceleration.x;
  accel["y"] = a.acceleration.y;
  accel["z"] = a.acceleration.z;
  
  JsonObject gyro = doc.createNestedObject("gyro");
  gyro["x"] = wx;
  gyro["y"] = wy;
  gyro["z"] = wz;
  
  JsonObject angle = doc.createNestedObject("angle");
  angle["x"] = angleX * 180.0 / PI;
  angle["y"] = angleY * 180.0 / PI;
  angle["z"] = angleZ * 180.0 / PI;
  
  doc["temperature"] = temp.temperature;
  doc["timestamp"] = millis();

  // Serialize JSON to string
  char jsonBuffer[300];
  serializeJson(doc, jsonBuffer);

  // Publish to MQTT
  if (client.publish(mqtt_topic, jsonBuffer)) {
    Serial.println("Data published:");
    Serial.println(jsonBuffer);
  } else {
    Serial.println("Failed to publish");
  }

  delay(500);
}