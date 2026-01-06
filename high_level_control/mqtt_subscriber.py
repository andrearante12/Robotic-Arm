import paho.mqtt.client as mqtt
import json
from datetime import datetime

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "esp32/mpu6050"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(MQTT_TOPIC)
        print(f"Subscribed to topic: {MQTT_TOPIC}")
    else:
        print(f"Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Received data:")
        print(f"  Acceleration: X={data['acceleration']['x']:.2f}, "
              f"Y={data['acceleration']['y']:.2f}, "
              f"Z={data['acceleration']['z']:.2f} m/s²")
        print(f"  Gyro:         X={data['gyro']['x']:.2f}, "
              f"Y={data['gyro']['y']:.2f}, "
              f"Z={data['gyro']['z']:.2f} rad/s")
        print(f"  Angle:        X={data['angle']['x']:.2f}°, "
              f"Y={data['angle']['y']:.2f}°, "
              f"Z={data['angle']['z']:.2f}°")
        print(f"  Temperature:  {data['temperature']:.2f}°C")
            
    except Exception as e:
        print(f"Error: {e}")
        print(f"Raw message: {msg.payload.decode()}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print("Connecting to MQTT broker...")
client.connect(MQTT_BROKER, MQTT_PORT, 60)

print("Listening for messages... (Press Ctrl+C to exit)")
try:
    client.loop_forever()
except KeyboardInterrupt:
    print("\nDisconnecting...")
    client.disconnect()
