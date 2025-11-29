#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 125  // Minimum pulse length (0 degrees)
#define SERVOMAX 625  // Maximum pulse length (180 degrees)

void setup() {
// Serial communication initialization
  Serial.begin(9600);
  
  board1.begin();
  board1.setPWMFreq(60); // Servos operate at ~60 Hz

  // Reset servo to default position
  reset();
  
}


void loop() {  
  
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int servo, angle;
  
    if (parseServoCommand(input, servo, angle)) {
      board1.setPWM(servo, 0, angleToPulse(angle));
    
      Serial.print("Servo ");
      Serial.print(servo);
      Serial.print(" set to ");
      Serial.print(angle);
      Serial.println("°");
    } else {
      Serial.println("Invalid command. Format: servo3=45");
    }

  }
  
}

void reset() {
  board1.setPWM(0, 0, angleToPulse(150)); // Servo 0 (Base)
  board1.setPWM(1, 0, angleToPulse(75)); // Servo 1 (Bottom Arm)
  board1.setPWM(2, 0, angleToPulse(180)); // Servo 2 (Mid Arm)
  board1.setPWM(3, 0, angleToPulse(180)); // Servo 3 (Wrist)
  board1.setPWM(4, 0, angleToPulse(0)); // Servo 4 (Hand Rotation)
  board1.setPWM(5, 0, angleToPulse(45)); // Servo 5 (Grip) Range for grip is 45-90 degrees without overstraining servo
}

int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}


bool parseServoCommand(String cmd, int &servo, int &angle) {
  cmd.trim();

  // Must start with "servo"
  if (!cmd.startsWith("servo")) return false;

  // Must contain '='
  int eqIndex = cmd.indexOf('=');
  if (eqIndex == -1) return false;

  // Extract substring between "servo" and '=' → servo number
  String servoStr = cmd.substring(5, eqIndex);
  String angleStr = cmd.substring(eqIndex + 1);

  // Ensure both are digits only
  for (char c : servoStr) if (!isDigit(c)) return false;
  for (char c : angleStr) if (!isDigit(c)) return false;

  servo = servoStr.toInt();
  angle = angleStr.toInt();

  // Validate angle range
  if (angle < 0 || angle > 180) return false;

  // Additional angle range verification for servo 5 (claw) to prevent overstraining
  if (servo == 5 & (angle < 45 || angle > 90)) return false;

  return true;
}
