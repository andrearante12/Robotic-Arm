#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 125  // Minimum pulse length (0 degrees)
#define SERVOMAX 625  // Maximum pulse length (180 degrees)

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  board1.begin();
  board1.setPWMFreq(60); // Servos operate at ~60 Hz
}

void reset() {
  // Servo 0 (Base)
  board1.setPWM(0, 0, angleToPulse(150));

  // Servo 1 (Bottom Arm)
  board1.setPWM(1, 0, angleToPulse(75));

//// Servo 2 (Mid Arm)
  board1.setPWM(2, 0, angleToPulse(180));

//// Servo 3 (Wrist)
  board1.setPWM(3, 0, angleToPulse(180));

// Servo 4 (Hand Rotation)
 board1.setPWM(4, 0, angleToPulse(0));

// Servo 5 (Grip)
 board1.setPWM(5, 0, angleToPulse(0));
}



void loop() {
  reset();
}

int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  Serial.print("Angle: "); Serial.print(ang);
  Serial.print(" pulse: "); Serial.println(pulse);
  return pulse;
}
