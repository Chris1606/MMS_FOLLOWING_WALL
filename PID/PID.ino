#include <Wire.h>

#define TOF_FRONT_ADDRESS 0x46  // Front sensor I2C address
#define TOF_LEFT_ADDRESS  0x4B  // Left sensor I2C address
#define TOF_RIGHT_ADDRESS 0x52  // Right sensor I2C address

// Define sensor registers (based on the TOF10120 sensor datasheet)
#define TOF_REG_DISTANCE 0x00  // Register to read distance data

#define INPUT_DISTANCE 150  // in mm (converted from cm to match TOF10120 units)
#define ERROR_DIST 50  // in mm (converted to mm)
#define SPEED 95
#define MAX_DIFFERNCE 10
#define MAX_TURN_SPEED 10
#define CALIBRATION 3
#define MAX_ALLIGN_ANGLE 5  // in mm
#define COLLISION_DISTANCE 100  // in mm

// Motor pins (same as before)
const int in1R = 7;
const int in2R = 8;
const int enR = 9;

const int in1L = 4;
const int in2L = 5;
const int enL = 3;

// PID
float kp_a = 1, kd_a = 0;
float kp_d = 1, kd_d = 0, ki_d = 0;

// Distance variables
float previousErrorD = 0, integral = 0;
float previousAngle = 0.0;
int speedL, speedR, distance;

// Function to read distance from a TOF sensor
uint16_t readTOFSensor(uint8_t sensorAddress) {
  uint16_t distance = 0;
  Wire.beginTransmission(sensorAddress);
  Wire.write(TOF_REG_DISTANCE);  // Request distance register
  Wire.endTransmission();

  Wire.requestFrom(sensorAddress, (uint8_t)2);  // Request 2 bytes
  if (Wire.available() == 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    distance = (highByte << 8) | lowByte;  // Combine into 16-bit value
  }
  return distance;
}

// Calculate current distance from two side sensors
float current_distance(uint16_t leftDistance, uint16_t rightDistance) {
  return (leftDistance + rightDistance) / 2.0;
}

// Determine which region the robot is in
int check_region(float leftDistance, float rightDistance) {
  float distance = current_distance(leftDistance, rightDistance);
  if (abs(distance - INPUT_DISTANCE) > ERROR_DIST) {
    return (distance > INPUT_DISTANCE) ? -1 : 1;  // Left or right movement
  }
  return 0;  // Inside region
}

// Control to reach the specified distance
void reach_distance(uint16_t leftDistance, uint16_t rightDistance) {
  float currentDistance = current_distance(leftDistance, rightDistance);
  float errorD = currentDistance - INPUT_DISTANCE;
  float derivative = errorD - previousErrorD;
  integral += errorD;
  float outputD = kp_d * errorD + ki_d * integral + kd_d * derivative;
  previousErrorD = errorD;

  speedL = SPEED - (int)outputD;
  speedR = SPEED + (int)outputD;

  if ((speedL - speedR) > MAX_DIFFERNCE) {
    speedL = SPEED + MAX_DIFFERNCE;
    speedR = SPEED - MAX_DIFFERNCE;
  } else if ((speedL - speedR) < -MAX_DIFFERNCE) {
    speedL = SPEED - MAX_DIFFERNCE;
    speedR = SPEED + MAX_DIFFERNCE;
  }

  analogWrite(enL, speedL);
  analogWrite(enR, speedR);
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
}

// Wall-following logic
void follow_wall(uint16_t leftDistance, uint16_t rightDistance) {
  float angle = rightDistance - leftDistance;  // +ve: turn left, -ve: turn right
  float derivativeA = angle - previousAngle;
  float outputA = kp_a * angle + kd_a * derivativeA;
  previousAngle = angle;

  speedL = SPEED + CALIBRATION - outputA;
  speedR = SPEED + outputA;

  if ((speedL - speedR) > MAX_TURN_SPEED) {
    speedL = SPEED + CALIBRATION + MAX_TURN_SPEED;
    speedR = SPEED - MAX_TURN_SPEED;
  } else if ((speedL - speedR) < -MAX_TURN_SPEED) {
    speedL = SPEED + CALIBRATION - MAX_TURN_SPEED;
    speedR = SPEED + MAX_TURN_SPEED;
  }

  analogWrite(enL, speedL);
  analogWrite(enR, speedR);
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
}

// Collision detection using the front TOF sensor
bool check_collision(uint16_t frontDistance) {
  return frontDistance < COLLISION_DISTANCE;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();  // Initialize I2C for the TOF sensors

  // Motor pin initialization
  pinMode(in1L, OUTPUT);
  pinMode(in2L, OUTPUT);
  pinMode(in1R, OUTPUT);
  pinMode(in2R, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
}

void loop() {
  // Read TOF sensor distances
  uint16_t frontDistance = readTOFSensor(TOF_FRONT_ADDRESS);
  uint16_t leftDistance = readTOFSensor(TOF_LEFT_ADDRESS);
  uint16_t rightDistance = readTOFSensor(TOF_RIGHT_ADDRESS);

  int region = check_region(leftDistance, rightDistance);
  float allign_angle = abs(leftDistance - rightDistance);

  if (check_collision(frontDistance)) {
    // Turn right in case of a collision
    speedL = SPEED + MAX_DIFFERNCE;
    speedR = SPEED - MAX_DIFFERNCE;
    analogWrite(enL, speedL);
    analogWrite(enR, speedR);
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
  } else {
    if (region == 0) {
      follow_wall(leftDistance, rightDistance);
    } else {
      if (allign_angle > MAX_ALLIGN_ANGLE) {
        follow_wall(leftDistance, rightDistance);
      } else {
        reach_distance(leftDistance, rightDistance);
      }
    }
  }
}
