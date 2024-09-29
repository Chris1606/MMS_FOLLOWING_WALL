#include <Wire.h>

#define TOF_FRONT_ADDRESS 0x46  // Front sensor I2C address
#define TOF_LEFT_ADDRESS  0x4B  // Left sensor I2C address
#define TOF_RIGHT_ADDRESS 0x52  // Right sensor I2C address

// Define sensor registers (based on the TOF10120 sensor datasheet)
#define TOF_REG_DISTANCE 0x00  // Register to read distance data

// Function to initialize I2C
void setup() {
  Serial.begin(9600);
  Wire.begin();  // Initialize I2C on ESP32

  delay(1000);  // Wait a moment for the sensors to stabilize

  Serial.println("Reading TOF sensors...");
}

// Function to read distance from a sensor
uint16_t readTOFSensor(uint8_t sensorAddress) {
  uint16_t distance = 0;
  
  Wire.beginTransmission(sensorAddress); // Select the sensor by its I2C address
  Wire.write(TOF_REG_DISTANCE);          // Request to read from distance register
  Wire.endTransmission();

  Wire.requestFrom(sensorAddress, (uint8_t)2); // Request 2 bytes of data
  if (Wire.available() == 2) {
    uint8_t highByte = Wire.read();   // Read high byte
    uint8_t lowByte = Wire.read();    // Read low byte
    distance = (highByte << 8) | lowByte;  // Combine the two bytes into a 16-bit value
  }
  return distance;
}

void loop() {
  // Read the distances from the 3 TOF sensors
  uint16_t frontDistance = readTOFSensor(TOF_FRONT_ADDRESS);
  uint16_t leftDistance = readTOFSensor(TOF_LEFT_ADDRESS);
  uint16_t rightDistance = readTOFSensor(TOF_RIGHT_ADDRESS);

  // Print out the distance readings to the Serial Monitor
  Serial.print("Front Distance: ");
  Serial.print(frontDistance);
  Serial.print(" mm, Left Distance: ");
  Serial.print(leftDistance);
  Serial.print(" mm, Right Distance: ");
  Serial.print(rightDistance);
  Serial.println(" mm");

  delay(500);  // Delay for half a second before the next reading
}
