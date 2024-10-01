#include <Wire.h>

#define TOF_FRONT_ADDRESS 0x46  // Front sensor I2C address
#define TOF_LEFT_ADDRESS  0x4B  // Left sensor I2C address
#define TOF_RIGHT_ADDRESS 0x52  // Right sensor I2C address

// Define sensor registers (based on the TOF10120 sensor datasheet)
#define TOF_REG_DISTANCE 0x00  // Register to read distance data

#define NUM_READINGS 10  // Number of readings to average
uint16_t frontDistanceReadings[NUM_READINGS];
uint16_t leftDistanceReadings[NUM_READINGS];
uint16_t rightDistanceReadings[NUM_READINGS];

int readIndex = 0;
uint16_t totalFront = 0, totalLeft = 0, totalRight = 0;  // Running totals

void setup() {
  Serial.begin(9600);
  Wire.begin();  // Initialize I2C on ESP32

  delay(1000);  // Wait a moment for the sensors to stabilize

  Serial.println("Reading TOF sensors...");

  // Initialize the reading arrays to 0
  for (int i = 0; i < NUM_READINGS; i++) {
    frontDistanceReadings[i] = 0;
    leftDistanceReadings[i] = 0;
    rightDistanceReadings[i] = 0;
  }
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
  // Read the new distances from the 3 TOF sensors
  uint16_t newFront = readTOFSensor(TOF_FRONT_ADDRESS);
  uint16_t newLeft = readTOFSensor(TOF_LEFT_ADDRESS);
  uint16_t newRight = readTOFSensor(TOF_RIGHT_ADDRESS);

  // Subtract the oldest reading from the totals
  totalFront = totalFront - frontDistanceReadings[readIndex];
  totalLeft = totalLeft - leftDistanceReadings[readIndex];
  totalRight = totalRight - rightDistanceReadings[readIndex];

  // Add the new readings to the totals
  frontDistanceReadings[readIndex] = newFront;
  leftDistanceReadings[readIndex] = newLeft;
  rightDistanceReadings[readIndex] = newRight;
  
  totalFront = totalFront + newFront;
  totalLeft = totalLeft + newLeft;
  totalRight = totalRight + newRight;

  // Increment the readIndex (wrap around if necessary)
  readIndex = (readIndex + 1) % NUM_READINGS;

  // Calculate the moving averages
  uint16_t avgFront = totalFront / NUM_READINGS;
  uint16_t avgLeft = totalLeft / NUM_READINGS;
  uint16_t avgRight = totalRight / NUM_READINGS;

  // Output the labeled moving average distances to the Serial Monitor
  Serial.print("Front Distance (Avg): ");
  Serial.print(avgFront);
  Serial.print(" mm, Left Distance (Avg): ");
  Serial.print(avgLeft);
  Serial.print(" mm, Right Distance (Avg): ");
  Serial.print(avgRight);
  Serial.println(" mm");

  // Output just the averaged values for the Serial Plotter (comma-separated)
  Serial.print(avgFront);
  Serial.print(",");
  Serial.print(avgLeft);
  Serial.print(",");
  Serial.println(avgRight);

  delay(1000);  // Delay for half a second before the next reading
}
