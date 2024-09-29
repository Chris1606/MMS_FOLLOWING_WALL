#include <HardwareSerial.h>

// Define serial ports
HardwareSerial mySerial(1); // Use Serial1 for communication

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    
    // Initialize UART on GPIO 3 (RX) and GPIO 1 (TX)
    mySerial.begin(9600, SERIAL_8N1, 3, 1); // Baud rate, data bits, RX pin, TX pin
    
    Serial.println("Starting communication with TOF10120...");
}

void loop() {
    // Command to request distance measurement
    const char* command = "s7-140#"; // use to read value
    
    // Send command
    mySerial.print(command); // Send command as string

    // Wait for the TOF10120 to respond
    delay(100); // Adjust as necessary based on the response time of your sensor

    // Check if data is available
    if (mySerial.available()) {
        // Read response
        String response = mySerial.readStringUntil('\n'); // Read until newline character

        // Print the response
        Serial.print("Response: ");
        Serial.println(response); // Print the response as a string
    }

    delay(1000); // Delay between readings
}
