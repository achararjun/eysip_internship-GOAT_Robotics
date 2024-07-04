// Include necessary headers for Arduino and serial communication
#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>

// Define GPIO pins for RS485 communication
const int mdDeRe = 15;  // DE/RE pin for controlling transmit/receive mode
const int rxPin = 16;   // RX pin for receiving data
const int txPin = 17;   // TX pin for transmitting data    

HardwareSerial mySerial(2); // Use UART2

// Function declarations
uint16_t calculateCRC16(const uint8_t* data, size_t length);
void appendCRC(uint8_t* message, size_t length);
void initializeMotor();
void sendMessage(uint8_t* message, size_t length);

// Setup function to initialize serial communication and motor
void setup() {
  pinMode(mdDeRe, OUTPUT);    
  digitalWrite(mdDeRe, LOW); // Set to receive mode initially
 
  mySerial.begin(9600, SERIAL_8N1, rxPin, txPin); // Initialize UART2
  Serial.begin(9600);    // Initialize Serial Monitor for debugging

  // Initialize motor and set to continuous run
  initializeMotor();
}

// Main loop function (empty for now)
void loop() {
  // Main loop can be used for other tasks
}

// CRC calculation function
uint16_t calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Function to append CRC to the message
void appendCRC(uint8_t* message, size_t length) {
    uint16_t crc = calculateCRC16(message, length);
    message[length] = crc & 0xFF;       // Low byte of CRC
    message[length + 1] = (crc >> 8) & 0xFF; // High byte of CRC
}

// Function to initialize and start motor
void initializeMotor() {
  // Define Modbus messages to set motor parameters and trigger motion
  uint8_t velocityModePR0[] = {0x01, 0x06, 0x62, 0x00, 0x00, 0x02};    // Set Velocity Mode for PR0
  uint8_t setVelocity0[] = {0x01, 0x06, 0x62, 0x03, 0x00, 0x3C};      // Set velocity to 60 RPM for PR0
  uint8_t triggerPR0[] = {0x01, 0x06, 0x60, 0x02, 0x00, 0x10};       // Trigger PR0 motion
  uint8_t velocityModePR1[] = {0x01, 0x06, 0x62, 0x08, 0x00, 0x02}; // Set Velocity Mode for PR1
  uint8_t setVelocity1[] = {0x01, 0x06, 0x62, 0x0B, 0x00, 0xC8};   // Set velocity to 200 RPM for PR1
  uint8_t triggerPR1[] = {0x01, 0x06, 0x60, 0x02, 0x00, 0x11};     // Trigger PR1 motion

  // Define the lengths of the original messages
  size_t velocityModePR0Length = sizeof(velocityModePR0);
  size_t setVelocity0Length = sizeof(setVelocity0);
  size_t triggerPR0Length = sizeof(triggerPR0);
  size_t velocityModePR1Length = sizeof(velocityModePR1);
  size_t setVelocity1Length = sizeof(setVelocity1);
  size_t triggerPR1Length = sizeof(triggerPR1);

  // Create buffers with space for CRC
  uint8_t velocityModePR0WithCRC[velocityModePR0Length + 2];
  uint8_t setVelocity0WithCRC[setVelocity0Length + 2];
  uint8_t triggerPR0WithCRC[triggerPR0Length + 2];
  uint8_t velocityModePR1WithCRC[velocityModePR1Length + 2];
  uint8_t setVelocity1WithCRC[setVelocity1Length + 2];
  uint8_t triggerPR1WithCRC[triggerPR1Length + 2];

  // Copy original messages to buffers
  memcpy(velocityModePR0WithCRC, velocityModePR0, velocityModePR0Length);
  memcpy(setVelocity0WithCRC, setVelocity0, setVelocity0Length);
  memcpy(triggerPR0WithCRC, triggerPR0, triggerPR0Length);
  memcpy(velocityModePR1WithCRC, velocityModePR1, velocityModePR1Length);
  memcpy(setVelocity1WithCRC, setVelocity1, setVelocity1Length);
  memcpy(triggerPR1WithCRC, triggerPR1, triggerPR1Length);

  // Calculate and append CRC
  appendCRC(velocityModePR0WithCRC, velocityModePR0Length);
  appendCRC(setVelocity0WithCRC, setVelocity0Length);
  appendCRC(triggerPR0WithCRC, triggerPR0Length);
  appendCRC(velocityModePR1WithCRC, velocityModePR1Length);
  appendCRC(setVelocity1WithCRC, setVelocity1Length);
  appendCRC(triggerPR1WithCRC, triggerPR1Length);

  // Send messages to initialize and start motor
  uint8_t* messages[] = {
    velocityModePR0WithCRC,
    setVelocity0WithCRC,
    triggerPR0WithCRC,
    velocityModePR1WithCRC,
    setVelocity1WithCRC,
    triggerPR1WithCRC   
  };

  // Array of message lengths
  size_t messageLengths[] = {
    velocityModePR0Length + 2,
    setVelocity0Length + 2,
    triggerPR0Length + 2,
    velocityModePR1Length + 2,
    setVelocity1Length + 2,
    triggerPR1Length + 2
  };

  // Send each message for PR0
  for (int i = 0; i < 3; i++) {
    // Set RS485 to transmit mode
    Serial.println("Setting RS485 to transmit mode...");
    digitalWrite(mdDeRe, HIGH); 
    delay(10); // Ensure the pin state is stable
  
    // Send Modbus message
    Serial.println("Sending Modbus message...");
    mySerial.write(messages[i], messageLengths[i]);
    mySerial.flush(); // Ensure all data is sent
    delay(100); // Short delay to ensure message is sent completely

    // Set RS485 to receive mode
    digitalWrite(mdDeRe, LOW); 
    delay(10); // Short delay to allow for mode switch

    // Read response from slave
    Serial.println("Checking for response...");
    if (mySerial.available()) {
      Serial.println("Data received:");
      while (mySerial.available() > 0) {
        byte incomingByte = mySerial.read();
        Serial.print(incomingByte, HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("No data received.");
    }

    delay(5000); // Delay before sending the next message
  }

  // Send each message for PR1
  for (int i = 3; i < 6; i++) {
    // Set RS485 to transmit mode
    Serial.println("Setting RS485 to transmit mode...");
    digitalWrite(mdDeRe, HIGH); 
    delay(10); // Ensure the pin state is stable
  
    // Send Modbus message
    Serial.println("Sending Modbus message...");
    mySerial.write(messages[i], messageLengths[i]);
    mySerial.flush(); // Ensure all data is sent
    delay(100); // Short delay to ensure message is sent completely

    // Set RS485 to receive mode
    digitalWrite(mdDeRe, LOW); 
    delay(10); // Short delay to allow for mode switch

    // Read response from slave
    Serial.println("Checking for response...");
    if (mySerial.available()) {
      Serial.println("Data received:");
      while (mySerial.available() > 0) {
        byte incomingByte = mySerial.read();
        Serial.print(incomingByte, HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("No data received.");
    }

    delay(5000); // Delay before sending the next message
  }
}