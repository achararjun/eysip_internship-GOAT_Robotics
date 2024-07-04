#include <HardwareSerial.h>

const int mdDeRe = 15;   // Define DE/RE pin for RS485 module
const int rxPin = 17;    // Define RX pin (should be connected to RO of RS485 module)
const int txPin = 16;    // Define TX pin (should be connected to DI of RS485 module)

HardwareSerial mySerial(2); // Define mySerial to use UART1

void setup() {
  pinMode(mdDeRe, OUTPUT);    // Set DE/RE pin as OUTPUT
  digitalWrite(mdDeRe, LOW);  // Set to receive mode by default
  
  mySerial.begin(115200, SERIAL_8N1, rxPin, txPin); // Initialize UART1 with proper pins and configuration
  Serial.begin(115200);       // Initialize Serial monitor for debugging
}

void loop() {
  uint8_t modbusMessage[] = {0x01, 0x03, 0x01, 0x91, 0x00, 0x01, 0xD3, 0x1B}; // Modbus message

  digitalWrite(mdDeRe, HIGH); // Set RS485 to transmit mode

  // Send Modbus message
  mySerial.write(modbusMessage, sizeof(modbusMessage));
  mySerial.flush();            // Ensure the message is completely sent

  delay(120);                  // Short delay to ensure complete transmission
  digitalWrite(mdDeRe, LOW);   // Set RS485 to receive mode
  delay(2000);                 // Delay between sending and receiving

  // Read response from slave
  if (mySerial.available()) {
    while (mySerial.available() > 0) {
      byte incomingByte = mySerial.read();
      Serial.print(incomingByte, HEX); // Print received byte to Serial monitor in HEX
      Serial.print(" ");
    }
    Serial.println();
  }
}
