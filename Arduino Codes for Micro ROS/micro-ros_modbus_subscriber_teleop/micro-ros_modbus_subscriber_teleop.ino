// Include necessary headers for micro-ROS and Arduino libraries
#include <micro_ros_arduino.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <geometry_msgs/msg/twist.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Define GPIO pins for RS485 communication
const int mdDeRe = 15;  // DE/RE pin for controlling transmit/receive mode
const int rxPin = 16;   // RX pin for receiving data
const int txPin = 17;   // TX pin for transmitting data    

HardwareSerial mySerial(2); // Use UART2 for serial communication

// Declare necessary micro-ROS structures and variables
rcl_subscription_t subscriber;           // Subscriber handle for ROS topic
geometry_msgs__msg__Twist msg;           // Message structure for receiving Twist messages
rclc_executor_t executor;                // Executor for managing ROS callbacks
rcl_allocator_t allocator;               // Allocator for managing memory
rclc_support_t support;                  // Support structure for initializing micro-ROS
rcl_node_t node;                         // Node handle for ROS node

// Macros for checking the return value of functions and handling errors
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// Function to handle errors by entering an infinite loop
void error_loop(){
  while(1){
    delay(100);
  }
}

// Function to calculate CRC16 for Modbus commands
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

// Function to send a Modbus command over RS485
void sendModbusCommand(uint8_t* command, size_t length) {
    // Calculate CRC and append to command
    uint16_t crc = calculateCRC16(command, length);
    command[length] = crc & 0xFF;
    command[length + 1] = (crc >> 8) & 0xFF;
    
    // Ensure the RS485 is in transmit mode
    digitalWrite(mdDeRe, HIGH); 
    delay(10);

    // Send the Modbus command
    mySerial.write(command, length + 2); // Send command including CRC
    mySerial.flush();
    delay(100);

    // Set RS485 to receive mode
    digitalWrite(mdDeRe, LOW); 
    delay(10);

    // Check for response
    if (mySerial.available()) {
        Serial.print("Response: ");
        while (mySerial.available() > 0) {
            byte incomingByte = mySerial.read();
            Serial.print(incomingByte, HEX);
            Serial.print(" ");
        }
        Serial.println();
    } else {
        Serial.println("No response received.");
    }
}

// Function to initialize the motor with specific Modbus commands
void initializeMotor() {
    uint8_t setVelocityModePR0[6] = {0x01, 0x06, 0x62, 0x00, 0x00, 0x02};  // Set Velocity Mode for PR0
    sendModbusCommand(setVelocityModePR0, 6);

    uint8_t setInitialVelocity[6] = {0x01, 0x06, 0x62, 0x03, 0x00, 0x00};  // Set initial velocity to 0 for PR0
    sendModbusCommand(setInitialVelocity, 6);

    uint8_t triggerPR0[6] = {0x01, 0x06, 0x60, 0x02, 0x00, 0x10};  // Trigger PR0 motion
    sendModbusCommand(triggerPR0, 6);
}

// Callback function for receiving Twist messages and executing motor commands
void subscription_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    
    float linear_velocity = msg->linear.x;  // Extract linear velocity
    
    uint8_t setVelocityPR0[8] = {0x01, 0x06, 0x62, 0x03, 0x00, 0x00, 0x00, 0x00};  // Command template to set velocity
    setVelocityPR0[4] = (uint8_t)((int)(linear_velocity * 100) >> 8);  // Convert float to suitable value and place in command
    setVelocityPR0[5] = (uint8_t)((int)(linear_velocity * 100) & 0xFF);
    
    sendModbusCommand(setVelocityPR0, 6);  // Send velocity command
    
    // Trigger PR0 motion
    uint8_t triggerPR0[6] = {0x01, 0x06, 0x60, 0x02, 0x00, 0x10};
    sendModbusCommand(triggerPR0, 6);
}

// Setup function to initialize micro-ROS, RS485 communication, and subscription
void setup() {
    set_microros_wifi_transports("Arjun", "1234arjun", "172.20.10.3", 8888);  // Initialize Wi-Fi transport for micro-ROS communication
    delay(1000); // Delay to ensure Wi-Fi connection is established
    pinMode(mdDeRe, OUTPUT);       // Set DE/RE pin as output
    digitalWrite(mdDeRe, LOW);     // Set DE/RE pin to receive mode

    mySerial.begin(9600, SERIAL_8N1, rxPin, txPin); // Initialize serial communication
    Serial.begin(9600);            // Initialize debug serial communication

    allocator = rcl_get_default_allocator(); // Get the default allocator
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); // Initialize support structure
    RCCHECK(rclc_node_init_default(&node, "modbus_node", "", &support)); // Initialize ROS node
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));               // Initialize subscription for "cmd_vel" topic

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // Initialize executor
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA)); // Add subscription to executor

    initializeMotor();  // Initialize motor settings
}

// Main loop function to handle incoming messages
void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); // Spin executor to handle messages
    delay(100); // Delay to avoid spamming the executor
}