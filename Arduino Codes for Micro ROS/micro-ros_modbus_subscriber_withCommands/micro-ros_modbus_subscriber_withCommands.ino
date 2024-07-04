// Include necessary headers for micro-ROS and Arduino libraries
#include <micro_ros_arduino.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <std_msgs/msg/bool.h>
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
std_msgs__msg__Bool msg;                 // Message structure for receiving Bool messages
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

// Function to send a Modbus command over RS485
void sendModbusCommand(const uint8_t* command, size_t length) {
    // Ensure the RS485 is in transmit mode
    digitalWrite(mdDeRe, HIGH); 
    delay(10);

    // Send the Modbus command
    mySerial.write(command, length);
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

// Function to execute a sequence of motor commands via Modbus
void executeMotorCommands() {
    uint8_t commands[][8] = {
        {0x01, 0x06, 0x62, 0x00, 0x00, 0x02, 0x17, 0xB3},  // Set Velocity Mode for PR0
        {0x01, 0x06, 0x62, 0x03, 0x00, 0x3C, 0x66, 0x63},  // Set Velocity to 60 RPM for PR0
        {0x01, 0x06, 0x60, 0x02, 0x00, 0x10, 0x37, 0xC6},  // Trigger PR0 motion
        {0x01, 0x06, 0x62, 0x08, 0x00, 0x02, 0x96, 0x71},  // Set Velocity Mode for PR1
        {0x01, 0x06, 0x62, 0x0B, 0x00, 0xC8, 0xE6, 0x26},  // Set Velocity to 200 RPM for PR1
        {0x01, 0x06, 0x60, 0x02, 0x00, 0x11, 0xF6, 0x06}   // Trigger PR1 motion
    };

    // Send each command with a delay between commands
    for (int i = 0; i < 6; i++) {
        sendModbusCommand(commands[i], sizeof(commands[i]));
        delay(5000); // Delay between commands to ensure they are processed correctly
    }
}

// Callback function for receiving Bool messages and executing motor commands
void subscription_callback(const void* msgin) {
    const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*)msgin;
    if (msg->data) {
        executeMotorCommands();
    }
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
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "modbus_command"));         // Initialize subscription for "modbus_command" topic

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // Initialize executor
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA)); // Add subscription to executor
}

// Main loop function to handle incoming messages
void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); // Spin executor to handle messages
    delay(100); // Delay to avoid spamming the executor
}