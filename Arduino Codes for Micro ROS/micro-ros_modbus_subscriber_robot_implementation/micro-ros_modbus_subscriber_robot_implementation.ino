#include <micro_ros_arduino.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <geometry_msgs/msg/twist.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

const int mdDeRe = 15;  // Define GPIO for DE/RE pin
const int rxPin = 16;   // Define GPIO for RX pin
const int txPin = 17;   // Define GPIO for TX pin    

HardwareSerial mySerial(2); // Use UART2

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    delay(100);
  }
}

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

void setupMotor() {
    uint8_t commands[][8] = {
        {0x01, 0x06, 0x20, 0x32, 0x00, 0x03, 0x63, 0xC4},  // Set Profile Velocity Mode
        {0x01, 0x06, 0x20, 0x37, 0x01, 0xF4, 0x33, 0xD3},  // Set S-type acceleration time 500ms
        {0x01, 0x06, 0x20, 0x38, 0x01, 0xF4, 0x03, 0xD0},  // Set S-type deceleration time 500ms
        {0x01, 0x06, 0x20, 0x31, 0x00, 0x08, 0xD2, 0x03},  // Motor enable
        {0x01, 0x06, 0x20, 0x3A, 0x00, 0x00, 0xA3, 0xEC}   // Initialize target speed to 0
    };

    for (int i = 0; i < 5; i++) {
        sendModbusCommand(commands[i], sizeof(commands[i]));
        delay(500); // Short delay between commands
    }
}

void subscription_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    uint8_t command[8] = {0x01, 0x06, 0x20, 0x3A}; // Base command for setting speed
    int16_t target_speed_left = msg->linear.x * 10 - msg->angular.z * 10;  // Left motor speed
    int16_t target_speed_right = msg->linear.x * 10 + msg->angular.z * 10; // Right motor speed

    // Send command to left motor
    command[4] = (target_speed_left >> 8) & 0xFF;  // High byte of speed
    command[5] = target_speed_left & 0xFF;         // Low byte of speed
    uint16_t crc = modbus_crc(command, 6);
    command[6] = crc & 0xFF;      // CRC low byte
    command[7] = (crc >> 8) & 0xFF;  // CRC high byte
    sendModbusCommand(command, 8);

    // Send command to right motor
    command[4] = (target_speed_right >> 8) & 0xFF;  // High byte of speed
    command[5] = target_speed_right & 0xFF;         // Low byte of speed
    crc = modbus_crc(command, 6);
    command[6] = crc & 0xFF;      // CRC low byte
    command[7] = (crc >> 8) & 0xFF;  // CRC high byte
    sendModbusCommand(command, 8);
}

uint16_t modbus_crc(uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void setup() {
    set_microros_wifi_transports("Arjun", "1234arjun", "172.20.10.3", 8888);
    delay(1000);
    pinMode(mdDeRe, OUTPUT);
    digitalWrite(mdDeRe, LOW);

    mySerial.begin(9600, SERIAL_8N1, rxPin, txPin);
    Serial.begin(9600);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "modbus_node", "", &support));
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg, &subscription_callback, ON_NEW_DATA));

    setupMotor();
}

void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    delay(100);
}
