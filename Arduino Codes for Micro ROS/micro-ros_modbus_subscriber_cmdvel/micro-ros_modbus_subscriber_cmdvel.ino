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
geometry_msgs__msg__Twist msg;
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

void sendModbusCommand(uint8_t* command, size_t length) {
    uint16_t crc = calculateCRC16(command, length);
    command[length] = crc & 0xFF;       // Low byte of CRC
    command[length + 1] = (crc >> 8) & 0xFF; // High byte of CRC

    // Ensure the RS485 is in transmit mode
    digitalWrite(mdDeRe, HIGH); 
    delay(10);

    // Send the Modbus command
    mySerial.write(command, length + 2); // length + 2 for CRC
    mySerial.flush();
    delay(100);

    // Set RS485 to receive mode
    digitalWrite(mdDeRe, LOW); 
    delay(10);

    // Check for response (optional)
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

void executeMotorCommand(float linear_x) {
    uint8_t command[6] = {0x01, 0x06, 0x62, 0x03, 0x00, 0x00};  // Command template to set velocity
    int16_t velocity = static_cast<int16_t>(linear_x * 10);  // Scale velocity

    command[4] = (velocity >> 8) & 0xFF;  // High byte of velocity
    command[5] = velocity & 0xFF;         // Low byte of velocity

    sendModbusCommand(command, sizeof(command));
}

void subscription_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    executeMotorCommand(msg->linear.x);
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
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    delay(100);
}
