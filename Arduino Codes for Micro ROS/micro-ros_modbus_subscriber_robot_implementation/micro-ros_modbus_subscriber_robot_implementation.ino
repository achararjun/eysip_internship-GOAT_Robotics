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
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Variables for command persistence
float last_linear_x = 0.0;
float last_angular_z = 0.0;
unsigned long last_command_time = 0;
const unsigned long command_timeout = 5000; // 500ms timeout

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    delay(100);
  }
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

void sendModbusCommand(const uint8_t* command, size_t length) {
    digitalWrite(mdDeRe, HIGH);
    delay(10);
    mySerial.write(command, length);
    mySerial.flush();
    delay(100);
    digitalWrite(mdDeRe, LOW);
    delay(10);

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
    uint8_t commands[][6] = {
        {0x01, 0x06, 0x20, 0x32, 0x00, 0x03},
        {0x01, 0x06, 0x20, 0x37, 0x01, 0xF4},
        {0x01, 0x06, 0x20, 0x38, 0x01, 0xF4},
        {0x01, 0x06, 0x20, 0x31, 0x00, 0x08},
        {0x02, 0x06, 0x20, 0x32, 0x00, 0x03},
        {0x02, 0x06, 0x20, 0x37, 0x01, 0xF4},
        {0x02, 0x06, 0x20, 0x38, 0x01, 0xF4},
        {0x02, 0x06, 0x20, 0x31, 0x00, 0x08}
    };

    for (int i = 0; i < 8; i++) {
        uint16_t crc = modbus_crc(commands[i], 6);
        uint8_t command_with_crc[8];
        memcpy(command_with_crc, commands[i], 6);
        command_with_crc[6] = crc & 0xFF;
        command_with_crc[7] = (crc >> 8) & 0xFF;
        sendModbusCommand(command_with_crc, sizeof(command_with_crc));
        delay(500);
    }
}

void subscription_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    last_linear_x = msg->linear.x;
    last_angular_z = msg->angular.z;
    last_command_time = millis();
}

void apply_motor_commands(float linear_x, float angular_z) {
    uint8_t command_left[8] = {0x01, 0x06, 0x20, 0x3A};
    uint8_t command_right[8] = {0x02, 0x06, 0x20, 0x3A};

    int16_t target_speed_left = linear_x * 10 - angular_z * 10;
    int16_t target_speed_right = linear_x * 10 + angular_z * 10;

    // Send command to left motor
    command_left[4] = (target_speed_left >> 8) & 0xFF;
    command_left[5] = target_speed_left & 0xFF;
    uint16_t crc_left = modbus_crc(command_left, 6);
    command_left[6] = crc_left & 0xFF;
    command_left[7] = (crc_left >> 8) & 0xFF;
    sendModbusCommand(command_left, 8);

    // Send command to right motor
    command_right[4] = (target_speed_right >> 8) & 0xFF;
    command_right[5] = target_speed_right & 0xFF;
    uint16_t crc_right = modbus_crc(command_right, 6);
    command_right[6] = crc_right & 0xFF;
    command_right[7] = (crc_right >> 8) & 0xFF;
    sendModbusCommand(command_right, 8);
}

void setup() {
    set_microros_wifi_transports("iQOO Z7 5G", "surya1234", "192.168.28.132", 8888);
    delay(1000);
    pinMode(mdDeRe, OUTPUT);
    digitalWrite(mdDeRe, LOW);

    mySerial.begin(115200, SERIAL_8N1, rxPin, txPin);
    Serial.begin(115200);

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
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

    unsigned long current_time = millis();
    if (current_time - last_command_time < command_timeout) {
        apply_motor_commands(last_linear_x, last_angular_z);
    } else {
        apply_motor_commands(0, 0);
    }

    delay(10);
}
