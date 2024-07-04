#include <micro_ros_arduino.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <std_msgs/msg/bool.h>

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
std_msgs__msg__Bool msg;
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

void executeMotorCommands() {
    uint8_t commands[][8] = {
        {0x01, 0x06, 0x62, 0x00, 0x00, 0x02, 0x17, 0xB3},  // Set Velocity Mode for PR0
        {0x01, 0x06, 0x62, 0x03, 0x00, 0x3C, 0x66, 0x63},  // Set Velocity to 60 RPM for PR0
        {0x01, 0x06, 0x60, 0x02, 0x00, 0x10, 0x37, 0xC6},  // Trigger PR0 motion
        {0x01, 0x06, 0x62, 0x08, 0x00, 0x02, 0x96, 0x71},  // Set Velocity Mode for PR1
        {0x01, 0x06, 0x62, 0x0B, 0x00, 0xC8, 0xE6, 0x26},  // Set Velocity to 200 RPM for PR1
        {0x01, 0x06, 0x60, 0x02, 0x00, 0x11, 0xF6, 0x06}   // Trigger PR1 motion
    };

    for (int i = 0; i < 6; i++) {
        sendModbusCommand(commands[i], sizeof(commands[i]));
        delay(5000); // Delay between commands to ensure they are processed correctly
    }
}

void subscription_callback(const void* msgin) {
    const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*)msgin;
    if (msg->data) {
        executeMotorCommands();
    }
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
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "modbus_command"));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    delay(100);
}
