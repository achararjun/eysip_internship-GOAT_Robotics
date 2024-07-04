#include <Arduino.h>
//#include <micro_ros_platformio.h>

#include <micro_ros_arduino.h>
#include <HardwareSerial.h>
include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <ModbusMaster.h>

#define MAX485_DE 15
#define MAX485_RE_NEG 15
#define DEBUG_LED 2

HardwareSerial mySerial(1); // UART1 (GPIO 16 for RX and GPIO 17 for TX)
ModbusMaster nodeModbus;  // Create a ModbusMaster object

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

// RS485 transmission control functions
void preTransmission() {
  digitalWrite(MAX485_DE, HIGH);
  digitalWrite(MAX485_RE_NEG, HIGH);
  digitalWrite(DEBUG_LED, HIGH);  // Turn on the LED during transmission
}

void postTransmission() {
  digitalWrite(MAX485_DE, LOW);
  digitalWrite(MAX485_RE_NEG, LOW);
  digitalWrite(DEBUG_LED, LOW);  // Turn off the LED after transmission
}

// Function to move the motor
void moveMotor() {
  uint8_t result;

  // Enable drive
  result = nodeModbus.writeSingleRegister(0x6000, 0x0006); // Enable drive
  if (result == nodeModbus.ku8MBSuccess) {
    Serial.println("Drive enabled successfully");
  } else {
    Serial.print("Failed to enable drive: ");
    Serial.println(result);
    return;
  }

  // Set velocity
  result = nodeModbus.writeSingleRegister(0x6040, 0x0200); // Set velocity
  if (result == nodeModbus.ku8MBSuccess) {
    Serial.println("Velocity set successfully");
  } else {
    Serial.print("Failed to set velocity: ");
    Serial.println(result);
    return;
  }

  // Start motion
  result = nodeModbus.writeSingleRegister(0x6042, 0x0001); // Start motion
  if (result == nodeModbus.ku8MBSuccess) {
    Serial.println("Motor started successfully");
  } else {
    Serial.print("Failed to start motor: ");
    Serial.println(result);
  }
}

// Timer callback function for micro-ROS
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
    moveMotor();  // Call the function to move the motor
  }
}

// Setup function
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT); 

  digitalWrite(MAX485_RE_NEG, LOW);
  digitalWrite(MAX485_DE, LOW);
  digitalWrite(DEBUG_LED, LOW);

  mySerial.begin(19200, SERIAL_8N1, 16, 17);
  nodeModbus.begin(1, mySerial);  
  nodeModbus.preTransmission(preTransmission);
  nodeModbus.postTransmission(postTransmission);

  Serial.println("Setup complete");

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

// Main loop
void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
