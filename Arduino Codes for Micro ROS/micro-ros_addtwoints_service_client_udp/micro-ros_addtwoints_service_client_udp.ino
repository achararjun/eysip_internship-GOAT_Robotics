// Include necessary headers for micro-ROS and standard libraries
#include <micro_ros_arduino.h>
#include <example_interfaces/srv/add_two_ints.h>
#include <stdio.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int64.h>

// Declare necessary micro-ROS structures and variables
rcl_node_t node;                       // Node handle for ROS node
rclc_support_t support;                // Support structure for initializing micro-ROS
rcl_allocator_t allocator;             // Allocator for managing memory
rclc_executor_t executor;              // Executor for managing ROS callbacks

rcl_client_t client;                   // Client handle for the service
example_interfaces__srv__AddTwoInts_Response res; // Response structure for the service
example_interfaces__srv__AddTwoInts_Request req;  // Request structure for the service
bool response_received = false;        // Flag to indicate if the response has been received

// Macros for checking the return value of functions and handling errors
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){};}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Callback function to handle the response from the service
void response_callback(const void *msgin)
{
  // Cast the incoming message to the appropriate response type
  const example_interfaces__srv__AddTwoInts_Response *res_in = (const example_interfaces__srv__AddTwoInts_Response *)msgin;
  // Print the sum received in the service response
  printf("Service response value: %d.\n", (int)res_in->sum);
  // Set the flag indicating the response has been received
  response_received = true;
}

// Setup function to initialize the micro-ROS client and send a request
void setup() {
  // Initialize Wi-Fi transport for micro-ROS communication
  set_microros_wifi_transports("Arjun", "123arjun", "172.20.10.3", 8888);
  delay(1000); // Delay to ensure Wi-Fi connection is established

  allocator = rcl_get_default_allocator(); // Get the default allocator

  // Initialize the support structure with default options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Initialize the ROS node with the specified name
  RCCHECK(rclc_node_init_default(&node, "add_twoints_client_rclc", "", &support));

  // Initialize the service client for the "AddTwoInts" service
  RCCHECK(rclc_client_init_default(&client, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts), "/addtwoints"));

  // Initialize the executor with one handle and the default allocator
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // Add the client and its response callback to the executor
  RCCHECK(rclc_executor_add_client(&executor, &client, &res, response_callback));
  
  // Initialize the request with the values to be added
  req.a = 3;
  req.b = 4;

  // Send the request to the service
  RCCHECK(rcl_send_request(&client, &req, NULL));
}

// Main loop function to handle the response
void loop() {
  delay(100); // Delay to avoid spamming the executor
  // If the response has not been received, spin the executor
  if (!response_received) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }
}