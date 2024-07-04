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

rcl_service_t service;                 // Service handle for the "AddTwoInts" service
rcl_wait_set_t wait_set;               // Wait set for managing waitable entities

example_interfaces__srv__AddTwoInts_Response res; // Response structure for the service
example_interfaces__srv__AddTwoInts_Request req;  // Request structure for the service

// Macros for checking the return value of functions and handling errors
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){};}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Callback function to handle the service request
void service_callback(const void * req, void * res) {
  // Cast the incoming request to the appropriate type
  example_interfaces__srv__AddTwoInts_Request * req_in = (example_interfaces__srv__AddTwoInts_Request *) req;
  // Cast the response to the appropriate type
  example_interfaces__srv__AddTwoInts_Response * res_in = (example_interfaces__srv__AddTwoInts_Response *) res;

  // Print the values received in the service request
  printf("Service request value: %d + %d.\n", (int) req_in->a, (int) req_in->b);

  // Calculate the sum and set it in the response
  res_in->sum = req_in->a + req_in->b;
}

// Setup function to initialize the micro-ROS service and its executor
void setup() {
  // Initialize Wi-Fi transport for micro-ROS communication
  set_microros_wifi_transports("Arjun", "123arjun", "172.20.10.3", 8888);
  delay(1000); // Delay to ensure Wi-Fi connection is established

  allocator = rcl_get_default_allocator(); // Get the default allocator

  // Initialize the support structure with default options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Initialize the ROS node with the specified name
  RCCHECK(rclc_node_init_default(&node, "add_twoints_client_rclc", "", &support));

  // Initialize the service for the "AddTwoInts" service
  RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts), "/addtwoints"));

  // Initialize the executor with one handle and the default allocator
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // Add the service and its callback to the executor
  RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));
}

// Main loop function to handle incoming service requests
void loop() {
  delay(100); // Delay to avoid spamming the executor
  // Spin the executor to handle any incoming requests
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}