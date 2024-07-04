#include <micro_ros_arduino.h>
#include <example_interfaces/srv/add_two_ints.h>
#include <stdio.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int64.h>

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_client_t client;
example_interfaces__srv__AddTwoInts_Response res;
example_interfaces__srv__AddTwoInts_Request req;
bool response_received = false;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){};}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void response_callback(const void *msgin)
{
  const example_interfaces__srv__AddTwoInts_Response *res_in = (const example_interfaces__srv__AddTwoInts_Response *)msgin;
  printf("Service response value: %d.\n", (int)res_in->sum);
  response_received = true;
}

void setup() {
  set_microros_wifi_transports("Arjun", "123arjun", "172.20.10.3", 8888);
  delay(1000); 

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "add_twoints_client_rclc", "", &support));

  // create client
  RCCHECK(rclc_client_init_default(&client, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts), "/addtwoints"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_client(&executor, &client, &res, response_callback));
  
  // Initialize the request
  req.a = 3;
  req.b = 4;

  // Send request
  RCCHECK(rcl_send_request(&client, &req, NULL));
}

void loop() {
  delay(100);
  if (!response_received) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }
}
