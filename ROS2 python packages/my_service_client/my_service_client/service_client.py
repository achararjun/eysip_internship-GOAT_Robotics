#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):

    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(AddTwoInts, 'addtwoints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    client = ServiceClient()
    response_received = False

    while rclpy.ok() and not response_received:
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info('Service call failed %r' % (e,))
            else:
                client.get_logger().info('Result of add_two_ints: for %d + %d = %d' %(client.req.a, client.req.b, response.sum))
            response_received = True

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


