#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

class ModbusPublisher(Node):
    def __init__(self):
        super().__init__('modbus_publisher')
        self.publisher_ = self.create_publisher(UInt8MultiArray, 'modbus_command', 10)
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        commands = [
            [0x01, 0x06, 0x62, 0x00, 0x00, 0x02, 0x17, 0xB3],  # Set Velocity Mode for PR0
            [0x01, 0x06, 0x62, 0x03, 0x00, 0x3C, 0x66, 0x63],  # Set Velocity to 60 RPM for PR0
            [0x01, 0x06, 0x60, 0x02, 0x00, 0x10, 0x37, 0xC6],  # Trigger PR0 motion
            [0x01, 0x06, 0x62, 0x08, 0x00, 0x02, 0x96, 0x71],  # Set Velocity Mode for PR1
            [0x01, 0x06, 0x62, 0x0B, 0x00, 0xC8, 0xE6, 0x26],  # Set Velocity to 200 RPM for PR1
            [0x01, 0x06, 0x60, 0x02, 0x00, 0x11, 0xF6, 0x06]   # Trigger PR1 motion
        ]
        for command in commands:
            msg = UInt8MultiArray()
            msg.data = command
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    modbus_publisher = ModbusPublisher()
    rclpy.spin(modbus_publisher)
    modbus_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
