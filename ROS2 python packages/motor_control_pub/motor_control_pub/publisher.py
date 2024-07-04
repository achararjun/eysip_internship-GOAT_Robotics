import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ModbusPublisher(Node):
    def __init__(self):
        super().__init__('modbus_publisher')
        self.publisher_ = self.create_publisher(String, 'modbus_commands', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.commands = [
            "01 06 62 00 00 02 17 B3",  # Set Velocity Mode for PR0
            "01 06 62 03 00 3C 66 63",  # Set Velocity to 60 RPM for PR0
            "01 06 60 02 00 10 37 C6",  # Trigger PR0 motion
            "01 06 62 08 00 02 96 71",  # Set Velocity Mode for PR1
            "01 06 62 0B 00 C8 E6 26",  # Set Velocity to 200 RPM for PR1
            "01 06 60 02 00 11 F6 06"   # Trigger PR1 motion
        ]
        self.current_command = 0

    def timer_callback(self):
        command_str = self.commands[self.current_command]
        msg = String()
        msg.data = command_str
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.current_command = (self.current_command + 1) % len(self.commands)  

def main(args=None):
    rclpy.init(args=args)
    modbus_publisher = ModbusPublisher()
    rclpy.spin(modbus_publisher)
    modbus_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
