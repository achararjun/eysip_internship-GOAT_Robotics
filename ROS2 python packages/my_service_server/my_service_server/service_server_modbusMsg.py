import rclpy
from rclpy.node import Node
from my_service_server.srv import SendModbusCommand
import socket

def calculate_crc(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

class ModbusServiceServer(Node):

    def __init__(self):
        super().__init__('modbus_service_server')
        self.srv = self.create_service(SendModbusCommand, 'send_modbus_command', self.send_modbus_command_callback)
        self.esp32_ip = "ESP32_IP_ADDRESS"  # Replace with the actual IP address of the ESP32
        self.udp_port = 12345
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_modbus_command_callback(self, request, response):
        # Predefined MODBUS commands
        commands = [
            [0x01, 0x06, 0x62, 0x00, 0x00, 0x02],  # Set Velocity Mode for PR0
            [0x01, 0x06, 0x62, 0x03, 0x00, 0x3C],  # Set Velocity to 60 RPM for PR0
            [0x01, 0x06, 0x60, 0x02, 0x00, 0x10],  # Trigger PR0 motion
            [0x01, 0x06, 0x62, 0x08, 0x00, 0x02],  # Set Velocity Mode for PR1
            [0x01, 0x06, 0x62, 0x0B, 0x00, 0xC8],  # Set Velocity to 200 RPM for PR1
            [0x01, 0x06, 0x60, 0x02, 0x00, 0x11]   # Trigger PR1 motion
        ]

        # Iterate over each command, calculate CRC, and send via UDP
        for command in commands:
            crc = calculate_crc(command)
            command.append(crc & 0xFF)  # Append low byte of CRC
            command.append((crc >> 8) & 0xFF)  # Append high byte of CRC
            self.get_logger().info(f'Sending Modbus message: {command}')

            try:
                self.sock.sendto(bytes(command), (self.esp32_ip, self.udp_port))
            except Exception as e:
                self.get_logger().error(f"Failed to send message: {e}")
                response.success = False
                return response

        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ModbusServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
