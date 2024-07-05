# eYSIP_internship-GOAT_Robotics 2024

## Project Title: Grid Solving Autonomous Sorting Robot

The objective of the overall problem statement is to create a multi agent sorting robot, that can handle multiple sources and multiple destinations. My contribution to this project is to create a micro ROS client that runs on an ESP32 and can communicate with the ROS2 server wirelessly. 

### Components and hardware used:
- ESP32
- RS485 to ttl Converter
- Leadshine CS2RS-D507 Stepper Motor Driver
- Leadshine CS-M22331-L Closed Loop Stepper Motor
- ZLAC8015 AC Servo Motor Driver

### Tech Stack used:
- ROS2
- Micro ROS
- Arduino IDE
- Platform IO
- MODBUS
- RS485

### Steps to run the code:

#### 1. Flash the ESP32 with the code

#### 2. Run the nmap package to create a netwrok bridge:
```
sudo nmap -sn <ip_address>/24
```
Download nmap package if not installed and replace ip_address with your host ip address.
![image](https://github.com/achararjun/eysip_internship-GOAT_Robotics/assets/106529997/30333bf2-34f6-4dbf-a677-fb42d3d893f4)

#### 3. Run the micro ROS Agent:

```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 --dev <ip_address>
```
Here the ip_address is not the address of the host machine, but that of the ESP32 which is obtained after step 2.

#### Run the teleop twist keyboard command

```
ros2 run telop_twist_keyboard teleop_twist_keyboard
```

