#!/usr/bin/env python3

import serial
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class TeensySerial(Node):
    def __init__(self):
        super().__init__("teensy_serial_node")
        print(serial.__version__)
        self.subscription = self.create_subscription(Int8,'/thruster_pwm', self.callback, 10)
        
        self.serial = serial.Serial(port='/dev/ttyACM1', baudrate=9600, timeout=0.1)

    def callback(self, msg):
    	thrusterPWM = msg.data
        command = "THR" + str(thrusterPWM) + "\n"
    	print(f"Sending:" + command)
    	self.serial.write(bytes(command, 'utf-8'))
    	print(self.serial.readline().decode())

def main(args=None):
    rclpy.init(args=args)

    teensySerial = TeensySerial()
    rclpy.spin(teensySerial)

    teensySerial.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
