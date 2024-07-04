#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import threading
import rclpy
from rclpy.node import Node
import math

class arm_decoder(Node):
    def __init__(self):
        super().__init__('arm_encoder')
        self.heartbeat = 0
        self.velocities = [0.] * 7
        self.pub = self.create_publisher( UInt8MultiArray, "can_tx", 10)
        self.sub = self.create_subscription(JointState,'arm_commands', self.callback, 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.publisher)
        self.get_logger().info("Arm_Encoder Started!")
    def __del__(self):
        self.get_logger().info("Arm_Encoder Killed!")
    def callback(self, message):
        self.velocities = list(message.velocities)
        self.heartbeat = 0
    def send(self):
        for c in range(7):
            msg = UInt8MultiArray()
            arr = np.array([self.velocities[c]], dtype = np.float16)
            data = bytes([c + 21]) + arr.tobytes()
            msg.data = data
      #      print(data)
            self.pub.publish(msg)
    def publisher(self):
        self.heartbeat += 1
        if(self.heartbeat > 10):
            self.velocities = [0.] * 7
        self.send()

def main(args=None):
    rclpy.init()
    ad = arm_decoder()
    rclpy.spin(ad)

    
    ad.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()