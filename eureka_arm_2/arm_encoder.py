#!/usr/bin/env python3.10

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
        self.velocities_filtered = [0.] * 7
        self.pub = self.create_publisher( UInt8MultiArray, "can_tx", 10)
        self.sub = self.create_subscription(JointState,'arm_commands', self.callback, 10)
        self.send_ctr = 0
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.publisher)
        timer_period_2 = 0.05  # seconds
        self.timer = self.create_timer(timer_period_2, self.filter)
        self.timer = self.create_timer(0.015, self.send)
        self.get_logger().info("Arm_Encoder Started!")
    def __del__(self):
        self.get_logger().info("Arm_Encoder Killed!")
    def callback(self, message):
        self.velocities = list(message.velocity )
        self.heartbeat = 0
    def send(self):
        msg = UInt8MultiArray()
        if(self.send_ctr < 6):
            arr = np.array([self.velocities_filtered[self.send_ctr]], dtype = np.float16)
        else:
            arr = np.array([self.velocities[self.send_ctr]], dtype = np.float16)
        data = bytes([self.send_ctr + 21]) + arr.tobytes()
        msg.data = data
        self.pub.publish(msg)
        self.send_ctr += 1
        self.send_ctr %= 7
    def publisher(self):
        self.heartbeat += 1
        if(self.heartbeat > 10):
            self.velocities = [0.] * 7
    def filter(self):
        for c in range(7):
            if (self.velocities[c] > self.velocities_filtered[c]  + 0.0035):
                self.velocities_filtered[c] += 0.003
            if (self.velocities[c] < self.velocities_filtered[c]  - 0.0035):
                self.velocities_filtered[c] -= 0.003

def main(args=None):
    rclpy.init()
    ad = arm_decoder()
    rclpy.spin(ad)

    
    ad.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()