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
        self.positions = [0.] * 7
        self.velocities = [0., 0.0, 0.01, 0.,0.,0.,0.,]
        self.offsets = [0., .9, 0.2, 0.,0.,0.,0.,]
        self.A = self.B = .4
        self.pub = self.create_publisher( JointState, 'arm_commands', 10)
        self.sub1 = self.create_subscription(JointState,'arm_states', self.callback1, 10)
        self.sub2 = self.create_subscription(JointState,'arm_js', self.callback2, 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.publisher)
        self.get_logger().info("Arm_Encoder Started!")
    def __del__(self):
        self.get_logger().info("Arm_Encoder Killed!")
    def callback1(self, message):
        self.positions = list(message.position)
    def callback2(self, message):
        self.velocities = list(message.velocity)
        self.heartbeat = 0

        alpha = self.positions[1] + self.offsets[1]
        beta = self.positions[2] + self.offsets[2]
        gamma = -3.14 + alpha + beta
        x_dot = self.velocities[1]
        y_dot = self.velocities[2]
        temp = (self.B * math.sin(gamma) * (math.cos(alpha)/math.sin(alpha)) - self.B * math.cos(gamma))
        if(temp == 0):
            temp = 0.01
        gamma_dot = (x_dot * (math.cos(alpha)/math.sin(alpha)) - y_dot)/temp
        temp = (self.A * math.sin(alpha))
        if(temp == 0):
            temp = 0.01
        alpha_dot = -(x_dot + self.B * math.sin(gamma) * gamma_dot)/temp
        beta_dot = gamma_dot + alpha_dot
        if(max([abs(alpha_dot), abs(beta_dot), abs(gamma_dot)]) > 0.2):
            alpha_dot *= .1 / max([abs(alpha_dot), abs(beta_dot), abs(gamma_dot)])
            beta_dot *= .1 / max([abs(alpha_dot), abs(beta_dot), abs(gamma_dot)])
            gamma_dot *= .1 / max([abs(alpha_dot), abs(beta_dot), abs(gamma_dot)])
        print(alpha_dot, beta_dot, gamma_dot)
        self.velocities[1] = alpha_dot
        self.velocities[2] = beta_dot
        self.velocities[3] += gamma_dot
        self.send()
    def send(self):
        for c in range(7):
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.velocity = self.velocities
            self.pub.publish(message)
    def publisher(self):
        self.heartbeat += 1
        if(self.heartbeat > 10):
            self.velocities = [0.] * 7
        

    #    alpha_dot = clamp(alpha_dot, -.1, .1)
   #     beta_dot = clamp(beta_dot, -.1, .1)
   #     gamma_dot = clamp(gamma_dot, -.1, .1)
      #  print(alpha, beta, gamma)
        

 #       self.send()



def clamp(n, min, max): 
    if n < min: 
        return min
    elif n > max: 
        return max
    else: 
        return n 


def main(args=None):
    rclpy.init()
    ad = arm_decoder()
    rclpy.spin(ad)

    
    ad.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()