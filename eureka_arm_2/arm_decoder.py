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
        super().__init__('arm_decoder')
        self.velocities = [0.] * 7
        self.efforts = [0.] * 7
        self.positions = [0.] * 7
        self.pub = self.create_publisher(JointState,'arm_states', 10)
        self.sub = self.sub = self.create_subscription( UInt8MultiArray, "can_rx", self.callback, 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.publisher)
        self.get_logger().info("Arm_Decoder Started!")
    def __del__(self):
        self.get_logger().info("Arm_Decoder Killed!")
    def callback(self, arr):
        index = int(arr.data[0])
        if( 20 < index < 28):
            temp = np.frombuffer(arr.data[1:3], dtype=np.float16)[0]
            if(math.isnan(temp)):
                temp = 0
            self.positions[index - 21] = float(temp)
            self.velocities[index - 21] = float(np.frombuffer(arr.data[3:5], dtype=np.float16)[0])
            self.efforts[index - 21] = float(np.frombuffer(arr.data[5:7], dtype=np.float16)[0])
    def publisher(self):
        message = JointState()
        message.name = ['Rotational1','Rotational2','Rotational3','Rotational4','Rotational5','Rotational6','Slider1']
  #      print(self.velocities)
        message.header.stamp = self.get_clock().now().to_msg()
        message.velocity = self.velocities
        message.effort = self.efforts
        message.position = self.positions
        self.pub.publish(message)

def main(args=None):
    rclpy.init()
    ad = arm_decoder()
    rclpy.spin(ad)

    
    ad.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()