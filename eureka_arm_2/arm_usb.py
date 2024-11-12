#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 
print("LALALA")
from sensor_msgs.msg import JointState
import numpy as np
import rclpy
from rclpy.node import Node


import serial
from neuron import h


class arm_usb(Node):
    def __init__(self):
        super().__init__('arm_usb')
        self.sub = self.create_subscription(JointState, "arm_commands", self.arm_callback, 10)
        self.sub_3 = self.create_subscription(JointState, "arm_settings", self.settings_callback, 10)
        self.pub = self.create_publisher(JointState, "arm_states", 10)
        self.pub_2 = self.create_publisher(JointState, "arm_commands_filtered", 10)
        #commands
        self.heartbeat = 1
        self.control_mode = 1
        self.power_saving = 0
        self.velocity_command = [0] * 7
        self.velocity_filtered = [0] * 7
        #feedback
        self.position_fb = [0] * 7
        self.velocity_fb = [0] * 7
        self.effort_fb = [0] * 7
        timer_period_2 = 0.01  # seconds
        self.timer = self.create_timer(timer_period_2, self.filter)
        self.x = [h.ref(0) for i in range(30)]
        self.command_format = "global: heartbeat=%d, control_mode=%d, power_saving=%d\r\n\
joint1: vel=%.4f\r\n\
joint2: vel=%.4f\r\n\
joint3: vel=%.4f\r\n\
joint4: vel=%.4f\r\n\
joint5: vel=%.4f\r\n\
joint6: vel=%.4f\r\n\
gripper: vel=%.4f\r\n\
__end__";
        self.reply_format = "joint1: pos=%f, vel=%f, effort=%f\r\n\
joint2: pos=%f, vel=%f, effort=%f\r\n\
joint3: pos=%f, vel=%f, effort=%f\r\n\
joint4: pos=%f, vel=%f, effort=%f\r\n\
joint5: pos=%f, vel=%f, effort=%f\r\n\
joint6: pos=%f, vel=%f, effort=%f\r\n\
gripper: pos=%f, vel=%f, effort=%f\r\n\
__end__"
        flag = 0
        while flag < 1:
            try:
                self.arm = serial.Serial('/dev/arm', 9600, timeout=1)
                flag = 1
                continue
            except serial.serialutil.SerialException:
                None
        flag = 0
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.send)
        self.send()
        self.get_logger().info("arm_usb Started!")
    def __del__(self):
        self.get_logger().info("arm_usb Killed!")

    def send(self):
          try:
           #     print(len(self.command_format))
                message = self.command_format % (self.heartbeat, self.control_mode, self.power_saving,
                                                 self.velocity_filtered[0], self.velocity_filtered[1], 
                                                 self.velocity_filtered[2], self.velocity_filtered[3], 
                                                 self.velocity_filtered[4], self.velocity_filtered[5], 
                                                 self.velocity_filtered[6])
                print(message)
                self.arm.write(bytes(message, encoding='utf8'))
                reply = self.arm.read_until(str.encode("__end__")).decode('utf-8')
                print(reply)
           #     print(reply.split('\n'))
                ctr = 0
                for (line, line_format) in zip(reply.split('\n'), self.reply_format.split('\n')):
          #          print('Iterated!')
                    num = h.sscanf(line, line_format, self.x[ctr], self.x[ctr + 1], self.x[ctr + 2])
                    ctr += 3
                for ctr in range(0,7):
                    self.position_fb[ctr] = float(self.x[(ctr) * 3][0])
                    self.velocity_fb[ctr] = float(self.x[(ctr) * 3 + 1][0])
                    self.effort_fb[ctr] = float(self.x[(ctr) * 3 + 2][0])
                message = JointState()
                message.name = ['Slider1','Rotational1','Rotational2','Rotational3','Rotational4','Rotational5', 'Slider2']
                message.header.stamp = self.get_clock().now().to_msg()
                message.velocity = self.velocity_fb
                message.effort = self.effort_fb
                message.position = self.position_fb 
                self.pub.publish(message)
                message.velocity = np.array(self.velocity_filtered, dtype=np.float32).tolist()
                message.position = [0.] * 7
                message.effort = [0.] * 7
                self.pub_2.publish(message)
          except serial.serialutil.SerialException:
                try:
                    self.arm = serial.Serial('/dev/arm', 9600, timeout=1)
                except serial.serialutil.SerialException:
                     None
        



    def arm_callback(self,data):
        #print("CALLBACK_DC")
        self.velocity_command = data.velocity
    def settings_callback(self,data):
        self.heartbeat = data.position[list(data.name).index('heartbeat')]
        self.control_mode = data.position[list(data.name).index('control_mode')]
        self.power_saving = data.position[list(data.name).index('power_saving')]
    def publish_jointstate(self):
        message = JointState()
        message.name = ['DC1','DC2','DC3','DC4','DC5','DC6']
  #      print(self.velocities)
        message.header.stamp = self.get_clock().now().to_msg()
        message.velocity = np.array(self.vel_wheel_filt, dtype=np.float32).tolist()
        message.effort = [0.] * 6
        message.position = np.array(self.ang_wheel, dtype=np.float32).tolist()
        self.pub.publish(message)
    def filter(self):
        for c in range(7):
            if (self.velocity_command[c] > self.velocity_filtered[c]  + 0.0009):
                self.velocity_filtered[c] += 0.001
            if (self.velocity_command[c] < self.velocity_filtered[c]  - 0.0009):
                self.velocity_filtered[c] -= 0.001




def main(args=None):
    rclpy.init()
    usb = arm_usb()
    rclpy.spin(usb)

    
    usb.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
