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
        self.power_saving = 1
        self.velocity_command = [0] * 6
        self.velocity_filtered = [0] * 6
        #feedback
        self.position_fb = [0] * 6
        self.velocity_fb = [0] * 6
        self.effort_fb = [0] * 6
        self.heartbeat_counter = 0
        timer_period_2 = 0.01  # seconds
        self.timer = self.create_timer(timer_period_2, self.filter)
        self.timer2 = self.create_timer(.1, self.heartbeat_function)
        self.x = [h.ref(0) for i in range(30)]
        self.command_format = "global: heartbeat=%d, control_mode=%d, power_saving=%d\r\n\
joint1: stepper_vel=%.4f\r\n\
joint2: stepper_vel=%.4f\r\n\
joint3: stepper_vel=%.4f\r\n\
joint4: stepper_vel=%.4f\r\n\
joint5: stepper_vel=%.4f\r\n\
joint6: stepper_vel=%.4f\r\n\
__end__";
        self.reply_format = "Nothing to reply so far!\r\n\
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
                                                 self.velocity_filtered[4], self.velocity_filtered[5])
             #   print(message)
                self.arm.write(bytes(message, encoding='utf8'))
                reply = self.arm.read_until(str.encode("__end__")).decode('utf-8')
                print(reply)
          except serial.serialutil.SerialException:
                self.get_logger().warning("No USB FS Connection to Arm!")
                try:
                    self.arm = serial.Serial('/dev/arm', 9600, timeout=1)
                except serial.serialutil.SerialException:
                     None
        



    def arm_callback(self,data):
        self.heartbeat_counter = 0
        print("CALLBACK_ARM")
        self.velocity_command = data.velocity

    def settings_callback(self,data):
        self.heartbeat = data.position[list(data.name).index('heartbeat')]
        self.control_mode = data.position[list(data.name).index('control_mode')]
        self.power_saving = data.position[list(data.name).index('power_saving')]
    def publish_jointstate(self):
        message = JointState()
        message.name = ['joint1','joint2','joint3','joint4','joint5','joint6']
  #      print(self.velocities)
        message.header.stamp = self.get_clock().now().to_msg()
        message.velocity = np.array(self.vel_wheel_filt, dtype=np.float32).tolist()
        message.effort = [0.] * 6
        message.position = np.array(self.ang_wheel, dtype=np.float32).tolist()
        self.pub.publish(message)
    def filter(self):
        for c in range(6):
            if (self.velocity_command[c] > self.velocity_filtered[c]  + 0.49):
                self.velocity_filtered[c] += 0.5
            if (self.velocity_command[c] < self.velocity_filtered[c]  - 0.49):
                self.velocity_filtered[c] -= 0.5
    
    def heartbeat_function(self):
        self.heartbeat_counter += 1
   #     print(self.heartbeat_counter)
        if(self.heartbeat_counter > 15):
            self.velocity_command = [0] * 6




def main(args=None):
    rclpy.init()
    usb = arm_usb()
    rclpy.spin(usb)

    
    usb.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
