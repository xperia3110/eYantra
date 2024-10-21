#!/usr/bin/env python3

'''
# Team ID:          3235
# Theme:            Warehouse Drone (WD)
# Author List:      Ben S George, Jemin V Thomas, Aaron S Binu, Jais P Jacob
# Filename:         pico_controller.py
# Functions:        main
# Global variables: None
'''

# Importing the required libraries
import time
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node

class Swift_Pico(Node):
    def __init__(self):
        super().__init__('pico_controller')  # initializing ros node with name pico_controller

        # [x,y,z]
        self.drone_position = [0.0, 0.0, 0.0]

        # [x_setpoint, y_setpoint, z_setpoint]
        self.setpoint = [2, 2, 19]  

        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        self.Kp = list(map(lambda x: x*0.03, [-360, 360, 913]))
        self.Ki = list(map(lambda x: x*0.008, [-1, 1, -3]))
        self.Kd = list(map(lambda x: x*0.6, [-900, 900, 1175]))

        self.error = [0,0,0]
        self.sum_error = [0,0,0]
        self.diff_error = [0,0,0]
        self.prev_error = [0,0,0]
    
        self.sample_time = 0.060  # in seconds

        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)
        self.pid_error = PIDError()

        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)

        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)
    

        self.arm()  # ARMING THE DRONE
        self.start_time = None

        self.timer = self.create_timer(self.sample_time, self.pid)

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)
        

    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)

    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp * 0.03
        self.Ki[2] = alt.ki * 0.008
        self.Kd[2] = alt.kd * 0.6

    def pitch_set_pid(self, alt):
        self.Kp[1] = alt.kp * 0.03  
        self.Ki[1] = alt.ki * 0.008
        self.Kd[1] = alt.kd * 0.6

    def roll_set_pid(self, alt):
        self.Kp[0] = -(alt.kp * 0.03)
        self.Ki[0] = -(alt.ki * 0.008)
        self.Kd[0] = -(alt.kd * 0.6)

    def pid(self):

        if self.start_time is None:
            self.start_time = time.time()

        # [roll, pitch, throttle]
        for i in range(3):
            self.error[i] = self.drone_position[i] - self.setpoint[i]
            self.sum_error[i] += self.error[i]
            self.diff_error[i] = self.error[i] - self.prev_error[i]
            self.prev_error[i] = self.error[i]

        if  18.60 > self.drone_position[2] or self.drone_position[2]> 19.40:
            # self.get_logger().info("Out of range")
            self.in_range_time = 0
        else:
            if self.in_range_time == 0:
                self.in_range_time = time.time()
            
            if (time.time() - self.in_range_time) >= 10:
                self.get_logger().info(f"Task Completed")
            
        # Calculate PID outputs
        roll_output = self.Kp[0] * self.error[0] + self.Ki[0] * self.sum_error[0] + self.Kd[0] * self.diff_error[0]
        pitch_output = self.Kp[1] * self.error[1] + self.Ki[1] * self.sum_error[1] + self.Kd[1] * self.diff_error[1]
        throttle_output = self.Kp[2] * self.error[2] + self.Ki[2] * self.sum_error[2] + self.Kd[2] * self.diff_error[2]

        # Apply outputs safely
        self.cmd.rc_roll = max(1000, min(2000, int(1500 + roll_output)))
        self.cmd.rc_pitch = max(1000, min(2000, int(1500 + pitch_output)))
        self.cmd.rc_throttle = max(1000, min(2000, int(1500 + throttle_output)))
        
        self.command_pub.publish(self.cmd)
        # Publish PID errors
        self.pid_error.roll_error = self.error[0]
        self.pid_error.pitch_error = self.error[1]
        self.pid_error.throttle_error = self.error[2]
        self.pid_error_pub.publish(self.pid_error)


def main(args=None):
    rclpy.init(args=args)
    swift_pico = Swift_Pico()
    rclpy.spin(swift_pico)
    swift_pico.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
