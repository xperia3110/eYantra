#!/usr/bin/env python3

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node
import time

class Swift_Pico(Node):
    def __init__(self):
        super().__init__('pico_controller')

        # Current position of the drone [x, y, z]
        self.drone_position = [0.0, 0.0, 0.0]

        # Target setpoint [x_setpoint, y_setpoint, z_setpoint]
        self.setpoint = [2.0, 2.0, 19.0]

        # Initialize the command message for roll, pitch, yaw, and throttle
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        # PID gains for roll, pitch, throttle [roll, pitch, throttle]
        self.Kp = [28.0, 72.0, 620.0]
        self.Ki = [0.0, 1.0, 3.0]
        self.Kd = [70.0, 15.0, 99.0]

        self.error = [0, 0, 0]
        self.sum_error = [0, 0, 0]
        self.diff_error = [0, 0, 0]
        self.prev_error = [0, 0, 0]

        self.sample_time = 0.060  # in seconds

        # Publishers
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)
        self.pid_error = PIDError()

        # Subscribers
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)

        self.arm()

        # Timer to run PID control
        self.create_timer(self.sample_time, self.pid)

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)

    def arm(self):
        self.disarm()
        time.sleep(1)
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)

    # Callback for /whycon/poses
    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    # Callback to set PID parameters for throttle
    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp * 0.03
        self.Ki[2] = alt.ki * 0.008
        self.Kd[2] = alt.kd * 0.6

    # Callback to set PID parameters for roll
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.kp * 0.03
        self.Ki[0] = roll.ki * 0.008
        self.Kd[0] = roll.kd * 0.6

    # Callback to set PID parameters for pitch
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.kp * 0.03
        self.Ki[1] = pitch.ki * 0.008
        self.Kd[1] = pitch.kd * 0.6

    # PID control function
    def pid(self):
        for i in range(3):  # Loop through roll, pitch, and throttle
            self.error[i] = self.drone_position[i] - self.setpoint[i]
            self.sum_error[i] += self.error[i] * self.sample_time
            self.diff_error[i] = (self.error[i] - self.prev_error[i]) / self.sample_time
            self.prev_error[i] = self.error[i]

            # PID output calculation
            pid_output = self.Kp[i] * self.error[i] + self.Ki[i] * self.sum_error[i] + self.Kd[i] * self.diff_error[i]

            # Apply PID output to appropriate command
            if i == 0:
                self.cmd.rc_roll = int(1500 - pid_output)
                self.cmd.rc_roll = max(min(self.cmd.rc_roll, 2000), 1000)
                self.pid_error.roll_error = self.error[i]
            
            elif i == 1:
                self.cmd.rc_pitch = int(1500 + pid_output)
                self.cmd.rc_pitch = max(min(self.cmd.rc_pitch, 2000), 1000)
                self.pid_error.pitch_error = self.error[i]
            
            elif i == 2:
                self.cmd.rc_throttle = int(1500 + pid_output)
                self.cmd.rc_throttle = max(min(self.cmd.rc_throttle, 2000), 1000)
                self.pid_error.throttle_error = self.error[i]

        # Publish the command and the PID errors
        self.command_pub.publish(self.cmd)
        self.pid_error_pub.publish(self.pid_error)

def main(args=None):
    rclpy.init(args=args)
    swift_pico = Swift_Pico()
    rclpy.spin(swift_pico)
    swift_pico.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
