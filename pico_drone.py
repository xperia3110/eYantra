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
        self.setpoint = [2.0, 2.0, 20]

        # Initialize the command message for roll, pitch, yaw, and throttle
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        # PID gains for roll, pitch, throttle
        self.Kp = [0.0, 0.0, 0.0]
        self.Ki = [0.0, 0.0, 0.0]
        self.Kd = [0.0, 0.0, 0.0]

        self.error = [0,0,0]
        self.sum_error = [0,0,0]
        self.diff_error = [0,0,0]
        self.prev_error = [0,0,0]

        ''' # PID variables
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]

        # Output limits
        self.max_values = [2000, 2000, 2000]  # [roll, pitch, throttle]
        self.min_values = [1000, 1000, 1000]'''

        self.sample_time = 0.060  # in seconds
        #self.last_time = time.time()  # Initialize last_time to current time

        # Publishers
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)
        self.pid_error = PIDError()

        # Subscribers
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        #self.create_subscription(PIDTune, "/roll_pid",self.roll_set_pid, 1)


        self.arm()

        # Timer to run PID control

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

    def pid(self):

        self.error[2] = self.drone_position - self.setpoint[2]
        self.sum_error[2] = self.sum_error[2] + self.error[2]
        self.diff_error[2] = self.error[2] - self.prev_error[2]
        self.prev_error = self.error[2]

        self.pid_error.throttle_error = self.error[2]

        self.throttle = self.Kp * self.error[2] + self.Ki * self.sum_error[2] * self.diff_error[2]
        

        self.cmd.rc_throttle = int(1500 + self.throttle)

        if self.cmd.rc_throttle >2000:
            self.cmd.rc_throttle = 2000
        elif self.cmd.rc_throttle < 1000:
            self.cmd.rc_throttle = 1000


        '''current_time = time.time()
        delta_time = current_time - self.last_time

        if delta_time >= self.sample_time:
            # Calculate error for roll (x-axis), pitch (y-axis), and throttle (z-axis)
            error = [self.drone_position[i] - self.setpoint[i] for i in range(3)]

            # Compute PID for each axis
            pid_output = []
            for i in range(3):
                # Proportional
                proportional = self.Kp[i] * error[i]
                # Integral
                self.error_sum[i] += error[i] * delta_time
                integral = self.Ki[i] * self.error_sum[i]
                # Derivative
                derivative = self.Kd[i] * (error[i] - self.prev_error[i]) / delta_time

                # PID output
                pid_value = proportional + integral + derivative

                # Limit the output to be within the range
                pid_value = max(min(pid_value, self.max_values[i] - 1500), self.min_values[i] - 1500)

                # Add to command
                pid_output.append(pid_value)

                # Update previous error
                self.prev_error[i] = error[i]

            # Apply PID outputs to roll, pitch, and throttle
            self.cmd.rc_roll = int(1500 + pid_output[0])
            self.cmd.rc_pitch = int(1500 + pid_output[1])
            self.cmd.rc_throttle = int(1500 + pid_output[2])


            # Limit the command values to be within the range
            self.cmd.rc_roll = max(min(self.cmd.rc_roll, self.max_values[0]), self.min_values[0])
            self.cmd.rc_pitch = max(min(self.cmd.rc_pitch, self.max_values[1]), self.min_values[1])
            self.cmd.rc_throttle = max(min(self.cmd.rc_throttle, self.max_values[2]), self.min_values[2])'''

            # Publish the command
            self.command_pub.publish(self.cmd)

            # Publish PID error for debugging
            #pid_error_msg = PIDError()
            #pid_error_msg.roll_error = error[0]   # Roll error
            #pid_error_msg.pitch_error = error[1]  # Pitch error
            #pid_error_msg.throttle_error = error[2]  # Throttle error
            #pid_error_msg.yaw_error = 0.0  # You can set yaw error if required, or leave as 0.0

            self.pid_error_pub.publish(pid_error_msg)

            #self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    swift_pico = Swift_Pico()
    rclpy.spin(swift_pico)
    swift_pico.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()