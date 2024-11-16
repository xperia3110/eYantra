#!/usr/bin/env python3

import time
import math
from tf_transformations import euler_from_quaternion

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
from nav_msgs.msg import Odometry

# Import your custom action
from waypoint_navigation.action import NavToWaypoint

class WayPointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        self.pid_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()

        # Initialize variables
        self.time_inside_sphere = 0
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.dtime = 0

        # Drone position (updated in real-time from callbacks)
        self.drone_position = [0.0, 0.0, 0.0, 0.0]  # [x, y, z, yaw]

        # Setpoint for the waypoint navigation (updated from action goal)
        self.setpoint = [0.0, 0.0, 0.0, 0.0]  # [x, y, z, yaw] (initially 0)
        
        # Initializing the SwiftMsgs object for commands
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        # PID gains for throttle, pitch, roll, and yaw
        self.Kp = [0.6, 0.6, 0.6, 0.3]
        self.Ki = [0.0, 0.0, 0.0, 0.0]
        self.Kd = [0.3, 0.3, 0.3, 0.1]

        # Variables for error tracking
        self.prev_errors = [0.0, 0.0, 0.0, 0.0]
        self.integral_errors = [0.0, 0.0, 0.0, 0.0]
        self.pid_error = PIDError()
        self.sample_time = 0.060

        # Publishers and Subscribers
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(Odometry, '/rotors/odometry', self.odometry_callback, 10)

        # Action server for waypoint navigation
        self.action_server = ActionServer(
            self,
            NavToWaypoint,
            'waypoint_navigation',
            self.execute_callback,
            callback_group=self.action_callback_group
        )

        # Initialize and arm the drone
        self.arm()
        self.timer = self.create_timer(self.sample_time, self.pid, callback_group=self.pid_callback_group)

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)

    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)

    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        self.dtime = msg.header.stamp.sec

    def altitude_set_pid(self, alt):
        self.Kp[1] = alt.kp
        self.Ki[1] = alt.ki
        self.Kd[1] = alt.kd

    def odometry_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        self.drone_position[3] = math.degrees(yaw)

    def pid(self):
        errors = [self.setpoint[i] - self.drone_position[i] for i in range(4)]
        outputs = []

        for i in range(4):
            self.integral_errors[i] += errors[i] * self.sample_time
            derivative_error = (errors[i] - self.prev_errors[i]) / self.sample_time
            output = (
                self.Kp[i] * errors[i]
                + self.Ki[i] * self.integral_errors[i]
                + self.Kd[i] * derivative_error
            )
            outputs.append(output)
            self.prev_errors[i] = errors[i]

        self.cmd.rc_throttle = max(1000, min(2000, 1500 + int(outputs[2])))
        self.cmd.rc_roll = max(1000, min(2000, 1500 + int(outputs[0])))
        self.cmd.rc_pitch = max(1000, min(2000, 1500 + int(outputs[1])))
        self.cmd.rc_yaw = 1500  # Keep yaw fixed

        self.command_pub.publish(self.cmd)
        self.pid_error_pub.publish(self.pid_error)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.setpoint[0] = goal_handle.request.waypoint.position.x
        self.setpoint[1] = goal_handle.request.waypoint.position.y
        self.setpoint[2] = goal_handle.request.waypoint.position.z
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.time_inside_sphere = 0

        feedback_msg = NavToWaypoint.Feedback()

        while True:
            feedback_msg.current_waypoint.pose.position.x = self.drone_position[0]
            feedback_msg.current_waypoint.pose.position.y = self.drone_position[1]
            feedback_msg.current_waypoint.pose.position.z = self.drone_position[2]
            goal_handle.publish_feedback(feedback_msg)

            drone_is_in_sphere = self.is_drone_in_sphere(self.drone_position, goal_handle, 0.4)

            if drone_is_in_sphere:
                if self.point_in_sphere_start_time is None:
                    self.point_in_sphere_start_time = self.dtime
                else:
                    self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time
            else:
                self.point_in_sphere_start_time = None

            if self.time_inside_sphere >= 3:
                break

        goal_handle.succeed()
        result = NavToWaypoint.Result()
        result.hov_time = self.dtime
        return result

    def is_drone_in_sphere(self, drone_pos, sphere_center, radius):
        return (
            (drone_pos[0] - sphere_center.request.waypoint.position.x) ** 2
            + (drone_pos[1] - sphere_center.request.waypoint.position.y) ** 2
            + (drone_pos[2] - sphere_center.request.waypoint.position.z) ** 2
        ) <= radius**2


def main(args=None):
    rclpy.init(args=args)
    waypoint_server = WayPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_server)
    
    try:
         executor.spin()
    except KeyboardInterrupt:
        pass

    waypoint_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
