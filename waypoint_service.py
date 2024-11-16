import rclpy
from rclpy.node import Node
from waypoint_navigation.srv import GetWaypoints

class WaypointService(Node):
    def __init__(self):
        super().__init__('waypoint_service')
        self.srv = self.create_service(GetWaypoints, 'get_waypoints', self.get_waypoints_callback)
        self.get_logger().info("Waypoint Service started")

    def get_waypoints_callback(self, request, response):
        response.waypoints_x = [1.0, 2.0, 3.0, 4.0, 5.0]
        response.waypoints_y = [1.0, 2.0, 3.0, 4.0, 5.0]
        response.waypoints_z = [2.0, 2.0, 2.0, 2.0, 2.0]
        return response

def main(args=None):
    rclpy.init(args=args)
    waypoint_service = WaypointService()
    rclpy.spin(waypoint_service)
    rclpy.shutdown()
