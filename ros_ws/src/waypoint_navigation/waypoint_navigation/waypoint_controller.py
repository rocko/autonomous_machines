
"""ROS 2 waypoint-following controller for a simulated rover."""

import math
import time
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from waypoint_navigation.controller import calculate_command


class WaypointController(Node):
    """Follow predefined waypoints using simulated odometry."""

    def __init__(self) -> None:
        super().__init__("waypoint_controller")

        # Mission parameters are supplied through a ROS YAML file.
        self.declare_parameter("waypoints", [0.0, 0.0])
        self.declare_parameter("arrival_tolerance", 0.2)
        self.declare_parameter("max_linear_velocity", 0.3)
        self.declare_parameter("max_angular_velocity", 0.8)
        self.declare_parameter("odometry_timeout", 2.0)

        coordinates = [
            float(value)
            for value in self.get_parameter("waypoints").value
        ]

        if len(coordinates) < 2 or len(coordinates) % 2 != 0:
            raise ValueError(
                "Waypoints must contain x,y coordinate pairs."
            )

        # Convert the flat parameter array into (x, y) tuples.
        self.waypoints: List[Tuple[float, float]] = [
            (coordinates[i], coordinates[i + 1])
            for i in range(0, len(coordinates), 2)
        ]

        self.tolerance = float(
            self.get_parameter("arrival_tolerance").value
        )
        self.max_linear = float(
            self.get_parameter("max_linear_velocity").value
        )
        self.max_angular = float(
            self.get_parameter("max_angular_velocity").value
        )
        self.odom_timeout = float(
            self.get_parameter("odometry_timeout").value
        )

        # Fail early on invalid mission parameters.
        if (
            self.tolerance <= 0.0
            or self.max_linear <= 0.0
            or self.max_angular <= 0.0
            or self.odom_timeout <= 0.0
        ):
            raise ValueError("Controller parameters must be positive.")

        self.current_waypoint = 0
        self.last_odometry_time: float = 0.0
        self.pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.has_odometry = False
        self.mission_complete = False

        # The controller publishes commands at 10 Hz.
        self.command_pub = self.create_publisher(
            Twist,
            "/model/vehicle_blue/cmd_vel",
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/model/vehicle_blue/odometry",
            self.on_odometry,
            10,
        )

        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info(
            f"Mission loaded: {len(self.waypoints)} waypoints."
        )

    def on_odometry(self, msg: Odometry) -> None:
        """Extract planar position and heading from odometry."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Convert the orientation quaternion to yaw.
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )

        cos_yaw = 1.0 - 2.0 * (
            orientation.y ** 2 + orientation.z ** 2
        )

        yaw = math.atan2(sin_yaw, cos_yaw)

        self.pose = (position.x, position.y, yaw)
        self.last_odometry_time = time.monotonic()
        self.has_odometry = True

    def publish_velocity(
        self,
        linear: float,
        angular: float,
    ) -> None:
        """Publish a planar velocity command."""
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.command_pub.publish(command)

    def stop(self) -> None:
        """Request zero linear and angular velocity."""
        self.publish_velocity(0.0, 0.0)

    def update(self) -> None:
        """Advance the mission using the latest odometry."""
        if self.mission_complete:
            self.stop()
            return

        # Do not move until valid odometry is available.
        if not self.has_odometry:
            self.stop()
            return

        # Never continue using stale sensor measurements.
        if (
            time.monotonic() - self.last_odometry_time
            > self.odom_timeout
        ):
            self.stop()
            return

        x, y, yaw = self.pose
        goal_x, goal_y = self.waypoints[self.current_waypoint]

        linear, angular, reached = calculate_command(
            x=x,
            y=y,
            yaw=yaw,
            goal_x=goal_x,
            goal_y=goal_y,
            max_linear=self.max_linear,
            max_angular=self.max_angular,
            arrival_tolerance=self.tolerance,
        )

        if reached:
            self.stop()
            self.get_logger().info(
                f"Waypoint {self.current_waypoint + 1} reached."
            )

            self.current_waypoint += 1

            if self.current_waypoint >= len(self.waypoints):
                self.mission_complete = True
                self.get_logger().info("Mission complete.")
            return

        self.publish_velocity(linear, angular)


def main(args=None) -> None:
    """Initialize ROS 2 and execute the waypoint controller."""
    rclpy.init(args=args)
    node = None

    try:
        node = WaypointController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # Attempt a clean stop before closing the publisher.
            for _ in range(3):
                node.stop()
                time.sleep(0.05)
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
