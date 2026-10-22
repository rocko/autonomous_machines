
"""Pure navigation calculations without ROS dependencies."""

import math
from typing import Tuple


def normalize_angle(angle: float) -> float:
    """Normalize an angle to the interval [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def calculate_command(
    x: float,
    y: float,
    yaw: float,
    goal_x: float,
    goal_y: float,
    max_linear: float = 0.3,
    max_angular: float = 0.8,
    arrival_tolerance: float = 0.2,
) -> Tuple[float, float, bool]:
    """Calculate linear and angular velocity toward one waypoint.

    The controller first aligns the robot with its target, then
    drives forward while correcting smaller heading errors.

    Returns:
        Linear velocity, angular velocity, waypoint reached.
    """
    dx = goal_x - x
    dy = goal_y - y

    distance = math.hypot(dx, dy)

    # A waypoint counts as reached within its position tolerance.
    if distance <= arrival_tolerance:
        return 0.0, 0.0, True

    desired_heading = math.atan2(dy, dx)
    heading_error = normalize_angle(desired_heading - yaw)

    # Proportional steering with a maximum angular velocity.
    angular = max(
        -max_angular,
        min(max_angular, 1.5 * heading_error),
    )

    # Avoid driving forward while facing far away from the goal.
    if abs(heading_error) > 0.5:
        linear = 0.0
    else:
        # Slow down as the robot approaches the waypoint.
        linear = min(max_linear, 0.8 * distance)

    return linear, angular, False
