
"""Unit tests for waypoint navigation calculations."""

import math

from waypoint_navigation.controller import (
    calculate_command,
    normalize_angle,
)


def test_angle_normalization():
    """Verify that equivalent headings normalize near zero."""

    # Floating-point trigonometry introduces tiny rounding errors.
    # An absolute tolerance is appropriate when comparing against zero.
    assert math.isclose(
        normalize_angle(2 * math.pi),
        0.0,
        abs_tol=1e-12,
    )


def test_waypoint_reached():
    # Inside tolerance, the controller must stop.
    linear, angular, reached = calculate_command(
        x=0.0,
        y=0.0,
        yaw=0.0,
        goal_x=0.1,
        goal_y=0.0,
    )

    assert reached
    assert linear == 0.0
    assert angular == 0.0


def test_forward_motion():
    # When facing the target, move forward without turning.
    linear, angular, reached = calculate_command(
        x=0.0,
        y=0.0,
        yaw=0.0,
        goal_x=2.0,
        goal_y=0.0,
    )

    assert not reached
    assert linear > 0.0
    assert math.isclose(angular, 0.0)


def test_turn_toward_target():
    # Target to the left: rotate without forward motion.
    linear, angular, reached = calculate_command(
        x=0.0,
        y=0.0,
        yaw=0.0,
        goal_x=0.0,
        goal_y=2.0,
    )

    assert not reached
    assert linear == 0.0
    assert angular > 0.0


def test_velocity_limits():
    # Controller output must respect configured speed limits.
    linear, angular, _ = calculate_command(
        x=0.0,
        y=0.0,
        yaw=0.0,
        goal_x=100.0,
        goal_y=100.0,
        max_linear=0.3,
        max_angular=0.8,
    )

    assert abs(linear) <= 0.3
    assert abs(angular) <= 0.8
