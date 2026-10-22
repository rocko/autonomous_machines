
#!/usr/bin/env python3
"""Evaluate one recorded ROS 2 waypoint mission.

Reads a rosbag2 recording containing nav_msgs/msg/Odometry,
loads the planned waypoints, and exports CSV, JSON, and PNG.

This script operates offline and does not control the robot.
"""

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Tuple

import matplotlib

# Headless rendering: no X11 display or Gazebo GUI required.
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ODOMETRY_TOPIC = "/model/vehicle_blue/odometry"


def detect_storage(bag_dir: Path) -> str:
    """Identify the storage plugin from the bag's file extension."""
    if list(bag_dir.glob("*.mcap")):
        return "mcap"

    if list(bag_dir.glob("*.db3")):
        return "sqlite3"

    raise FileNotFoundError(
        f"No MCAP or SQLite3 recording found in {bag_dir}"
    )


def read_odometry(
    bag_dir: Path,
) -> List[Dict[str, float]]:
    """Extract odometry measurements from a ROS 2 bag."""
    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_dir),
            storage_id=detect_storage(bag_dir),
        ),
        rosbag2_py.ConverterOptions("", ""),
    )

    topic_types = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }

    if ODOMETRY_TOPIC not in topic_types:
        raise ValueError(
            f"Odometry topic not found: {ODOMETRY_TOPIC}"
        )

    if topic_types[ODOMETRY_TOPIC] != "nav_msgs/msg/Odometry":
        raise ValueError("Unexpected odometry message type.")

    message_type = get_message(topic_types[ODOMETRY_TOPIC])
    samples: List[Dict[str, float]] = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic != ODOMETRY_TOPIC:
            continue

        msg = deserialize_message(data, message_type)

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        velocity = msg.twist.twist

        # Extract planar yaw from the orientation quaternion.
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y ** 2 + orientation.z ** 2
        )

        yaw = math.atan2(sin_yaw, cos_yaw)

        samples.append({
            "timestamp_ns": timestamp,
            "x": position.x,
            "y": position.y,
            "yaw": yaw,
            "linear_velocity": velocity.linear.x,
            "angular_velocity": velocity.angular.z,
        })

    if not samples:
        raise ValueError("No odometry samples were recorded.")

    # Ensure time increases monotonically in the output.
    samples.sort(key=lambda item: item["timestamp_ns"])

    first_timestamp = samples[0]["timestamp_ns"]

    for sample in samples:
        sample["time_s"] = (
            sample["timestamp_ns"] - first_timestamp
        ) / 1e9

    return samples


def read_waypoints(
    mission_path: Path,
) -> List[Tuple[float, float]]:
    """Load the flat coordinate array from the ROS parameter file."""
    with mission_path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    coordinates = config["waypoint_controller"]["ros__parameters"][
        "waypoints"
    ]

    if len(coordinates) < 2 or len(coordinates) % 2:
        raise ValueError("Mission must contain x,y pairs.")

    return [
        (float(coordinates[i]), float(coordinates[i + 1]))
        for i in range(0, len(coordinates), 2)
    ]


def calculate_metrics(
    samples: List[Dict[str, float]],
    waypoints: List[Tuple[float, float]],
) -> Dict[str, Any]:
    """Calculate descriptive metrics from recorded odometry."""
    path_length = 0.0

    for previous, current in zip(samples[:-1], samples[1:]):
        # Sum distances between consecutive odometry samples.
        path_length += math.hypot(
            current["x"] - previous["x"],
            current["y"] - previous["y"],
        )

    final_x, final_y = waypoints[-1]
    last = samples[-1]

    final_error = math.hypot(
        last["x"] - final_x,
        last["y"] - final_y,
    )

    duration = last["time_s"]

    return {
        "odometry_samples": len(samples),
        "recording_duration_s": round(duration, 3),
        "path_length_m": round(path_length, 3),
        "final_position_error_m": round(final_error, 3),
        "mean_path_speed_m_s": (
            round(path_length / duration, 3)
            if duration > 0.0 else 0.0
        ),
        "planned_waypoints": len(waypoints),
    }


def export_csv(
    samples: List[Dict[str, float]],
    output: Path,
) -> None:
    """Export odometry samples for subsequent experiments."""
    fieldnames = [
        "time_s",
        "x",
        "y",
        "yaw",
        "linear_velocity",
        "angular_velocity",
    ]

    with output.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()

        for sample in samples:
            writer.writerow({
                key: sample[key]
                for key in fieldnames
            })


def plot_trajectory(
    samples: List[Dict[str, float]],
    waypoints: List[Tuple[float, float]],
    output: Path,
) -> None:
    """Save actual odometry trajectory and planned waypoints."""
    actual_x = [sample["x"] for sample in samples]
    actual_y = [sample["y"] for sample in samples]

    waypoint_x = [point[0] for point in waypoints]
    waypoint_y = [point[1] for point in waypoints]

    fig, ax = plt.subplots(figsize=(8, 6))

    ax.plot(
        actual_x,
        actual_y,
        label="Recorded trajectory",
        linewidth=2,
    )

    # Connecting the waypoints illustrates the mission sequence,
    # not an actual planned collision-free path.
    ax.plot(
        waypoint_x,
        waypoint_y,
        "o--",
        label="Waypoint sequence",
    )

    for index, (x, y) in enumerate(waypoints, start=1):
        ax.annotate(
            f"WP{index}",
            (x, y),
            xytext=(6, 6),
            textcoords="offset points",
        )

    ax.scatter(
        [actual_x[0]],
        [actual_y[0]],
        marker="s",
        label="Recorded start",
    )

    ax.scatter(
        [actual_x[-1]],
        [actual_y[-1]],
        marker="x",
        label="Recorded finish",
    )

    ax.set_title("EXP-0007 - Autonomous waypoint navigation")
    ax.set_xlabel("X position [m]")
    ax.set_ylabel("Y position [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend()

    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main() -> None:
    """Parse arguments and generate the experiment artifacts."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    if not args.bag.is_dir():
        parser.error(f"Bag directory not found: {args.bag}")

    if not args.mission.is_file():
        parser.error(f"Mission file not found: {args.mission}")

    args.output.mkdir(parents=True, exist_ok=True)

    samples = read_odometry(args.bag)
    waypoints = read_waypoints(args.mission)
    metrics = calculate_metrics(samples, waypoints)

    export_csv(samples, args.output / "telemetry.csv")
    plot_trajectory(
        samples,
        waypoints,
        args.output / "trajectory.png",
    )

    with (args.output / "metrics.json").open(
        "w", encoding="utf-8"
    ) as file:
        json.dump(metrics, file, indent=2)

    print(json.dumps(metrics, indent=2))
    print(f"Results saved to: {args.output}")


if __name__ == "__main__":
    main()
