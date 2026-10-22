
#!/usr/bin/env python3
"""Validate an experiment configuration for the Bash runner."""

import argparse
import re
from pathlib import Path
from typing import Any, Dict

import yaml


WORKSPACE = Path("/workspace/ros_ws")
IDENTIFIER_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")


def require_identifier(value: Any, field: str) -> str:
    """Validate a Gazebo world or robot identifier."""
    if not isinstance(value, str):
        raise ValueError(f"{field} must be a string.")

    if not IDENTIFIER_PATTERN.fullmatch(value):
        raise ValueError(f"Invalid {field}: {value}")

    return value


def load_config(path: Path) -> Dict[str, Any]:
    """Load and validate one experiment definition."""
    with path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    if not isinstance(config, dict):
        raise ValueError("Experiment configuration must be a mapping.")

    experiment = config["experiment"]
    simulation = config["simulation"]
    mission = config["mission"]
    evaluation = config["evaluation"]

    experiment_id = experiment["id"]
    if (
        not isinstance(experiment_id, str)
        or not re.fullmatch(r"EXP-[0-9]{4}", experiment_id)
    ):
        raise ValueError("Invalid experiment ID.")

    world = require_identifier(simulation["world"], "world")
    robot = require_identifier(simulation["robot"], "robot")

    timeout = mission["timeout_s"]
    if type(timeout) is not int or timeout <= 0:
        raise ValueError("Mission timeout must be a positive integer.")

    # Resolve the mission path and reject paths escaping the workspace.
    mission_path = (WORKSPACE / mission["config"]).resolve()

    if not mission_path.is_relative_to(WORKSPACE):
        raise ValueError("Mission file must remain inside the workspace.")

    if not mission_path.is_file():
        raise FileNotFoundError(f"Mission file missing: {mission_path}")

    enabled = evaluation["enabled"]
    if not isinstance(enabled, bool):
        raise ValueError("Evaluation enabled must be a boolean.")

    threshold = evaluation["max_final_position_error_m"]
    if isinstance(threshold, bool) or not isinstance(
        threshold, (int, float)
    ) or not 0 < threshold < float("inf"):
        raise ValueError("Final error threshold must be positive and finite.")

    return {
        "id": experiment_id,
        "world": world,
        "robot": robot,
        "mission": str(mission_path),
        "timeout": timeout,
        "evaluation_enabled": enabled,
        "max_final_error": float(threshold),
    }


def main() -> None:
    """Print validated fields in a fixed order for the Bash runner."""
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=Path)
    args = parser.parse_args()

    config = load_config(args.config)

    # Fixed-order lines avoid evaluating configuration as shell code.
    # Values containing newlines are excluded by input validation.
    values = [
        config["id"],
        config["world"],
        config["robot"],
        config["mission"],
        str(config["timeout"]),
        str(config["evaluation_enabled"]).lower(),
        str(config["max_final_error"]),
    ]

    print("\n".join(values))


if __name__ == "__main__":
    main()
