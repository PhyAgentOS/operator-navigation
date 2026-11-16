"""YAML/environment configuration, validated before opening ROS or hardware."""

from __future__ import annotations

import math
import os
from pathlib import Path

import yaml

CONFIG_ENV_VAR = "NAVIGATION_CONFIG"
REQUIRED = [
    "ros_domain_id",
    "session_dir",
    "odometry_topic",
    "safe_velocity_topic",
    "max_vx_mps",
    "max_wz_radps",
    "velocity_timeout_seconds",
    "backend_params",
    "goal_timeout_seconds",
    "require_motion_evidence",
]


def load_config(path: str | Path | None = None) -> dict:
    selected = path or os.environ.get(CONFIG_ENV_VAR)
    if not selected:
        raise ValueError(f"use --config or {CONFIG_ENV_VAR}")
    source = Path(selected).expanduser().resolve()
    value = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("configuration must be a mapping")
    value.setdefault("launch_file", str(Path(__file__).with_name("backend.launch.py")))
    if value.get("runtime_status_output", "status") not in ("status", "runtime"):
        raise ValueError("runtime_status_output must be status or runtime")
    missing = [key for key in REQUIRED if key not in value]
    if missing:
        raise ValueError(f"missing configuration fields: {', '.join(missing)}")
    domain = value["ros_domain_id"]
    if isinstance(domain, bool) or not isinstance(domain, int) or not 0 <= domain <= 232:
        raise ValueError("ros_domain_id must be an integer in 0..232")
    for key, item in value.items():
        if key.endswith(("_seconds", "_hz", "_mps", "_radps")):
            if (
                isinstance(item, bool)
                or not isinstance(item, (float, int))
                or not math.isfinite(item)
                or item <= 0
            ):
                raise ValueError(f"{key} must be finite and positive")
        if key.endswith("_topic") and (not isinstance(item, str) or not item.startswith("/")):
            raise ValueError(f"{key} must be an absolute ROS topic")
        if key in {
            "deployment_root",
            "backend_executable",
            "backend_params",
            "session_dir",
            "map_file",
            "pcd_path",
            "pose_cache_file",
            "snapshot_dir",
        }:
            if not isinstance(item, str) or not item.strip():
                raise ValueError(f"{key} must be a path")
            target = Path(os.path.expandvars(item)).expanduser()
            value[key] = str(
                (source.parent / target).resolve() if not target.is_absolute() else target
            )
    fusion = value.get("map_fusion", {})
    if not isinstance(fusion, dict) or type(fusion.get("enabled", False)) is not bool:
        raise ValueError("map_fusion must be a mapping with a boolean enabled flag")
    if fusion.get("enabled"):
        memory = fusion.setdefault("dynamic_memory_seconds", 0.0)
        if type(memory) not in (float, int) or not math.isfinite(memory) or not 0 <= memory <= 120:
            raise ValueError("map_fusion.dynamic_memory_seconds must be finite in 0..120")
        for key in ("prior_topic", "output_topic", "scan_topic"):
            if not isinstance(fusion.get(key), str) or not fusion[key].startswith("/"):
                raise ValueError(f"map_fusion.{key} must be an absolute ROS topic")
        if fusion["prior_topic"] == fusion["output_topic"]:
            raise ValueError("map_fusion must not overwrite the localization prior")
        if not isinstance(fusion.get("odom_frame"), str) or not fusion["odom_frame"]:
            raise ValueError("map_fusion.odom_frame is required")
        for key in (
            "observation_range_m",
            "clear_min_seconds",
            "free_memory_seconds",
            "endpoint_margin_m",
        ):
            item = fusion.get(key)
            if type(item) not in (float, int) or not math.isfinite(item) or item <= 0:
                raise ValueError(f"map_fusion.{key} must be finite and positive")
        if (
            type(fusion.get("clear_confirmations")) is not int
            or not 2 <= fusion["clear_confirmations"] <= 100
        ):
            raise ValueError("map_fusion.clear_confirmations must be an integer in 2..100")
    return value
