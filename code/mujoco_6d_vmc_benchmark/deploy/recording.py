"""Validated libfranka recordings for offline analysis; no robot commands."""
from __future__ import annotations

import json

import numpy as np


def decode_pose(flat: np.ndarray) -> np.ndarray:
    """libfranka stores O_T_EE in column-major order."""
    flat = np.asarray(flat, dtype=float)
    if flat.shape != (16,) or not np.all(np.isfinite(flat)):
        raise ValueError("O_T_EE must contain 16 finite values")
    pose = flat.reshape((4, 4), order="F")
    rotation = pose[:3, :3]
    if (not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-6, rtol=0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5, rtol=0)):
        raise ValueError("O_T_EE must encode a proper homogeneous rigid transform")
    return pose.copy()


def load_recording(path, *, allow_legacy=False, allow_partial=False, max_gap_s=0.01):
    """Reject malformed streams instead of repairing timestamps silently.

    Strictly increasing host timestamps alone do not prove 1 kHz sampling,
    unique robot states, or acceptable latency; those require robot-time data.
    """
    from deploy.contracts import positive_seconds, validate_robot_config
    positive_seconds(max_gap_s, "maximum robot sample gap")
    with np.load(path, allow_pickle=False) as archive:
        schema = int(archive["schema_version"].item()) if "schema_version" in archive else 0
        if schema not in (0, 2):
            raise ValueError("unsupported recording schema")
        if schema != 2 and not allow_legacy:
            raise ValueError("legacy recording: explicit analysis-only opt-in required")
        complete = ("complete" in archive and archive["complete"].shape == ()
                    and archive["complete"].dtype.kind == "b" and archive["complete"].item() is True)
        interrupted = bool(archive["interrupted"].item()) if "interrupted" in archive else True
        reason = str(archive["stop_reason"].item()) if "stop_reason" in archive else "unknown"
        if not allow_partial and (not complete or interrupted or reason != "duration_elapsed"):
            raise ValueError("incomplete/interrupted recording: explicit analysis-only opt-in required")
        fields = {key: np.asarray(archive[key], dtype=float).copy()
                  for key in ("time_s", "q", "dq", "tau_J", "O_T_EE")}
        if "read_duration_s" in archive:
            fields["read_duration_s"] = np.asarray(archive["read_duration_s"], dtype=float).copy()
        if schema == 2:
            fields["robot_time_s"] = np.asarray(archive["robot_time_s"], dtype=float).copy()
            fields["control_command_success_rate"] = np.asarray(archive["control_command_success_rate"], dtype=float).copy()
            fields["sample_metadata"] = archive["sample_metadata"].astype(str).copy()
            metadata = json.loads(str(archive["metadata_json"].item()))
            if not metadata.get("config") or not metadata.get("binding_version"):
                raise ValueError("missing robot configuration/binding metadata")
            validate_robot_config(metadata["config"])
            fields["metadata"] = metadata
        fields["analysis_only"] = schema != 2 or not complete or interrupted or reason != "duration_elapsed"
    times = fields["time_s"]
    if times.ndim != 1 or len(times) < 10 or not np.all(np.isfinite(times)):
        raise ValueError("recording requires at least 10 finite timestamps")
    if np.any(np.diff(times) <= 0):
        raise ValueError("recording timestamps must be strictly increasing")
    for key, width in (("q", 7), ("dq", 7), ("tau_J", 7), ("O_T_EE", 16)):
        values = fields[key]
        if values.shape != (len(times), width) or not np.all(np.isfinite(values)):
            raise ValueError(f"invalid recording field {key}")
    for pose in fields["O_T_EE"]:
        decode_pose(pose)
    if "read_duration_s" in fields:
        read_duration = fields["read_duration_s"]
        if (read_duration.shape != times.shape or not np.all(np.isfinite(read_duration))
                or np.any(read_duration < 0)):
            raise ValueError("invalid recording field read_duration_s")
    if "robot_time_s" in fields:
        rt = fields["robot_time_s"]
        if rt.shape != times.shape or not np.isfinite(rt).all() or np.any(np.diff(rt) <= 0):
            raise ValueError("robot timestamps must be finite, unique and increasing")
        if np.any(np.diff(rt) > max_gap_s + 1e-12):
            raise ValueError("robot sample gap exceeds offline replay budget")
        rate = fields["control_command_success_rate"]
        if rate.shape != times.shape or not np.isfinite(rate).all() or np.any((rate < 0) | (rate > 1)):
            raise ValueError("invalid control command success rate")
        if fields["sample_metadata"].shape != times.shape:
            raise ValueError("missing per-sample robot status")
        for item in fields["sample_metadata"]:
            status = json.loads(item)
            if any(key not in status for key in ("robot_mode", "current_errors", "last_motion_errors")):
                raise ValueError("incomplete robot status")
    return fields


def sampling_summary(time_s: np.ndarray, read_duration_s: np.ndarray | None = None) -> dict:
    """Describe host-observed intervals without asserting FCI timing quality."""
    dt = np.diff(np.asarray(time_s, dtype=float))
    if dt.size == 0 or not np.all(np.isfinite(dt)) or np.any(dt <= 0):
        raise ValueError("sampling summary requires increasing finite timestamps")
    summary = {
        "interval_median_s": float(np.median(dt)),
        "interval_p99_s": float(np.quantile(dt, 0.99)),
        "interval_max_s": float(np.max(dt)),
        "host_observed_rate_hz": float(1.0 / np.median(dt)),
        "interpretation": "host read_once cadence only; not robot-time or FCI deadline evidence",
    }
    if read_duration_s is not None:
        duration = np.asarray(read_duration_s, dtype=float)
        if (duration.shape != (dt.size + 1,) or not np.all(np.isfinite(duration))
                or np.any(duration < 0)):
            raise ValueError("read duration samples must align with timestamps")
        summary["read_call_duration_p99_s"] = float(np.quantile(duration, 0.99))
        summary["read_call_duration_max_s"] = float(np.max(duration))
    return summary
