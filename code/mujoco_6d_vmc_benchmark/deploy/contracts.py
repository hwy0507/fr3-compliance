"""Offline contracts. No robot imports, network, or hardware parameter writes."""
import json
from pathlib import Path

import numpy as np


def finite_vector(value, size, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError(f"{name} must be a finite {size}-vector")
    return value.copy()


def policy_action(raw, gain=1.0):
    # Check BEFORE clipping: np.clip(inf, -1, 1) would hide a broken policy.
    raw = finite_vector(raw, 7, "raw policy action")
    if not np.isfinite(gain) or not 0 < gain <= 1:
        raise ValueError("action gain must be finite and in (0, 1]")
    return np.clip(raw * gain, -1, 1)


def positive_seconds(value, name, maximum=None):
    if not np.isfinite(value) or value <= 0 or (maximum is not None and value > maximum):
        raise ValueError(f"invalid {name}: expected finite positive seconds <= {maximum}")
    return float(value)


def rigid_transform(value, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError(f"invalid {name}")
    r = value[:3, :3]
    if (not np.allclose(value[3], [0, 0, 0, 1], atol=1e-6, rtol=0)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-5, rtol=0)
            or not np.isclose(np.linalg.det(r), 1, atol=1e-5, rtol=0)):
        raise ValueError(f"{name} must be a proper rigid transform")
    return value.copy()


def load_frames(path):
    data = json.loads(Path(path).read_text())
    if data.get("verified") is not True or not data.get("evidence"):
        raise ValueError("frame mapping requires verification evidence; no identity default")
    return {key: rigid_transform(data[key], key) for key in ("W_T_O", "EE_T_hand")}


def mapped_hand_pose(flat_o_t_ee, frames):
    from deploy.recording import decode_pose
    return frames["W_T_O"] @ decode_pose(flat_o_t_ee) @ frames["EE_T_hand"]


def validate_model_alignment(backend, measured_pose, position_tolerance_m=0.005,
                             rotation_tolerance_rad=0.02):
    """Diagnostic rejection threshold, NOT a hardware acceptance certificate."""
    position_error = np.linalg.norm(backend.hand_position - measured_pose[:3, 3])
    relative = backend.hand_rotation.T @ measured_pose[:3, :3]
    angle = np.arccos(np.clip((np.trace(relative) - 1) / 2, -1, 1))
    if position_error > position_tolerance_m or angle > rotation_tolerance_rad:
        raise ValueError(f"model/frame mismatch: {position_error:.6f} m, {angle:.6f} rad")


def save_npz_exclusive(path, **fields):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        np.savez_compressed(stream, **fields)


def validate_robot_config(config):
    """Validate recorded config shape/units only, never endorse physical values."""
    from deploy.recording import decode_pose
    for key in ("F_T_EE", "F_T_NE", "NE_T_EE", "EE_T_K"):
        decode_pose(config[key])
    for key in ("F_x_Cee", "F_x_Cload", "F_x_Ctotal"):
        finite_vector(config[key], 3, key)
    for key in ("I_ee", "I_load", "I_total"):
        finite_vector(config[key], 9, key)
    for key in ("m_ee", "m_load", "m_total"):
        value = config[key]
        if np.ndim(value) != 0 or not np.isfinite(value) or value < 0:
            raise ValueError(f"invalid recorded {key}")
