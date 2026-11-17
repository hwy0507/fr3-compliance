"""Offline-only static recording analysis. No hardware imports or network."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code/mujoco_6d_vmc_benchmark"))
from deploy.recording import load_recording

# FR3 nominal bounds, not a replacement for robot safety configuration.
# https://frankarobotics.github.io/docs/robot_specifications.html (2026-11-14)
LOWER = np.array([-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, .4398, -3.0508])
UPPER = np.array([2.9007, 1.8361, 2.9007, -.1169, 2.8763, 4.6216, 3.0508])


def stats(values):
    values = np.asarray(values, dtype=float)
    if not values.size or not np.isfinite(values).all():
        raise ValueError("empty/nonfinite diagnostic data")
    return {"mean": values.mean(axis=0).tolist(), "std": values.std(axis=0).tolist(),
            "peak_to_peak": np.ptp(values, axis=0).tolist(),
            "max_abs": np.max(np.abs(values), axis=0).tolist()}


def analyze(recording, model_diagnostic):
    data = load_recording(recording)
    with np.load(recording, allow_pickle=False) as archive:
        external = archive["tau_ext_hat_filtered"].copy()
    if external.shape != data["q"].shape:
        raise ValueError("external torque samples misaligned")
    diagnostic = json.loads(Path(model_diagnostic).read_text())
    if diagnostic["complete"] is not True or diagnostic["stop_reason"] != "sample_count_reached":
        raise ValueError("incomplete supplementary diagnostic")
    samples = diagnostic["samples"]
    if len(samples) != 100:
        raise ValueError("expected 100 supplementary samples")
    arrays = {key: np.asarray([s[key] for s in samples], dtype=float) for key in
              ("q", "dq", "tau_J", "gravity", "coriolis", "tau_ext_hat_filtered",
               "joint_contact", "cartesian_contact", "joint_collision", "cartesian_collision")}
    for key, value in arrays.items():
        if not np.isfinite(value).all():
            raise ValueError(f"nonfinite diagnostic {key}")
    q = data["q"].mean(axis=0)
    config = data["metadata"]["config"]
    for s in samples:
        for key in ("m_total", "F_x_Ctotal", "I_total", "F_T_EE"):
            if not np.allclose(s[key], config[key], atol=1e-8, rtol=0):
                raise ValueError(f"configuration changed between captures: {key}")
    robot_dt = np.diff(data["robot_time_s"])
    interval_ms, counts = np.unique(np.rint(robot_dt * 1000).astype(int), return_counts=True)
    residual = arrays["tau_J"] - arrays["gravity"] - arrays["coriolis"]
    report = dict(
        analysis_scope="static capture files only; preceding robot motion, if any, is recorded separately",
        hardware_qualified=False, payload_identified=False, motion_command_sent=False,
        robot_parameters_written=False,
        primary_samples=len(data["q"]), primary_robot_span_s=float(np.ptp(data["robot_time_s"])),
        robot_interval_counts_ms={str(int(k)): int(v) for k, v in zip(interval_ms, counts)},
        robot_interval_max_s=float(robot_dt.max()),
        host_read_duration_quantiles_s=np.quantile(data["read_duration_s"], [.5, .99, 1]).tolist(),
        command_success_rate_unique=np.unique(data["control_command_success_rate"]).tolist(),
        command_success_rate_interpretation="No command loop was running; this is not a command-link success/failure test.",
        state_status_unique=sorted(set(data["sample_metadata"].tolist())),
        q_rad=stats(data["q"]), q_mean_deg=np.rad2deg(q).tolist(),
        q_peak_to_peak_deg=np.rad2deg(np.ptp(data["q"], axis=0)).tolist(),
        joint_nearest_limit_margin_deg=np.rad2deg(np.minimum(q - LOWER, UPPER - q)).tolist(),
        nearest_limit_side=["lower" if a < b else "upper" for a, b in zip(q - LOWER, UPPER - q)],
        dq_radps=stats(data["dq"]), tau_measured_nm=stats(data["tau_J"]),
        estimated_external_joint_torque_nm=stats(external),
        tcp_position_peak_to_peak_mm=(1000 * np.ptp(data["O_T_EE"][:, 12:15], axis=0)).tolist(),
        configured_tool=config, binding_version=data["metadata"]["binding_version"],
        diagnostic_samples=100,
        diagnostic_status_unique=sorted(set(json.dumps({k: s[k] for k in ("robot_mode", "current_errors", "last_motion_errors")}) for s in samples)),
        diagnostic_model_gravity_nm=stats(arrays["gravity"]),
        diagnostic_measured_minus_gravity_minus_coriolis_nm=stats(residual),
        diagnostic_filtered_external_torque_nm=stats(arrays["tau_ext_hat_filtered"]),
        model_residual_minus_filtered_external_mean_nm=(residual-arrays["tau_ext_hat_filtered"]).mean(0).tolist(),
        diagnostic_contact_collision_max={key: np.max(value, axis=0).tolist() for key, value in arrays.items() if "contact" in key or "collision" in key},
        mean_pose_change_between_captures_rad=(arrays["q"].mean(0) - q).tolist(),
        capture_start_separation_robot_s=samples[0]["robot_time_s"]-float(data["robot_time_s"][0]),
        limitations=["One unchanged pose does not identify mass, COM and inertia together.",
                     "Filtered external torque is model-dependent, not an independent force sensor.",
                     "No contact flag does not prove physical absence of contact.",
                     "Torque residual can include tool/load, robot-model, sensor-offset, friction and cable effects.",
                     "A static read-only capture does not certify FCI real-time control or safe motion."],
        raw_files={str(path.resolve()): {"bytes": path.stat().st_size,
                                       "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                   for path in (Path(recording), Path(model_diagnostic))})
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", required=True, type=Path)
    parser.add_argument("--model-diagnostic", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = analyze(args.recording, args.model_diagnostic)
    with args.output.open("x") as output:
        json.dump(result, output, indent=2, allow_nan=False)
        output.write("\n")
    print(json.dumps({k: result[k] for k in ("primary_samples", "primary_robot_span_s",
                     "robot_interval_counts_ms", "q_mean_deg", "joint_nearest_limit_margin_deg",
                     "diagnostic_measured_minus_gravity_minus_coriolis_nm")}, indent=2))


if __name__ == "__main__":
    main()
