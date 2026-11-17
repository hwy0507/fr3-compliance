"""Offline teaching review and slowed visualization CSV; NEVER robot commands.

This deliberately exports observed samples only, not an executable control plan.
No smoothing, guessed grasp force, automatic return-to-start, or trajectory repair.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from deploy.recording import load_recording
from deploy.record_teaching import validate_hand, READ_ONLY_TEACHING_MODES
from deploy.teaching_reference import check_reference_packet


def review_session(directory):
    directory = Path(directory)
    receipt = json.loads((directory / "session.json").read_text())
    if receipt.get("schema") != "fr3_teaching_readonly_v1" or receipt.get("complete") is not True:
        raise ValueError("incomplete or unsupported teaching session")
    for name in ("arm.npz", "hand.jsonl"):
        data = (directory / name).read_bytes()
        expected = receipt["files"][name]
        if len(data) != expected["bytes"] or hashlib.sha256(data).hexdigest() != expected["sha256"]:
            raise ValueError("recording hash mismatch")
    arm = load_recording(directory / "arm.npz", max_gap_s=.05)
    if arm["analysis_only"]:
        raise ValueError("partial recordings cannot be taught paths")
    hand = [json.loads(line) for line in (directory / "hand.jsonl").read_text().splitlines()]
    if len(hand) < 2 or len(hand) != receipt.get("hand_samples"):
        raise ValueError("missing Hand stream")
    previous = None
    for packet in hand:
        validate_hand(packet, previous)
        previous = packet
    modes = []
    for value in arm["sample_metadata"]:
        s = json.loads(value)
        if (s["robot_mode"] not in READ_ONLY_TEACHING_MODES
                or s["current_errors"] != "[]" or s["last_motion_errors"] != "[]"):
            raise ValueError("recording includes non-guiding mode or errors")
        modes.append(s["robot_mode"])
    t, q = arm["robot_time_s"] - arm["robot_time_s"][0], arm["q"]
    reference = receipt.get('start_reference')
    if reference is not None:
        check_reference_packet(dict(q=q[0], dq=arm['dq'][0], config=arm['metadata']['config']), reference)
    first = receipt["arm_first_host_time_s"]
    last = receipt["arm_last_host_time_s"]
    if not np.isfinite([first, last]).all() or abs(last - first - arm["time_s"][-1]) > 1e-6:
        raise ValueError("invalid arm host time anchor")
    ht = np.array([x["host_time_s"] for x in hand])
    blocks = []
    if "RobotMode.Guiding" not in modes:
        blocks.append("no Guiding state observed; manual trajectory not established")
    if ht[0] > first or ht[-1] < last - .2 or np.diff(ht).max() > .2:
        blocks.append("Hand stream gaps/coverage require review")
    if t[-1] < 3:
        blocks.append("insufficient recorded duration")
    for name, mask in (("start", t <= .5), ("end", t >= t[-1] - .5)):
        if (np.max(np.ptp(q[mask], axis=0)) > .001
                or np.max(np.abs(arm["dq"][mask])) > .01):
            blocks.append(f"{name} not stationary for 0.5 s")
    excursion = np.ptp(q, axis=0)
    if np.any(excursion > np.deg2rad(15) + 1e-6):
        blocks.append("joint excursion exceeds prior 15-degree scope")
    j1_positive = float(np.max(q[:, 0] - q[0, 0]))
    if j1_positive > .001:
        blocks.append("J1 moves positive toward the previously reported computer obstacle")
    transitions = []
    for before, after in zip(hand[:-1], hand[1:]):
        if before["is_grasped"] != after["is_grasped"]:
            transitions.append(dict(host_time_from_arm_start_s=after["host_time_s"] - first,
                                    is_grasped=after["is_grasped"], width_m=after["width_m"],
                                    interpretation="observed state transition; not an inferred command"))
    report = dict(schema="fr3_teaching_review_v1", hardware_replay_ready=False,
                  data_review_flags=blocks, sample_count=len(t), hand_sample_count=len(hand),
                  duration_robot_s=float(t[-1]), arm_max_gap_s=float(np.diff(t).max()),
                  modes=sorted(set(modes)), mode_sample_counts={mode:modes.count(mode) for mode in set(modes)},
                  joint_excursion_deg=np.rad2deg(excursion).tolist(),
                  j1_max_positive_from_start_deg=float(np.rad2deg(j1_positive)),
                  start_q_rad=q[0].tolist(), end_q_rad=q[-1].tolist(),
                  start_reference=reference,
                  initial_hand_is_grasped=hand[0]["is_grasped"],
                  final_hand_is_grasped=hand[-1]["is_grasped"],
                  hand_width_range_m=[min(x["width_m"] for x in hand), max(x["width_m"] for x in hand)],
                  hand_state_transitions=transitions,
                  robot_config=arm["metadata"]["config"],
                  unresolved=["review whole-arm/tool/camera/cable path against current scene",
                              "verify payload and grasp/release force, width and support conditions",
                              "fit and validate smooth retimed trajectory, acceleration and jerk",
                              "robot-reported directional velocity/zero-hold limits and limiter preview",
                              "fresh stationary start match; never automatically return to first point",
                              "separate approval and bounded supervised hardware execution"],
                  note="read-only data is not a collision, force, or hardware safety certificate")
    return report, arm, hand


def export_review(directory, output, slowdown=2.):
    if not np.isfinite(slowdown) or not 1 <= slowdown <= 10:
        raise ValueError("visualization slowdown must be 1..10")
    report, arm, hand = review_session(directory)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    report["visualization_time_scale"] = slowdown
    with (output / "review.json").open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    with (output / "observed_path_visualization_only.csv").open("x", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["recorded_robot_t_s", "visualization_t_s"] + [f"q{j}_rad" for j in range(1, 8)]
                        + ["ee_x_m", "ee_y_m", "ee_z_m"])
        t = arm["robot_time_s"] - arm["robot_time_s"][0]
        for i, ti in enumerate(t):
            writer.writerow([ti, slowdown * ti, *arm["q"][i], *arm["O_T_EE"][i, 12:15]])
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--slowdown", type=float, default=2.)
    args = parser.parse_args()
    print(json.dumps(export_review(args.session, args.output, args.slowdown), indent=2))


if __name__ == "__main__":
    main()
