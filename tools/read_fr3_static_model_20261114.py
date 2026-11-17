"""Bounded-count read-only static diagnostic; use an external 15 s process timeout.

Hardware API allowlist: Robot(...), load_model(), read_once().
Model.gravity/coriolis/zero_jacobian only compute from measured state locally.
No control, recovery, gripper, configuration setters, or state-field writes.
Raw data stay outside Git. This is not a mass-identification or motion tool.
"""
import argparse
import importlib.metadata
import json
import time
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive destination exists before any hardware object is constructed.
    with args.output.open("x") as output:
        samples = []
        reason = "not_started"
        primary_error = None
        try:
            from pylibfranka import Robot
            robot = Robot(args.robot_ip)
            model = robot.load_model()
            deadline = time.monotonic() + 3.
            for _ in range(100):
                if time.monotonic() >= deadline:
                    raise TimeoutError("diagnostic read loop exceeded 3 s")
                state = robot.read_once()
                if str(state.robot_mode) != "RobotMode.Idle":
                    raise ValueError("static diagnostic requires Idle; no mode changes attempted")
                if np.max(np.abs(state.dq)) > .02:
                    raise ValueError("motion observed; stopped reading without issuing a robot command")
                fields = {name: np.asarray(getattr(state, name), dtype=float).tolist() for name in
                          ("q", "dq", "tau_J", "tau_ext_hat_filtered", "O_T_EE", "F_T_EE",
                           "O_F_ext_hat_K", "K_F_ext_hat_K", "O_ddP_O", "joint_contact",
                           "cartesian_contact", "joint_collision", "cartesian_collision",
                           "m_total", "F_x_Ctotal", "I_total")}
                fields.update(robot_time_s=state.time.to_sec(), robot_mode=str(state.robot_mode),
                              current_errors=str(state.current_errors),
                              last_motion_errors=str(state.last_motion_errors),
                              gravity=np.asarray(model.gravity(state)).tolist(),
                              coriolis=np.asarray(model.coriolis(state)).tolist(),
                              zero_jacobian=np.asarray(model.zero_jacobian(state)).tolist())
                samples.append(fields)
            reason = "sample_count_reached"
        except BaseException as error:
            primary_error = error
            reason = f"{type(error).__name__}:{error}"
            raise
        finally:
            try:
                json.dump(dict(complete=reason == "sample_count_reached", stop_reason=reason,
                               binding_version=importlib.metadata.version("pylibfranka"),
                               hardware_commands_sent=False, parameters_written=False,
                               samples=samples), output, indent=2, allow_nan=False)
                output.write("\n")
            except BaseException:
                if primary_error is None:
                    raise


if __name__ == "__main__":
    main()
