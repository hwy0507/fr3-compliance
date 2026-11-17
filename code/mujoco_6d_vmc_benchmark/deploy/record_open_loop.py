"""Explicit hardware READ-ONLY entry point. Never imported by offline tests.

Construction and read_once live in a separate process with parent-side timeout.
This is NOT a 1 kHz controller or motion program. Do not run without a separate
on-site authorization. No load, frame, collision, recovery, or motion setters.
"""
import argparse
import importlib.metadata
import multiprocessing as mp
import time
from pathlib import Path

import numpy as np

from deploy.capture import capture_to_file, validate_request


def _worker(connection, robot_ip):
    try:
        # Only this explicitly launched worker imports the hardware binding.
        from pylibfranka import Robot
        robot = Robot(robot_ip)
        version = importlib.metadata.version("pylibfranka")
        while True:
            start = time.monotonic()
            state = robot.read_once()
            end = time.monotonic()
            config = {name: np.asarray(getattr(state, name)).tolist() for name in
                      ("F_T_EE", "F_T_NE", "NE_T_EE", "EE_T_K", "m_ee", "F_x_Cee",
                       "I_ee", "m_load", "F_x_Cload", "I_load", "m_total", "F_x_Ctotal", "I_total")}
            sample = {name: np.asarray(getattr(state, name), dtype=float).copy() for name in
                      ("q", "dq", "tau_J", "tau_ext_hat_filtered", "O_T_EE")}
            sample.update(robot_time_s=state.time.to_sec(),
                          control_command_success_rate=state.control_command_success_rate,
                          host_time_s=(start + end) / 2, read_duration_s=end - start,
                          status={name: str(getattr(state, name)) for name in
                                  ("robot_mode", "current_errors", "last_motion_errors")},
                          config=config, binding_version=version)
            connection.send(("sample", sample))
    except BaseException as error:
        try:
            connection.send(("error", f"{type(error).__name__}: {error}"))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        connection.close()


class IsolatedReader:
    def __init__(self, robot_ip):
        context = mp.get_context("spawn")
        self.connection, child = context.Pipe(duplex=False)
        self.process = context.Process(target=_worker, args=(child, robot_ip), daemon=True)
        self.process.start()
        child.close()

    def read(self, timeout):
        if not self.connection.poll(timeout):
            raise TimeoutError("robot connect/read deadline exceeded")
        kind, value = self.connection.recv()
        if kind != "sample":
            raise RuntimeError(value)
        return value

    def close(self):
        if self.process.is_alive():
            self.process.terminate()
        self.process.join(timeout=1)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=1)
        self.connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--read-timeout", type=float, default=2)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    validate_request(args.duration, args.read_timeout)
    capture_to_file(lambda: IsolatedReader(args.robot_ip), args.output,
                    args.duration, args.read_timeout)


if __name__ == "__main__":
    main()
