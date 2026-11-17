"""Explicit READ-ONLY arm + Hand teaching recorder. No motion or parameter API.

Arm samples use the existing bounded schema-2 recorder. Hand reads run in a
separate process, nominally at 20 Hz; host timestamps are shared monotonic time,
NOT synchronized device clocks. Failure of either stream invalidates the session.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import multiprocessing as mp
from pathlib import Path
import time

import numpy as np

from deploy.capture import capture_to_file, validate_request
from deploy.record_open_loop import IsolatedReader
from deploy.teaching_reference import load_start_reference, check_reference_packet

# These states may be OBSERVED by this read-only recorder. This is deliberately
# not a control-start allowlist: UserStopped must never enable/recover motion.
READ_ONLY_TEACHING_MODES = ("RobotMode.Idle", "RobotMode.Guiding", "RobotMode.UserStopped")


def _hand_worker(connection, robot_ip):
    try:
        from pylibfranka import Gripper
        hand = Gripper(robot_ip)
        while True:
            before = time.monotonic()
            state = hand.read_once()
            after = time.monotonic()
            connection.send(("sample", dict(
                host_time_s=(before + after) / 2,
                read_duration_s=after - before,
                device_time_s=state.time.to_sec(),
                width_m=state.width, max_width_m=state.max_width,
                is_grasped=state.is_grasped, temperature_c=state.temperature)))
            time.sleep(max(0., .05 - (time.monotonic() - before)))
    except BaseException as error:
        try:
            connection.send(("error", f"{type(error).__name__}: {error}"))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        connection.close()


class HandReader(IsolatedReader):
    def __init__(self, robot_ip):
        context = mp.get_context("spawn")
        self.connection, child = context.Pipe(duplex=False)
        self.process = context.Process(target=_hand_worker, args=(child, robot_ip), daemon=True)
        self.process.start()
        child.close()

    def poll(self):
        return self.connection.poll()


def validate_hand(packet, previous=None):
    for key in ("host_time_s", "read_duration_s", "device_time_s", "width_m",
                "max_width_m", "temperature_c"):
        if not np.isfinite(packet[key]):
            raise ValueError(f"invalid Hand field {key}")
    if (packet["read_duration_s"] < 0 or packet["device_time_s"] < 0
            or not 0 <= packet["width_m"] <= packet["max_width_m"]
            or packet["max_width_m"] <= 0 or type(packet["is_grasped"]) is not bool):
        raise ValueError("invalid Hand state")
    if previous is not None and (packet["host_time_s"] <= previous["host_time_s"]
                                 or packet["device_time_s"] <= previous["device_time_s"]):
        raise ValueError("nonmonotonic Hand timestamps")


class TeachingReader:
    """Wrap readers, log both host clocks, and reject non-guiding activity.

    Ending this recorder stops READS ONLY. It cannot stop hand guiding or the Hand.
    """
    def __init__(self, arm_factory, hand_factory, stream, receipt, clock=time.monotonic):
        self.arm = self.hand = None
        self.stream, self.receipt, self.clock = stream, receipt, clock
        self.previous_hand = None
        self.first = True
        try:
            self.hand = hand_factory()
            self._save_hand(self.hand.read(2))
            self.arm = arm_factory()
        except BaseException:
            try:
                self.close()
            except BaseException:
                pass  # Preserve the original construction/read failure.
            raise

    def _save_hand(self, packet):
        validate_hand(packet, self.previous_hand)
        self.stream.write(json.dumps(packet, allow_nan=False) + "\n")
        self.stream.flush()
        self.previous_hand = packet
        self.receipt["hand_samples"] = self.receipt.get("hand_samples", 0) + 1

    def read(self, timeout):
        # Drain a bounded batch; reads never issue stop/grasp/move/recovery calls.
        for _ in range(100):
            if not self.hand.poll():
                break
            self._save_hand(self.hand.read(.1))
        if self.clock() - self.previous_hand["host_time_s"] > 2:
            raise TimeoutError("Hand state stale; teaching session invalid")
        packet = self.arm.read(timeout)
        status = packet["status"]
        self.receipt["last_observed_status"] = dict(status)
        if (status["robot_mode"] not in READ_ONLY_TEACHING_MODES
                or status["current_errors"] != "[]" or status["last_motion_errors"] != "[]"):
            raise ValueError(f"unsupported read-only teaching state: {status}; no recovery attempted")
        if self.first:
            if self.receipt.get('start_reference') is not None:
                check_reference_packet(packet, self.receipt['start_reference'])
            self.receipt["arm_first_host_time_s"] = packet["host_time_s"]
            self.first = False
            print("RECORDING_READY: read-only; hold still 1 s, guide manually, finish with 1 s still.", flush=True)
        self.receipt["arm_last_host_time_s"] = packet["host_time_s"]
        return packet

    def close(self):
        error = None
        for reader in (self.arm, self.hand):
            if reader is not None:
                try:
                    reader.close()
                except BaseException as exc:
                    error = error or exc
        if error:
            raise error


def record_session(output, robot_ip, duration, *, arm_factory=None, hand_factory=None, start_reference=None):
    validate_request(duration, 2)
    if duration < 5:
        raise ValueError("teaching duration must be 5..120 seconds")
    reference = load_start_reference(start_reference) if start_reference is not None else None
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)  # Reserve before hardware factories.
    receipt = dict(schema="fr3_teaching_readonly_v1", complete=False,
                   started_utc=datetime.now(timezone.utc).isoformat(),
                   requested_duration_s=duration, motion_commands_sent=False,
                   parameter_writes=False, gripper_commands_sent=False,
                   force_recorded=False, hardware_replay_ready=False,
                   clock="host monotonic; arm/Hand device clocks are NOT interchangeable",
                   source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    if reference is not None:
        receipt['start_reference'] = reference
    try:
        with (output / "hand.jsonl").open("x") as stream:
            capture_to_file(lambda: TeachingReader(
                arm_factory or (lambda: IsolatedReader(robot_ip)),
                hand_factory or (lambda: HandReader(robot_ip)), stream, receipt),
                output / "arm.npz", duration, 2)
        receipt["complete"] = True
    except BaseException as error:
        receipt["failure"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        receipt["files"] = {
            name: dict(bytes=(output / name).stat().st_size,
                       sha256=hashlib.sha256((output / name).read_bytes()).hexdigest())
            for name in ("arm.npz", "hand.jsonl") if (output / name).exists()}
        with (output / "session.json").open("x") as stream:
            json.dump(receipt, stream, indent=2, allow_nan=False)
        print("READS_ENDED: no robot stop, movement, or gripper command was sent.", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--start-read-only-recording", action="store_true", required=True)
    parser.add_argument("--start-reference", type=Path,
                        help="Saved stationary zero; rejects initial mismatch without moving")
    args = parser.parse_args()
    record_session(args.output, args.robot_ip, args.duration, start_reference=args.start_reference)


if __name__ == "__main__":
    main()
