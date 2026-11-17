"""Bounded capture core tested using fake readers; no hardware imports."""
import json
import time
from pathlib import Path

import numpy as np

from deploy.contracts import finite_vector, positive_seconds, validate_robot_config
from deploy.recording import decode_pose


def validate_request(duration, read_timeout):
    positive_seconds(duration, "recording duration", 120)
    positive_seconds(read_timeout, "read timeout", 10)


def capture_to_file(reader_factory, output, duration, read_timeout=2, *, clock=time.monotonic,
                    max_samples=150000):
    validate_request(duration, read_timeout)
    if not isinstance(max_samples, int) or max_samples < 10 or max_samples > 150000:
        raise ValueError("max_samples must be an integer in [10, 150000]")
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve exclusive output BEFORE constructing anything that can connect.
    with output.open("xb") as stream:
        reader = None
        samples = []
        metadata = {}
        reason = "not_started"
        interrupted = False
        primary_error = None
        try:
            start = clock()
            deadline = start + duration
            reader = reader_factory()
            while clock() < deadline:
                remaining = deadline - clock()
                if remaining <= 0:
                    break
                packet = reader.read(min(read_timeout, remaining))
                for name, width in (("q", 7), ("dq", 7), ("tau_J", 7),
                                    ("tau_ext_hat_filtered", 7), ("O_T_EE", 16)):
                    packet[name] = finite_vector(packet[name], width, name)
                decode_pose(packet["O_T_EE"])
                for name in ("robot_time_s", "host_time_s", "read_duration_s", "control_command_success_rate"):
                    if not np.isfinite(packet[name]):
                        raise ValueError(f"nonfinite {name}")
                if packet["read_duration_s"] < 0 or not 0 <= packet["control_command_success_rate"] <= 1:
                    raise ValueError("invalid timing/success rate")
                if any(key not in packet["status"] for key in ("robot_mode", "current_errors", "last_motion_errors")):
                    raise ValueError("missing status fields")
                current = {"config": packet["config"], "binding_version": packet["binding_version"]}
                validate_robot_config(current["config"])
                if not isinstance(current["binding_version"], str) or not current["binding_version"]:
                    raise ValueError("missing binding version")
                # No silent coordinate/payload reconfiguration during a recording.
                if not metadata:
                    metadata = current
                elif current != metadata:
                    raise ValueError("robot configuration changed during recording")
                if samples and (packet["robot_time_s"] <= samples[-1]["robot_time_s"]
                                or packet["host_time_s"] <= samples[-1]["host_time_s"]):
                    raise ValueError("duplicate/nonmonotonic robot or host time")
                # Keep one verified config snapshot, not a large duplicate per sample.
                packet.pop("config")
                packet.pop("binding_version")
                samples.append(packet)
                if len(samples) >= max_samples:
                    raise RuntimeError("sample budget exhausted")
            if len(samples) < 10:
                raise RuntimeError("fewer than 10 robot samples recorded")
            reason = "duration_elapsed"
        except BaseException as error:
            primary_error = error
            interrupted = isinstance(error, KeyboardInterrupt)
            reason = "keyboard_interrupt" if interrupted else f"error:{type(error).__name__}:{error}"
            raise
        finally:
            cleanup_error = None
            if reader is not None:
                try:
                    reader.close()
                except BaseException as error:
                    cleanup_error = error
                    if primary_error is None:
                        reason = f"cleanup_error:{type(error).__name__}:{error}"
            try:
                fields = {name: np.asarray([s[name] for s in samples], dtype=float).reshape(-1, width)
                          for name, width in (("q", 7), ("dq", 7), ("tau_J", 7),
                                              ("tau_ext_hat_filtered", 7), ("O_T_EE", 16))}
                fields.update({name: np.asarray([s[name] for s in samples], dtype=float) for name in
                               ("robot_time_s", "read_duration_s", "control_command_success_rate")})
                host = np.asarray([s["host_time_s"] for s in samples], dtype=float)
                fields["time_s"] = host - host[0] if len(host) else host
                np.savez_compressed(stream, **fields, schema_version=2,
                                    metadata_json=json.dumps(metadata),
                                    sample_metadata=np.asarray([json.dumps(s["status"]) for s in samples], dtype=str),
                                    complete=reason == "duration_elapsed", interrupted=interrupted, stop_reason=reason)
            except BaseException:
                if primary_error is None:
                    raise
            if cleanup_error is not None and primary_error is None:
                raise cleanup_error
