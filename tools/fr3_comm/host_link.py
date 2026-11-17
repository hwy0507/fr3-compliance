"""Query/disable/restore EEE on the reviewed robot-facing NIC only.

Changes host networking, never contacts the robot. Run --show unprivileged.
--disable/--restore need CAP_NET_ADMIN (normally sudo). Stop all FCI sessions
first: a network setting change can interrupt the link. Backups are exclusive.
"""
import argparse
import ctypes
import fcntl
import json
import os
from pathlib import Path
import socket
import struct

NIC = "enp131s0"
PCI = "0000:83:00.0"
SIOCETHTOOL = 0x8946
GEEE, SEEE = 0x44, 0x45
FIELDS = ("cmd", "supported", "advertised", "lp_advertised", "eee_active",
          "eee_enabled", "tx_lpi_enabled", "tx_lpi_timer", "reserved0", "reserved1")


def identity():
    base = Path("/sys/class/net") / NIC
    if (base / "device").resolve().name != PCI:
        raise RuntimeError("NIC PCI identity changed; refusing")
    return {"interface": NIC, "pci": PCI, "mac": (base / "address").read_text().strip()}


def ioctl_eee(values):
    data = ctypes.create_string_buffer(struct.pack("10I", *values), 40)
    request = struct.pack("16sP", NIC.encode(), ctypes.addressof(data))
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        fcntl.ioctl(sock.fileno(), SIOCETHTOOL, request)
    return dict(zip(FIELDS, struct.unpack("10I", data.raw)))


def read_eee():
    return ioctl_eee([GEEE] + [0] * 9)


def validate_readback(after, enabled, tx_lpi, advertised, timer):
    # Drivers may report no advertised EEE modes while EEE is disabled.
    # The original advertisement remains in the exclusive backup for restoration.
    # Enabling/restoring still requires the exact requested advertisement.
    for key, expected in {"eee_enabled": enabled, "tx_lpi_enabled": tx_lpi,
                          "tx_lpi_timer": timer}.items():
        if after[key] != expected:
            raise RuntimeError(f"EEE readback mismatch: {key}; write may already have "
                               "taken effect; inspect --show and original backup, do not repeat --disable")
    allowed_advertisements = {advertised} if enabled else {0, advertised}
    if after["advertised"] not in allowed_advertisements:
        raise RuntimeError("EEE readback mismatch: advertised; inspect --show and original backup")
    if not enabled and after["eee_active"]:
        raise RuntimeError("EEE still active after disable; inspect --show and original backup")


def set_eee(current, enabled, tx_lpi, advertised, timer):
    values = dict(current, cmd=SEEE, eee_enabled=enabled, tx_lpi_enabled=tx_lpi,
                  advertised=advertised, tx_lpi_timer=timer, reserved0=0, reserved1=0)
    ioctl_eee([values[k] for k in FIELDS])
    after = read_eee()
    validate_readback(after, enabled, tx_lpi, advertised, timer)
    return after


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--show", action="store_true")
    group.add_argument("--disable", type=Path, metavar="NEW_BACKUP_JSON")
    group.add_argument("--restore", type=Path, metavar="BACKUP_JSON")
    args = parser.parse_args()
    nic, current = identity(), read_eee()
    if args.show:
        print(json.dumps({**nic, "eee": current}, indent=2))
        return
    if os.geteuid() != 0:
        raise PermissionError("Use sudo for host NIC change; never send a password to chat")
    if args.disable:
        # Save and fsync BEFORE the potentially link-interrupting ioctl.
        with args.disable.open("x") as stream:
            json.dump({**nic, "original_eee": current}, stream, indent=2)
            stream.write("\n"); stream.flush(); os.fsync(stream.fileno())
        after = set_eee(current, 0, 0, current["advertised"], current["tx_lpi_timer"])
    else:
        backup = json.loads(args.restore.read_text())
        if any(backup[k] != nic[k] for k in nic):
            raise ValueError("Backup NIC identity mismatch")
        original = backup["original_eee"]
        after = set_eee(current, original["eee_enabled"], original["tx_lpi_enabled"],
                        original["advertised"], original["tx_lpi_timer"])
    print(json.dumps({**nic, "before": current, "after": after,
                      "scope": "host EEE only; no robot command"}, indent=2))


if __name__ == "__main__":
    main()
