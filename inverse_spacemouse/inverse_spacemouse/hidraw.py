"""Find and decode a 3Dconnexion SpaceMouse through Linux hidraw."""

import glob
import os
import struct


# Logitech (older 3Dconnexion devices) and 3Dconnexion.
VENDOR_IDS = (0x046D, 0x256F)
# Report descriptor items: Usage Page (Generic Desktop), Usage (Multi-axis Controller).
MULTI_AXIS_USAGE = b"\x05\x01\x09\x08"


def _read(path, mode="r"):
    try:
        with open(path, mode) as f:
            return f.read()
    except OSError:
        return None


def find_device():
    """Return (/dev/hidrawN, name) of the first SpaceMouse found, or None."""
    fallback = None
    for sys_path in sorted(glob.glob("/sys/class/hidraw/hidraw*")):
        uevent = _read(os.path.join(sys_path, "device", "uevent")) or ""
        fields = dict(line.split("=", 1) for line in uevent.splitlines() if "=" in line)
        try:
            vendor = int(fields["HID_ID"].split(":")[1], 16)
        except (KeyError, IndexError, ValueError):
            continue
        if vendor not in VENDOR_IDS:
            continue
        dev_path = "/dev/" + os.path.basename(sys_path)
        name = fields.get("HID_NAME", "")
        # A device can expose several hidraw interfaces; take the multi-axis one.
        descriptor = _read(os.path.join(sys_path, "device", "report_descriptor"), "rb")
        if descriptor and MULTI_AXIS_USAGE in descriptor:
            return dev_path, name
        is_spacemouse = any(s in name.lower() for s in ("space", "3dconnexion"))
        if fallback is None and is_spacemouse:
            fallback = dev_path, name
    return fallback


def parse_report(data, axes, buttons):
    """Update axes [x, y, z, rx, ry, rz] in place; return the new button bitmask.

    Raw HID frame: x = right, y = toward the user, z = down; values about +-350.
    """
    report_id = data[0]
    if report_id == 1 and len(data) >= 7:
        axes[0:3] = struct.unpack_from("<hhh", data, 1)
        if len(data) >= 13:  # newer devices pack rotation into report 1
            axes[3:6] = struct.unpack_from("<hhh", data, 7)
    elif report_id == 2 and len(data) >= 7:
        axes[3:6] = struct.unpack_from("<hhh", data, 1)
    elif report_id == 3:
        buttons = int.from_bytes(data[1:], "little")
    return buttons
