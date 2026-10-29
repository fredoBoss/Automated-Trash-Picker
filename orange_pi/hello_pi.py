"""
RoboSort - Upload / Connection Smoke Test (Orange Pi 1)
Run this first, right after copying code to the board. It confirms the
upload landed and reports whether the hardware the vision pipeline needs
is actually present.

Stdlib only - it must run on a bare Armbian Minimal image before any
pip install, otherwise it cannot tell you the upload worked.

Usage on the Pi:
    python3 hello_pi.py
"""

import glob
import os
import platform
import socket
import sys
from datetime import datetime

BANNER = "CODE UPLOAD CONNECTED TO ORANGE PI"


def lan_ip() -> str:
    """Primary LAN address, found by asking the kernel which route it would use."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 1))   # no packets sent, just picks the route
        return s.getsockname()[0]
    except OSError:
        return "no route to LAN"
    finally:
        s.close()


def total_ram_mb() -> str:
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                if line.startswith("MemTotal:"):
                    return f"{int(line.split()[1]) // 1024} MB"
    except OSError:
        pass
    return "unknown"


def board_model() -> str:
    for path in ("/proc/device-tree/model", "/sys/firmware/devicetree/base/model"):
        try:
            with open(path) as f:
                return f.read().strip("\x00\n")
        except OSError:
            continue
    return "unknown (not an ARM board?)"


def report(label: str, value: str, ok: bool = True) -> None:
    mark = "OK " if ok else "-- "
    print(f"  [{mark}] {label:<16} {value}")


def main():
    print()
    print("=" * 52)
    print(BANNER.center(52))
    print("=" * 52)
    print(f"  {datetime.now():%Y-%m-%d %H:%M:%S}")
    print()

    print("Board")
    report("model", board_model())
    report("hostname", socket.gethostname())
    report("lan ip", lan_ip())
    report("ram", total_ram_mb())
    report("kernel", platform.release())
    report("arch", platform.machine())
    report("python", sys.version.split()[0])
    print()

    print("Upload")
    here = os.path.dirname(os.path.abspath(__file__))
    report("running from", here)
    siblings = sorted(f for f in os.listdir(here) if f.endswith(".py"))
    report("py files here", ", ".join(siblings) if siblings else "none")
    print()

    # These are the two devices vision.py needs. Missing is fine right now -
    # this is a checklist, not a failure.
    print("RoboSort hardware")
    # /dev/video0 on the H3 is the hardware video decoder (cedrus), not a
    # camera - only USB-backed nodes count.
    cameras, others = [], []
    for node in sorted(glob.glob("/sys/class/video4linux/video*")):
        try:
            with open(os.path.join(node, "name")) as f:
                name = f.read().strip()
        except OSError:
            name = "?"
        dev = "/dev/" + os.path.basename(node)
        usb = "/usb" in os.path.realpath(os.path.join(node, "device"))
        (cameras if usb else others).append(f"{dev} ({name})")
    report("camera", ", ".join(cameras) if cameras else "no USB camera", bool(cameras))
    if others:
        report("not cameras", ", ".join(others))

    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    report("arduino serial", ", ".join(ports) if ports else "not detected", bool(ports))
    print()

    print("Upload path verified. You can now run the vision pipeline.")
    print()


if __name__ == "__main__":
    main()
