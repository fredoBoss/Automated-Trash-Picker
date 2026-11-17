

import glob
import os
import sys
from dataclasses import dataclass
from pathlib import Path

# Repo root, so model paths work no matter where python was launched from.
ROOT = Path(__file__).resolve().parent.parent
HERE = Path(__file__).resolve().parent


# --- Profiles ---
PROFILES = {
    "laptop": {
        "camera": "0",              # default webcam
        "serial_port": "COM3",      # fallback only - _find_serial_by_usb_id() picks the Mega
        "display": True,            # a window is fine here
        "threads": 0,               # every core
        "max_temp": 0,              # no thermal guard
    },
    "pi": {
        "camera": "0",              # fallback only - find_camera() picks the USB cam
        "serial_port": "/dev/ttyUSB0",
        "display": False,           # headless over SSH, no X server
        # All 4 cores took the H3 to 105 C in under a minute with no heatsink,
        # and the kernel powered the Pi off. Measured on the live camera with
        # the thermal guard, 4 min each: 2 cores ran 0.69 s/frame but paused
        # 81% of the time (15 frames/min); 1 core ran 1.13 s/frame and paused
        # 5% (48 frames/min). Back to 2 once the Pi has a heatsink.
        "threads": 1,
        # At this temperature vision.py turns the model off for 30 s
        # (vision.py COOL_PAUSE_S) and the Mega stops the wheels meanwhile.
        # The kernel throttles from 75 C and powers off at 105 C; one frame
        # can add 5-8 C, so 94 leaves room. With no heatsink this Pi idles at
        # ~85 C (the CPU stays at 1.3 V), so a much lower limit would keep the
        # model off for good.
        "max_temp": 94,
    },
}

# Model candidates, tried in order. Lets the laptop pick up freshly trained
# weights while the Pi picks up the exported NCNN dir, with no edit either side.
MODEL_CANDIDATES = [
    HERE / "models" / "robosort_ncnn_model",              # exported NCNN (Pi)
    HERE / "models" / "robosort.pt",                      # hand-copied best.pt
    ROOT / "runs" / "train" / "robosort_yolo11" / "weights" / "best.pt",
    ROOT / "best.pt",                                  # untrained fallback
]


@dataclass(frozen=True)
class Config:
    profile: str
    camera: str            # webcam index as a string, or a path to an image folder
    serial_port: str
    baud_rate: int
    display: bool
    model_path: str
    confidence: float
    capture_size: str        # "WxH" (pinned), "native", or "max"
    infer_size: int          # long side fed to the model; 0 = camera native
    serial_retries: int
    pickup_zone: tuple | None    # (x1, y1, x2, y2) fractions of the frame; None = off
    grab_spot: tuple | None      # the same, where B/N goes out; None = anywhere in the zone
    threads: int             # CPU cores the model may use; 0 = all of them
    max_temp: float          # CPU deg C at which vision.py pauses; 0 = never

    def describe(self) -> str:
        def fmt(z):
            return ",".join(f"{v:.2f}" for v in z) if z else "off"
        zone = fmt(self.pickup_zone)
        if self.pickup_zone:
            zone += f"  grab={fmt(self.grab_spot)}"
        return (
            f"profile={self.profile}  camera={self.camera}  "
            f"serial={self.serial_port}@{self.baud_rate}  "
            f"model={Path(self.model_path).name}  conf={self.confidence}  "
            f"capture={self.capture_size}  infer={self.infer_size or 'native'}  "
            f"zone={zone}  threads={self.threads or 'all'}  "
            f"max_temp={f'{self.max_temp:g}C' if self.max_temp else 'off'}"
        )


# Both as (x1, y1, x2, y2) fractions of the frame. Only valid while the
# camera mount does not move - re-measure after touching the mast or the
# camera's tilt, and keep the camera panned straight ahead.
#
# PICKUP_ZONE: trash in here gets collected. Drawn on a 640x480 frame of the
# rover's own camera (2026-11-15): all the floor in view, edge to edge, from
# just under the top of the frame (y 38) down to the scoop (y 414). Trash in
# it but short of the grab spot makes the rover step up to it (F, or L/R to
# centre it). Earlier zones: (0.22, 0.47, 0.73, 0.87), and before that the
# grab spot alone, (0.36, 0.84, 0.55, 0.93).
PICKUP_ZONE = (0.00, 0.08, 1.00, 0.86)
# GRAB_SPOT: B/N - lift and belt - once the item's centre is in its columns
# and the item's near edge (its box's bottom) has reached its top edge, where
# the floor meets the scoop. Move the top edge (y1) to where an item has to
# be for the lift to catch it. Its bottom edge is drawn but not used. Set as
# the middle third and bottom third of the (0.22, 0.47, 0.73, 0.87) zone,
# and kept when the zone grew.
GRAB_SPOT = (0.39, 0.74, 0.56, 0.87)


def _zone(raw: str | None, default: tuple, name: str) -> tuple | None:
    """ROBOSORT_PICKUP_ZONE / ROBOSORT_GRAB_SPOT = "x1,y1,x2,y2" overrides
    one; "off" turns it off. No zone: B/N for a detection anywhere in the
    frame. No grab spot: B/N anywhere in the zone, with no approach."""
    if raw is None:
        return default
    if raw.strip().lower() in ("off", "none", "0", ""):
        return None
    try:
        x1, y1, x2, y2 = (float(v) for v in raw.split(","))
    except ValueError:
        raise SystemExit(f"{name}={raw!r}: expected x1,y1,x2,y2 or off")
    if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
        raise SystemExit(f"{name}={raw!r}: need 0 <= x1 < x2 <= 1 "
                         f"and 0 <= y1 < y2 <= 1")
    return (x1, y1, x2, y2)


def _default_profile() -> str:
    return "laptop" if sys.platform.startswith("win") else "pi"


def _env(key: str, default, cast=str):
    raw = os.environ.get(f"ROBOSORT_{key.upper()}")
    if raw is None:
        return default
    if cast is bool:
        return raw.strip().lower() in ("1", "true", "yes", "on")
    return cast(raw)


def _find_serial(default: str) -> str:
    """The Mega's port on the Pi. A genuine Mega enumerates as /dev/ttyACM*,
    a CH340 clone as /dev/ttyUSB*, and which one is wired up is not known in
    advance. Nothing plugged in -> the profile default, so the error from
    serial_link names a real path."""
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return default


# USB vendor IDs the Mega can show up with: WCH (the CH340 on clones, like
# this one at 1A86:7523) and the two Arduino vendors on genuine boards.
MEGA_USB_VENDORS = (0x1A86, 0x2341, 0x2A03)


def _find_serial_by_usb_id(default: str) -> str:
    """The Mega's port on the laptop. Windows hands out COM numbers by USB
    socket and history - the Mega has been COM3 and COM4 - so find it by
    its USB vendor ID. No pyserial or no match -> the profile default."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return default
    for port in sorted(list_ports.comports(), key=lambda p: p.device):
        if port.vid in MEGA_USB_VENDORS:
            return port.device
    return default


def find_camera(default: str | None) -> str | None:
    """The USB webcam's index on the Pi. It is not /dev/video0 there: the H3's
    hardware video decoder (cedrus) registers first and takes that node. A
    UVC cam adds two nodes - the image stream (index 0) and a metadata node -
    so take the first USB-backed node whose index is 0."""
    nodes = glob.glob("/sys/class/video4linux/video*")
    for node in sorted(nodes, key=lambda n: int(n.rsplit("video", 1)[1])):
        try:
            with open(os.path.join(node, "index")) as f:
                primary = f.read().strip() == "0"
        except OSError:
            continue
        if primary and "/usb" in os.path.realpath(os.path.join(node, "device")):
            return node.rsplit("video", 1)[1]
    return default


def _find_model() -> str:
    for candidate in MODEL_CANDIDATES:
        if candidate.exists():
            return str(candidate)
    # Nothing on disk yet - hand back the first candidate so the error message
    # from classify.py names the path you are expected to create.
    return str(MODEL_CANDIDATES[0])


def load() -> Config:
    profile = _env("profile", _default_profile())
    if profile not in PROFILES:
        raise SystemExit(
            f"Unknown ROBOSORT_PROFILE={profile!r}. Known: {', '.join(PROFILES)}"
        )
    p = PROFILES[profile]

    return Config(
        profile=profile,
        camera=_env("camera", find_camera(p["camera"])
                    if profile == "pi" else p["camera"]),
        serial_port=_env("serial_port", _find_serial(p["serial_port"])
                         if profile == "pi" else _find_serial_by_usb_id(p["serial_port"])),
        baud_rate=_env("baud_rate", 9600, int),
        display=_env("display", p["display"], bool),
        model_path=_env("model", _find_model()),
        confidence=_env("confidence", 0.55, float),
        # Pinned, not "native": the driver default is machine-specific, and
        # laptop and Pi must feed the model identical frames. 640x480 is 2x
        # the 320 training size (a clean downscale) and is the one resolution
        # every USB webcam supports. A cam that refuses it gets downscaled in
        # software, so the pin holds either way.
        capture_size=_env("capture", "640x480"),
        # Inference is a separate decision from capture: the long side the
        # model sees. Aspect ratio comes from the frame, so nothing is
        # stretched. 0 runs the model at the camera's full resolution.
        infer_size=_env("infer_size", 320, int),
        serial_retries=_env("serial_retries", 5, int),
        pickup_zone=_zone(os.environ.get("ROBOSORT_PICKUP_ZONE"), PICKUP_ZONE,
                          "ROBOSORT_PICKUP_ZONE"),
        grab_spot=_zone(os.environ.get("ROBOSORT_GRAB_SPOT"), GRAB_SPOT,
                        "ROBOSORT_GRAB_SPOT"),
        threads=_env("threads", p["threads"], int),
        max_temp=_env("max_temp", p["max_temp"], float),
    )


CFG = load()


if __name__ == "__main__":
    print(CFG.describe())
    print(f"  model exists: {Path(CFG.model_path).exists()}  ({CFG.model_path})")
