

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
    },
    "pi": {
        "camera": "0",              # fallback only - _find_camera() picks the USB cam
        "serial_port": "/dev/ttyUSB0",
        "display": False,           # headless over SSH, no X server
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



    def describe(self) -> str:
        zone = (",".join(f"{v:.2f}" for v in self.pickup_zone)
                if self.pickup_zone else "off")
        return (
            f"profile={self.profile}  camera={self.camera}  "
            f"serial={self.serial_port}@{self.baud_rate}  "
            f"model={Path(self.model_path).name}  conf={self.confidence}  "
            f"capture={self.capture_size}  infer={self.infer_size or 'native'}  "
            f"zone={zone}"
        )


# Where trash sits in the frame when the arm can grab it, as (x1, y1, x2, y2)
# fractions of the frame. Measured from a frame of the rover's own camera,
# looking straight ahead, with the grab spot marked on it: the mark covered
# x 247-337, y 405-444 of 640x480. The zone is that mark, 0.025 wider each
# side for small misalignment. Only valid while the camera mount does not
# move - re-measure after touching the mast or the camera's tilt.
PICKUP_ZONE = (0.36, 0.84, 0.55, 0.93)


def _zone(raw: str | None) -> tuple | None:
    """ROBOSORT_PICKUP_ZONE="x1,y1,x2,y2" overrides the zone; "off" sends B/N
    for a detection anywhere in the frame, as before the zone existed."""
    if raw is None:
        return PICKUP_ZONE
    if raw.strip().lower() in ("off", "none", "0", ""):
        return None
    try:
        x1, y1, x2, y2 = (float(v) for v in raw.split(","))
    except ValueError:
        raise SystemExit(f"ROBOSORT_PICKUP_ZONE={raw!r}: expected x1,y1,x2,y2 or off")
    if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
        raise SystemExit(f"ROBOSORT_PICKUP_ZONE={raw!r}: need 0 <= x1 < x2 <= 1 "
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


def _find_camera(default: str) -> str:
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
        camera=_env("camera", _find_camera(p["camera"])
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
        pickup_zone=_zone(os.environ.get("ROBOSORT_PICKUP_ZONE")),
    )


CFG = load()


if __name__ == "__main__":
    print(CFG.describe())
    print(f"  model exists: {Path(CFG.model_path).exists()}  ({CFG.model_path})")
