"""
RoboSort - Frame Capture (laptop + Orange Pi One)

The only module that touches a camera. Everything downstream consumes
(label, frame) pairs and never learns where they came from, so swapping the
laptop webcam for the Pi's USB cam - or for a folder of saved images during
benchmarking - changes nothing but the source string.

    open_source("0")                  webcam, whatever resolution it offers
    open_source("0", "max")           webcam at its highest supported size
    open_source("0", "640x480")       webcam pinned to one size
    open_source("../data/test")       every image in the folder, once each

Frames are handed on at the resolution they were captured at. Shrinking for
the model is classify.py's business, and it derives that size from the frame
itself, so nothing here has to guess what the model wants.
"""

import sys
import time
from pathlib import Path
from typing import Iterator, Tuple

import cv2

Frame = Tuple[str, "cv2.typing.MatLike"]   # (label, image)

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

# Windows defaults to MSMF, which can take seconds to open a camera and
# sometimes refuses a second one. DirectShow opens fast and honours
# resolution requests. On Linux (the Pi) V4L2 is the native path.
CAMERA_BACKEND = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_V4L2

MAX_READ_FAILURES = 30      # consecutive dropped frames before giving up

# Highest first - probing for "max" takes the first one the camera accepts.
RESOLUTION_LADDER = [
    (1920, 1080), (1280, 720), (1024, 768),
    (800, 600), (640, 480), (352, 288), (320, 240),
]

NATIVE_SPECS = ("native", "", "default")


def _quiet(on: bool) -> None:
    """OpenCV shouts on stderr when a camera index does not exist. While
    probing, that is the expected outcome, not news."""
    try:
        cv2.setLogLevel(0 if on else 3)     # 0 SILENT, 3 WARNING (default)
    except Exception:
        pass                                # older/headless builds


def _read_size(cap) -> tuple[int, int] | None:
    ok, frame = cap.read()
    if not ok or frame is None:
        return None
    h, w = frame.shape[:2]
    return w, h


def probe_max(cap) -> tuple[int, int] | None:
    """Highest resolution the camera will actually deliver.

    A webcam reports success for sizes it does not support and quietly hands
    back something else, so every candidate is confirmed against a real frame
    instead of trusting the property read-back.
    """
    for w, h in RESOLUTION_LADDER:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if _read_size(cap) == (w, h):
            return w, h
    return None


def list_cameras(max_index: int = 4) -> list[dict]:
    """Which indices work, what they give by default, and what they can do.
    Windows renumbers cameras between runs, so this is worth re-checking
    rather than memorising an index.
    """
    found = []
    _quiet(True)
    try:
        for idx in range(max_index):
            cap = cv2.VideoCapture(idx, CAMERA_BACKEND)
            if cap.isOpened():
                default = _read_size(cap)
                if default:
                    best = probe_max(cap) or default
                    found.append({"index": idx, "default": default, "max": best})
            cap.release()
    finally:
        _quiet(False)
    return found


def fit(frame, width: int, height: int):
    """Scale down to fit inside width x height, preserving aspect ratio. Never
    stretches: a 16:9 frame squashed into 4:3 shifts every box the model has
    been trained to expect."""
    h, w = frame.shape[:2]
    if w <= width and h <= height:
        return frame
    scale = min(width / w, height / h)
    return cv2.resize(frame, (max(1, int(w * scale)), max(1, int(h * scale))),
                      interpolation=cv2.INTER_AREA)


def _parse_size(spec: str) -> tuple[int, int]:
    try:
        w, h = (int(v) for v in spec.split("x"))
        return w, h
    except ValueError:
        raise SystemExit(
            f"Bad capture size {spec!r} - use 'native', 'max', or WxH (640x480)"
        )


class CameraSource:
    """Live camera. Infinite stream unless bounded by max_frames/max_seconds."""

    def __init__(self, index: int, capture: str = "native",
                 max_frames: int = 0, max_seconds: float = 0.0):
        self.index = index
        self.capture_spec = capture
        self.max_frames = max_frames
        self.max_seconds = max_seconds

        self.cap = cv2.VideoCapture(index, CAMERA_BACKEND)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Cannot open camera {index}. On the Pi check /dev/video* exists "
                f"and the user is in the 'video' group; on Windows check no other "
                f"app (Teams, Zoom, Camera) is holding the webcam. "
                f"Run test_cv.py --list-cameras to see what is available."
            )

        # MJPG lets USB cams push a usable frame rate at higher resolutions;
        # raw YUYV often caps at 5 FPS. Ignored by cams that do not support it.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # prefer latest over queued

        self.target = self._apply_capture_size(capture)

        # First frames off a webcam are often black while exposure settles.
        for _ in range(5):
            self.cap.read()

        size = _read_size(self.cap)
        if size is None:
            raise RuntimeError(f"Camera {index} opened but returned no frames.")
        self.width, self.height = size
        self.is_live = True

    def _apply_capture_size(self, capture: str) -> tuple[int, int] | None:
        """Returns a (w, h) to downscale to later, or None to pass frames
        through untouched."""
        spec = capture.strip().lower()

        if spec in NATIVE_SPECS:
            return None                     # driver default, used as-is

        if spec == "max":
            if probe_max(self.cap) is None:
                print("  no ladder resolution accepted, using the camera default")
            return None                     # already capturing at that size

        w, h = _parse_size(spec)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if _read_size(self.cap) == (w, h):
            return None                     # camera honoured it, nothing to do
        # Camera refused and handed back a different size. Keep the promise in
        # software rather than silently running at a resolution nobody asked for.
        return (w, h)

    def describe(self) -> str:
        base = f"camera {self.index}: {self.width}x{self.height}"
        if self.capture_spec.strip().lower() not in NATIVE_SPECS:
            base += f"  (asked {self.capture_spec})"
        if self.target:
            base += f" -> downscaled to fit {self.target[0]}x{self.target[1]}"
        return base

    def __len__(self) -> int:
        return self.max_frames        # 0 = unbounded

    def frames(self) -> Iterator[Frame]:
        n = 0
        failures = 0
        started = time.perf_counter()

        while True:
            ok, frame = self.cap.read()
            if not ok:
                # A dropped frame is normal on USB cams; a stream of them means
                # the camera is gone - unplugged, or grabbed by another app.
                failures += 1
                if failures >= MAX_READ_FAILURES:
                    raise RuntimeError(
                        f"Camera {self.index} stopped returning frames after "
                        f"{n} good ones ({failures} failures in a row)."
                    )
                time.sleep(0.01)
                continue

            failures = 0
            n += 1
            if self.target:
                frame = fit(frame, *self.target)
            yield f"frame{n}", frame

            if self.max_frames and n >= self.max_frames:
                return
            if self.max_seconds and (time.perf_counter() - started) >= self.max_seconds:
                return

    def close(self) -> None:
        self.cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class FolderSource:
    """Every image in a folder, once, in sorted order. This is what makes the
    benchmark repeatable and identical on both machines. Images are used at
    their own resolution - the same rule the camera path follows."""

    def __init__(self, path: Path, capture: str = "native"):
        self.path = path
        self.capture_spec = capture
        self.target = None
        if capture.strip().lower() not in NATIVE_SPECS + ("max",):
            self.target = _parse_size(capture.strip().lower())

        if path.is_file():
            self.paths = [path]
        else:
            self.paths = sorted(
                p for p in path.rglob("*") if p.suffix.lower() in IMAGE_SUFFIXES
            )
        if not self.paths:
            raise RuntimeError(f"No images found under {path}")
        self.is_live = False

    def describe(self) -> str:
        return f"{len(self.paths)} images from {self.path}"

    def __len__(self) -> int:
        return len(self.paths)

    def frames(self) -> Iterator[Frame]:
        for p in self.paths:
            img = cv2.imread(str(p))
            if img is None:
                print(f"  skipped unreadable image: {p.name}")
                continue
            if self.target:
                img = fit(img, *self.target)
            yield p.name, img

    def close(self) -> None:
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def open_source(spec: str, capture: str = "native",
                max_frames: int = 0, max_seconds: float = 0.0):
    """A bare integer is a camera index; anything else is a file or folder."""
    if spec.isdigit():
        return CameraSource(int(spec), capture, max_frames, max_seconds)

    path = Path(spec)
    if not path.exists():
        raise RuntimeError(f"Source not found: {spec}")
    return FolderSource(path, capture)
