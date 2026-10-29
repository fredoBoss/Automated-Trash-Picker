

import os
import time
from dataclasses import dataclass
from pathlib import Path

BIO, NON_BIO, NONE = "B", "N", "X"

# The 4 trained dataset classes (Roboflow TRASH export). Keys are lowercase:
# Roboflow names them "Leaves", "Paper", ... and the lookup ignores case.
DATASET_CLASSES = {
    "leaves": BIO,
    "paper": BIO,
    "metal": NON_BIO,
    "plastic": NON_BIO,
}

# COCO names, so an untrained yolo11n.pt still produces sensible commands
# while the real weights are still training. Harmless once robosort.pt exists.
COCO_FALLBACK = {
    "banana": BIO, "apple": BIO, "orange": BIO, "broccoli": BIO, "carrot": BIO,
    "sandwich": BIO, "hot dog": BIO, "pizza": BIO, "donut": BIO, "cake": BIO,
    "bottle": NON_BIO, "wine glass": NON_BIO, "cup": NON_BIO, "fork": NON_BIO,
    "knife": NON_BIO, "spoon": NON_BIO, "bowl": NON_BIO, "cell phone": NON_BIO,
    "scissors": NON_BIO, "toothbrush": NON_BIO, "vase": NON_BIO, "book": NON_BIO,
}

CLASS_MAP = {**COCO_FALLBACK, **DATASET_CLASSES}


STRIDE = 32

# Ultralytics' predictor drops boxes under 0.25 before we ever see them. The
# NCNN path applies the same floor, so both report "no detection" alike.
DETECT_FLOOR = 0.25


def _round_stride(value: int) -> int:
    return max(STRIDE, ((value + STRIDE - 1) // STRIDE) * STRIDE)


# An item at the grab spot is small in the frame - the spot itself is about
# 14% x 8% of it. A box over a quarter of the frame is the model calling the
# whole scene one object, which it does: trained on close-ups that fill the
# photo, it boxed an empty floor edge to edge as "Plastic 0.81".
MAX_ITEM_AREA = 0.25


def in_zone(box, zone) -> bool:
    
    x1, y1, x2, y2 = box
    zx1, zy1, zx2, zy2 = zone
    if (x2 - x1) * (y2 - y1) > MAX_ITEM_AREA:
        return False
    return zx1 <= (x1 + x2) / 2 <= zx2 and y2 >= zy1 and y1 <= zy2


def _read_metadata(path: Path) -> tuple[dict[int, str], tuple[int, int]]:
    """Class names and input size from an Ultralytics export's metadata.yaml.
    Parsed by hand - two plain fields do not justify PyYAML on the Pi."""
    names: dict[int, str] = {}
    imgsz: list[int] = []
    section = None
    for line in path.read_text(encoding="utf-8").splitlines():
        if line and not line[0].isspace() and not line.startswith("-"):
            section, _, rest = line.partition(":")
            if section == "imgsz" and "[" in rest:          # imgsz: [320, 320]
                imgsz = [int(v) for v in rest.strip(" []").split(",")]
        elif section == "imgsz" and line.startswith("- "):
            imgsz.append(int(line[2:]))
        elif section == "names" and ":" in line:
            key, _, value = line.partition(":")
            names[int(key)] = value.strip().strip("'\"")
    if not names or len(imgsz) != 2:
        raise SystemExit(f"Could not read names/imgsz from {path}")
    return names, (imgsz[0], imgsz[1])


def _letterbox(frame, height: int, width: int):
    """Fit the frame inside height x width without stretching, padding the
    rest with the same grey (114) Ultralytics trains and validates with."""
    import cv2

    h, w = frame.shape[:2]
    r = min(height / h, width / w)
    nw, nh = round(w * r), round(h * r)
    if (nw, nh) != (w, h):
        frame = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    dw, dh = (width - nw) / 2, (height - nh) / 2
    return cv2.copyMakeBorder(
        frame, round(dh - 0.1), round(dh + 0.1), round(dw - 0.1), round(dw + 0.1),
        cv2.BORDER_CONSTANT, value=(114, 114, 114),
    )


class _UltralyticsBackend:
    """best.pt and friends. Needs PyTorch, so laptop only."""

    fixed_size = None

    def __init__(self, path: Path):
        # Imported late: ultralytics takes seconds to load, and bench.py wants
        # that cost outside the measured loop.
        from ultralytics import YOLO

        self.model = YOLO(str(path))

    def predict(self, frame, imgsz) -> list[tuple[str, float, tuple]]:
        """Every box the model reports, as (name, confidence, box), box as
        (x1, y1, x2, y2) fractions of the frame. Classifier picks the one
        the Arduino acts on - the rover takes one item at a time."""
        r = self.model(frame, imgsz=imgsz, verbose=False)[0]
        return [
            (r.names[int(c)], float(p), tuple(b))
            for b, p, c in zip(r.boxes.xyxyn.tolist(), r.boxes.conf.tolist(),
                               r.boxes.cls.tolist())
        ]


class _NcnnBackend:
    """An Ultralytics NCNN export, run with the ncnn package alone.

    The export has a fixed input (320x320 here), so frames are letterboxed
    to it. Output out0 is (4 + classes, anchors): box xywh in input pixels,
    then per-class scores that already went through a sigmoid. Each anchor
    over DETECT_FLOOR is returned with its best class. There is no NMS, so
    one item comes back as several overlapping boxes - harmless, because
    only the single best one is ever acted on, and NMS never removes the
    highest-scoring box.
    """

    def __init__(self, path: Path):
        import ncnn
        import numpy as np

        self._ncnn, self._np = ncnn, np
        self.names, self.fixed_size = _read_metadata(path / "metadata.yaml")

        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = False     # no usable GPU on the Pi
        self.net.opt.num_threads = os.cpu_count() or 1
        if (self.net.load_param(str(path / "model.ncnn.param")) != 0
                or self.net.load_model(str(path / "model.ncnn.bin")) != 0):
            raise SystemExit(f"ncnn could not load the model in {path}")

    def predict(self, frame, imgsz) -> list[tuple[str, float, tuple]]:
        ncnn, np = self._ncnn, self._np
        h, w = self.fixed_size
        img = _letterbox(frame, h, w)

        mat = ncnn.Mat.from_pixels(img, ncnn.Mat.PixelType.PIXEL_BGR2RGB, w, h)
        mat.substract_mean_normalize([], [1 / 255.0] * 3)
        with self.net.create_extractor() as ex:
            ex.input("in0", mat)
            _, out = ex.extract("out0")

        out = np.array(out)
        scores = out[4:]
        cls = scores.argmax(axis=0)
        conf = scores[cls, np.arange(scores.shape[1])]
        keep = np.nonzero(conf >= DETECT_FLOOR)[0]
        if keep.size == 0:
            return []

        # Back from letterboxed input pixels to fractions of the frame,
        # undoing exactly the scale and padding _letterbox() applied.
        fh, fw = frame.shape[:2]
        r = min(h / fh, w / fw)
        nw, nh = round(fw * r), round(fh * r)
        left, top = round((w - nw) / 2 - 0.1), round((h - nh) / 2 - 0.1)
        cx, cy, bw, bh = out[:4, keep]
        x1 = ((cx - bw / 2 - left) / nw).clip(0, 1)
        y1 = ((cy - bh / 2 - top) / nh).clip(0, 1)
        x2 = ((cx + bw / 2 - left) / nw).clip(0, 1)
        y2 = ((cy + bh / 2 - top) / nh).clip(0, 1)

        return [
            (self.names[int(c)], float(p), (float(a), float(b), float(cc), float(d)))
            for c, p, a, b, cc, d in zip(cls[keep], conf[keep], x1, y1, x2, y2)
        ]


@dataclass
class Detection:
    class_name: str | None
    confidence: float
    command: str            # BIO / NON_BIO / NONE - what goes to the Mega
    latency_ms: float
    box: tuple | None = None    # (x1, y1, x2, y2), fractions of the frame
    in_reach: bool = True       # inside the pickup zone, or no zone in use

    @property
    def category(self) -> str:
        return {BIO: "BIO", NON_BIO: "NON-BIO"}.get(self.command, "none")

    def __str__(self) -> str:
        if self.class_name is None:
            return f"no detection ({self.latency_ms:.0f} ms)"
        if not self.in_reach:
            return (
                f"{self.class_name}, out of reach -> none "
                f"({self.confidence:.2f}, {self.latency_ms:.0f} ms)"
            )
        return (
            f"{self.class_name} -> {self.category} "
            f"({self.confidence:.2f}, {self.latency_ms:.0f} ms)"
        )


COLORS = {BIO: (0, 200, 0), NON_BIO: (0, 0, 220)}     # BGR; anything else grey


def draw_box_and_zone(frame, det: Detection, zone) -> None:
    """The pickup zone in yellow and the detection's box: green/red when it
    sends B/N, grey when it is out of reach or below the threshold."""
    import cv2

    h, w = frame.shape[:2]
    if zone is not None:
        x1, y1, x2, y2 = zone
        cv2.rectangle(frame, (int(x1 * w), int(y1 * h)), (int(x2 * w), int(y2 * h)),
                      (0, 220, 220), 2)
        cv2.putText(frame, "pickup zone", (int(x1 * w), int(y1 * h) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 220), 1)
    if det.box is not None:
        x1, y1, x2, y2 = det.box
        cv2.rectangle(frame, (int(x1 * w), int(y1 * h)), (int(x2 * w), int(y2 * h)),
                      COLORS.get(det.command, (170, 170, 170)), 2)


class Classifier:
    def __init__(self, model_path: str, confidence: float, infer_size: int = 320,
                 zone: tuple | None = None):
        path = Path(model_path)
        if not path.exists():
            models = Path(__file__).parent / "models"
            raise SystemExit(
                f"Model not found: {model_path}\n"
                f"Export best.pt to NCNN into {models / 'robosort_ncnn_model'} "
                f"(the Pi), or copy it to {models / 'robosort.pt'} (laptop)."
            )

        if (path / "model.ncnn.param").exists():
            self._backend = _NcnnBackend(path)
        else:
            self._backend = _UltralyticsBackend(path)
        self.confidence = confidence
        self.infer_size = infer_size
        self.zone = zone                # None: any detection sends B/N
        self.model_path = str(path)
        self._imgsz_cache: dict[tuple[int, int], tuple[int, int]] = {}

    def warmup(self, frame) -> float:
        """First inference allocates buffers and is always slow. Run it once
        before timing anything, or every benchmark reports a lie."""
        start = time.perf_counter()
        self._infer(frame)
        return (time.perf_counter() - start) * 1000

    def imgsz_for(self, frame) -> tuple[int, int]:
        """Inference size derived from the frame the camera actually produced.

        The aspect ratio comes from the camera, so a 16:9 frame is fed as
        16:9 - the model letterboxes far less and nothing is stretched. Only
        the long side is fixed, and both sides round up to the stride.
        An NCNN export has one fixed input size, and that is what it gets.
        """
        if self._backend.fixed_size:
            return self._backend.fixed_size

        h, w = frame.shape[:2]
        key = (w, h)
        if key in self._imgsz_cache:
            return self._imgsz_cache[key]

        if self.infer_size <= 0:
            size = (_round_stride(h), _round_stride(w))     # camera native
        else:
            scale = self.infer_size / max(w, h)
            size = (_round_stride(int(h * scale)), _round_stride(int(w * scale)))

        self._imgsz_cache[key] = size
        return size

    def _infer(self, frame) -> list[tuple[str, float, tuple]]:
        return self._backend.predict(frame, self.imgsz_for(frame))

    def classify(self, frame) -> Detection:
        start = time.perf_counter()
        found = self._infer(frame)
        latency_ms = (time.perf_counter() - start) * 1000

        # The best box overall - unless a zone is set and something worth
        # picking up is inside it, which wins over a stronger box elsewhere.
        best = max(found, key=lambda d: d[1], default=None)
        if self.zone is not None:
            reachable = [
                d for d in found
                if d[1] >= self.confidence
                and CLASS_MAP.get(d[0].lower(), NONE) != NONE
                and in_zone(d[2], self.zone)
            ]
            if reachable:
                best = max(reachable, key=lambda d: d[1])

        if best is None:
            return Detection(None, 0.0, NONE, latency_ms)
        name, conf, box = best
        in_reach = self.zone is None or in_zone(box, self.zone)
        if conf >= self.confidence and in_reach:
            command = CLASS_MAP.get(name.lower(), NONE)
        else:
            command = NONE
        return Detection(name, conf, command, latency_ms, box, in_reach)
