"""
RoboSort - Vision-Only Test (no Arduino, no serial, no rover)

Runs capture -> classify and stops there. Nothing is sent anywhere, so this
is safe to run with no hardware attached at all: it answers "does the model
see the trash correctly?" and nothing else.

Two modes, picked by --source:

    python test_cv.py                       # live webcam window
    python test_cv.py --source ../data/test/images   # score a folder
    python test_cv.py --source photo.jpg    # single image

Live keys:  q quit   s save the current frame to output/

Folder mode prints a per-image line and a summary. If the folder is one of
the class folders under data/ (data/plastic, data/paper, ...) the expected
label is taken from the folder name and you get a hit rate.

    python test_cv.py --source ../data/plastic --expect non-bio
    python test_cv.py --source ../data/test/images --save
"""

import argparse
import os
import statistics
import time
from collections import Counter
from pathlib import Path

import cv2

import capture
import classify
from config import CFG, ROOT

OUTPUT_DIR = ROOT / "output"

# The trained TRASH model (Leaves, Metal, Paper, Plastic) in weights/ on the
# laptop. The Pi only receives orange_pi/, so there it falls back to
# config.py's pick - the NCNN export. --model or ROBOSORT_MODEL override both.
_WEIGHTS = ROOT / "weights" / "best.pt"
DEFAULT_MODEL = (str(_WEIGHTS) if _WEIGHTS.exists() and "ROBOSORT_MODEL" not in os.environ
                 else CFG.model_path)

# Folder name -> what the classifier should be calling it. Used to score a
# run when you point the script at one of the sorted data/ folders.
FOLDER_TRUTH = {
    "cardboard": "bio", "paper": "bio", "bread": "bio", "eggs": "bio",
    "peels": "bio", "walnuts": "bio", "leaves": "bio", "trash": None,
    "plastic": "non-bio", "metal": "non-bio", "polystyrene": "non-bio",
}

CATEGORY = {classify.BIO: "bio", classify.NON_BIO: "non-bio", classify.NONE: None}


def parse_args():
    p = argparse.ArgumentParser(description="RoboSort vision-only test")
    p.add_argument("--source", default=CFG.camera,
                   help="camera index, image file, or folder (default: webcam)")
    p.add_argument("--model", default=DEFAULT_MODEL)
    p.add_argument("--conf", type=float, default=CFG.confidence)
    p.add_argument("--limit", type=int, default=0,
                   help="max images in folder mode (0 = all)")
    p.add_argument("--expect", choices=["bio", "non-bio"], default=None,
                   help="ground truth for every image, overrides folder name")
    p.add_argument("--save", action="store_true",
                   help="write annotated images to output/")
    p.add_argument("--no-display", action="store_true",
                   help="never open a window (required over SSH)")
    p.add_argument("--frames", type=int, default=0,
                   help="stop after N webcam frames (0 = until you press q)")
    p.add_argument("--seconds", type=float, default=0.0,
                   help="stop after N seconds of webcam (0 = until you press q)")
    p.add_argument("--list-cameras", action="store_true",
                   help="show which camera indices work, then exit")
    p.add_argument("--capture", default=CFG.capture_size,
                   help="camera resolution: native, max, or WxH (default: native)")
    p.add_argument("--infer-size", type=int, default=CFG.infer_size,
                   help="long side fed to the model (0 = full camera resolution)")
    return p.parse_args()


def annotate(frame, det: classify.Detection, fps: float | None = None, zone=None):
    """Only the single best detection is ever acted on, so that is the one
    box drawn - with the pickup zone, when one is in use."""
    color = classify.COLORS.get(det.command, (170, 170, 170))

    classify.draw_box_and_zone(frame, det, zone)
    cv2.rectangle(frame, (0, 0), (frame.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(frame, str(det), (8, 23),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    if fps is not None:
        text = f"{fps:4.1f} FPS"
        cv2.putText(frame, text, (frame.shape[1] - 95, 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    return frame


def expected_label(args, source) -> str | None:
    if args.expect:
        return args.expect
    if isinstance(source, capture.FolderSource):
        return FOLDER_TRUTH.get(source.path.name.lower())
    return None


def run_live(model: classify.Classifier, source, show: bool):
    print(f"  {source.describe()}")
    if show:
        print("Live vision test. q = quit, s = save frame.\n")
    else:
        print("Live vision test, headless. Ctrl+C to stop.\n")
    OUTPUT_DIR.mkdir(exist_ok=True)

    warmed = False
    recent: list[float] = []
    last_command = None
    saved = 0
    frame_count = 0

    for label, frame in source.frames():
        frame_count += 1
        if not warmed:
            model.warmup(frame)
            warmed = True

        det = model.classify(frame)
        recent.append(det.latency_ms)
        recent = recent[-30:]
        fps = 1000 / statistics.median(recent)

        # Only log transitions, otherwise the console is unreadable.
        if det.command != last_command:
            print(f"  {det}")
            last_command = det.command

        if show:
            cv2.imshow("RoboSort - vision only (no serial)",
                       annotate(frame, det, fps, model.zone))
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                saved += 1
                out = OUTPUT_DIR / f"cvtest_{int(time.time())}_{saved}.jpg"
                cv2.imwrite(str(out), frame)
                print(f"  saved {out.name}")

    if show:
        cv2.destroyAllWindows()
    if not recent:
        print("\n  no frames captured")
        return
    median = statistics.median(recent)
    print(f"\n  {frame_count} frames   median {median:.1f} ms "
          f"({1000 / median:.1f} FPS)   {saved} saved")


def run_folder(model: classify.Classifier, source, args, truth: str | None):
    frames = list(source.frames())
    if args.limit:
        frames = frames[: args.limit]

    if truth:
        print(f"Scoring {len(frames)} images, expecting: {truth}\n")
    else:
        print(f"Classifying {len(frames)} images (no ground truth)\n")

    if args.save:
        OUTPUT_DIR.mkdir(exist_ok=True)

    model.warmup(frames[0][1])

    latencies, seen = [], Counter()
    hits = misses = unsure = 0

    for label, frame in frames:
        det = model.classify(frame)
        latencies.append(det.latency_ms)
        seen[det.class_name or "-"] += 1
        got = CATEGORY[det.command]

        mark = " "
        if truth:
            if got is None:
                unsure += 1
                mark = "?"
            elif got == truth:
                hits += 1
                mark = "+"
            else:
                misses += 1
                mark = "X"

        print(f"  {mark} {label[:44]:<44} {det}")

        if args.save:
            cv2.imwrite(str(OUTPUT_DIR / f"cvtest_{label}"), annotate(frame, det))

    print()
    print(f"Latency  median {statistics.median(latencies):.1f} ms   "
          f"mean {statistics.mean(latencies):.1f} ms   "
          f"({1000 / statistics.median(latencies):.1f} FPS on this machine)")

    print("\nMost seen classes")
    for name, count in seen.most_common(6):
        print(f"  {name:<16} {count}")

    if truth:
        scored = hits + misses
        rate = (hits / scored * 100) if scored else 0.0
        print(f"\nAgainst '{truth}'")
        print(f"  correct    {hits}")
        print(f"  wrong      {misses}")
        print(f"  no call    {unsure}   (below conf {args.conf}, sends 'X')")
        print(f"  hit rate   {rate:.1f}%  of the {scored} it committed to")

    if args.save:
        print(f"\nAnnotated images written to {OUTPUT_DIR}")


def main():
    args = parse_args()
    show = CFG.display and not args.no_display

    if args.list_cameras:
        found = capture.list_cameras()
        if not found:
            print("No working cameras found.")
            print("  Windows: close Teams/Zoom/Camera and retry.")
            print("  Orange Pi: check /dev/video* exists and you are in the "
                  "'video' group.")
        else:
            print("Working cameras:")
            for cam in found:
                d = "x".join(str(v) for v in cam["default"])
                m = "x".join(str(v) for v in cam["max"])
                note = "" if d == m else f"   (--capture max -> {m})"
                print(f"  --source {cam['index']}   default {d}{note}")
        return

    print(f"RoboSort vision-only test  [{CFG.profile}]")
    print(f"  model:  {args.model}")
    print(f"  source: {args.source}")
    print(f"  conf:   {args.conf}")
    print(f"  infer:  {args.infer_size or 'camera native'}"
          f"{'' if args.infer_size else ' (slow - full resolution)'}")
    print("  serial: NOT USED - this script never opens a port")

    source = capture.open_source(
        args.source, args.capture,
        max_frames=args.frames, max_seconds=args.seconds,
    )
    # The pickup zone only means something for the rover's own camera. A
    # folder of dataset photos has trash all over the frame, and scoring
    # it is about the model's labels, so folders run without it.
    zone = CFG.pickup_zone if source.is_live else None
    print(f"  zone:   {', '.join(f'{v:.2f}' for v in zone) if zone else 'off'}"
          f"{'  (B/N only inside it)' if zone else ''}\n")
    model = classify.Classifier(args.model, args.conf, args.infer_size, zone=zone)

    try:
        if source.is_live:
            run_live(model, source, show)
        else:
            run_folder(model, source, args, expected_label(args, source))
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        source.close()


if __name__ == "__main__":
    main()
