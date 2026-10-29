"""
RoboSort - Vision Pipeline Entry Point (laptop + Orange Pi One)

Wires the three modules together and runs the loop:

    capture.py  -> frames        (the only camera-aware module)
    classify.py -> bio/non-bio   (the only model-aware module)
    serial_link -> the Mega      (the only serial-aware module)

Develop here on the laptop, then copy the folder to the Pi unchanged -
config.py picks the profile from the OS.

Connecting resets the Mega, so it runs its 10 s start-up countdown and
then drives on its own; this program only reports what the camera sees.
Trash is reported as B/N only once it is inside the pickup zone (config.py
PICKUP_ZONE, ROBOSORT_PICKUP_ZONE to override) - anywhere else it sends X.
Everything the Mega prints shows up here as "mega: ...". Stopping this
program does not stop the rover - send 5 (python mega.py 5) or switch
it off.

Usage:
    python vision.py                       # webcam + serial, per profile
    python vision.py --dry-run             # no Arduino needed
    python vision.py --source ../data/test # replay saved images
    python vision.py --no-display          # headless (default on the Pi)
    python vision.py --conf 0.4 --model runs/train/.../best.pt
"""

import argparse
import time

import capture
import classify
import serial_link
from config import CFG


def parse_args():
    p = argparse.ArgumentParser(description="RoboSort vision pipeline")
    p.add_argument("--source", default=CFG.camera,
                   help="camera index or path to an image file/folder")
    p.add_argument("--model", default=CFG.model_path)
    p.add_argument("--conf", type=float, default=CFG.confidence)
    p.add_argument("--port", default=CFG.serial_port)
    p.add_argument("--dry-run", action="store_true",
                   help="print serial commands instead of sending them")
    p.add_argument("--no-display", action="store_true",
                   help="never open a window (required over SSH)")
    p.add_argument("--capture", default=CFG.capture_size,
                   help="camera resolution: native, max, or WxH")
    p.add_argument("--infer-size", type=int, default=CFG.infer_size,
                   help="long side fed to the model (0 = camera resolution)")
    return p.parse_args()


def draw_overlay(frame, det: classify.Detection) -> None:
    import cv2

    classify.draw_box_and_zone(frame, det, CFG.pickup_zone)
    label = str(det)
    color = classify.COLORS.get(det.command, (180, 180, 180))
    cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def main():
    args = parse_args()
    show = CFG.display and not args.no_display

    print(f"RoboSort vision  [{CFG.describe()}]")

    # Only trash inside the pickup zone sends B/N - the Mega starts a pickup
    # the moment it gets one, and the arm reaches that one spot.
    model = classify.Classifier(args.model, args.conf, args.infer_size,
                                zone=CFG.pickup_zone)
    source = capture.open_source(args.source, args.capture)
    link = serial_link.open_link(
        args.port, CFG.baud_rate, CFG.serial_retries, dry_run=args.dry_run
    )

    if show:
        import cv2

    print("Running. Ctrl+C to stop." if source.is_live else "Replaying images.")

    warmed = False
    try:
        for label, frame in source.frames():
            if not warmed:
                # First inference allocates buffers and takes ~2 s. Burn it
                # before the loop so the rover is not blind on frame one.
                model.warmup(frame)
                warmed = True

            det = model.classify(frame)

            sent = link.send(det.command)
            if sent and source.is_live:
                print(f"  sent {det.command}  ({det})")

            if show:
                draw_overlay(frame, det)
                cv2.imshow("RoboSort Vision", frame)
                key = cv2.waitKey(1 if source.is_live else 400) & 0xFF
                if key == ord("q"):
                    break
            elif not source.is_live:
                mark = "->" if sent else "  "
                print(f"  {mark} {label}: {det}")

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        source.close()
        link.close()
        if link.resets:
            print(f"The Mega reset {link.resets} time(s) during the run - "
                  "check its power (see the wiring notes).")
        if show:
            import cv2
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
