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
Trash outside the pickup zone (config.py PICKUP_ZONE, ROBOSORT_PICKUP_ZONE
to override) sends X. Inside it, short of the grab spot (GRAB_SPOT), it
sends F, L or R, and the Mega steps toward it - one step per frame, the
next frame taken once the step is over. At the grab spot it sends B/N: lift,
belt, gate. See Pilot for the rules on top of that.
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
import os
import time

import capture
import classify
import config
import serial_link
from config import CFG

THERMAL_ZONE = "/sys/class/thermal/thermal_zone0/temp"
# After a pause, resume this many deg C below max_temp. Small, because the
# Pi with no heatsink idles only ~10 C under the limit and never gets lower.
COOL_MARGIN = 7
# Within SLOW_MARGIN of max_temp, rest SLOW_REST_S between frames. Back to
# back, one or two frames took the bare H3 from 88 C to 96-100 C - past the
# limit before the next check.
SLOW_MARGIN = 4
SLOW_REST_S = 1.0

# After an F/L/R the Mega moves up to 500 ms (main.ino APPROACH_STEP_MS),
# and the rover coasts. The
# next frame has to show where the step left it, or the item looks as far
# away as before and the rover steps past it.
STEP_SETTLE_S = 0.8
# Pickups tried on one item before giving up on it. A scoop that missed
# leaves the item at the grab spot, and a mark on the floor the model takes
# for trash would otherwise be "picked up" for ever.
MAX_PICKUP_TRIES = 2
# Approach steps spent on one item before giving up on it. The zone reaches
# the far floor, a good many steps away, but something on the rover itself
# taken for trash never gets nearer - without a cap the rover would turn
# toward it for ever.
MAX_APPROACH_STEPS = 40


class Pilot:
    """What to send the Mega, from what the classifier sees, with rules one
    frame cannot know:
      - B/N only right after an approach step, so the frame that starts the
        lift was taken with the rover standing still. Trash first seen at the
        grab spot while driving sends F instead: the Mega stops on its first
        F (main.ino serviceApproach), and the next frame decides.
      - At most MAX_PICKUP_TRIES pickups and MAX_APPROACH_STEPS steps per
        item, then X until a frame with no trash in reach."""

    def __init__(self):
        self.tries = 0
        self.steps = 0

    @property
    def gave_up(self) -> bool:
        return self.tries >= MAX_PICKUP_TRIES or self.steps >= MAX_APPROACH_STEPS

    def command(self, seen: str, last_sent: str | None) -> str:
        if seen == classify.NONE:
            self.tries = self.steps = 0
            return classify.NONE
        if self.gave_up:
            return classify.NONE
        if seen in (classify.BIO, classify.NON_BIO) and last_sent not in classify.APPROACH:
            return classify.AHEAD
        return seen

    def sent(self, command: str) -> None:
        if command in (classify.BIO, classify.NON_BIO):
            self.tries += 1
            self.steps = 0           # a missed scoop gets a fresh approach
        elif command in classify.APPROACH:
            self.steps += 1


def cpu_temp() -> float | None:
    """The SoC temperature in deg C; None where there is no such file (Windows)."""
    try:
        with open(THERMAL_ZONE) as f:
            return int(f.read()) / 1000
    except (OSError, ValueError):
        return None


def cool_down(link, temp: float) -> None:
    """Too hot: stop running the model until the chip has cooled. Without a
    heatsink the H3 reached 105 C and the kernel powered the Pi off mid-run.
    X goes out first, so the Mega does not act on an old sighting. The link's
    heartbeat runs on its own thread, so the Mega still sees the Pi as alive."""
    resume = CFG.max_temp - COOL_MARGIN
    link.send(classify.NONE)
    print(f"  CPU {temp:.0f} C - pausing the model until it is down to {resume:g} C")
    start = time.monotonic()
    while (temp := cpu_temp()) is not None and temp > resume:
        time.sleep(2)
    print(f"  CPU {temp:.0f} C - resuming after {time.monotonic() - start:.0f} s")


def wait_for_camera(source: str) -> str:
    """On the Pi with no USB camera plugged in, wait here until there is one
    and return its index. Exiting instead makes systemd start this program
    again every 10 s, and each start - Python, OpenCV - is ~9 s of full CPU
    on the Pi: with no heatsink that took the H3 to 101 C, in a loop.
    Anything named on purpose (--source, ROBOSORT_CAMERA) is used as given."""
    if CFG.profile != "pi" or source != CFG.camera or "ROBOSORT_CAMERA" in os.environ:
        return source
    found = config.find_camera(None)
    if found is None:
        print("No USB camera plugged in - waiting for one (no restart needed).")
        while (found := config.find_camera(None)) is None:
            time.sleep(3)
        time.sleep(2)                # let it finish setting up
        print(f"USB camera found: index {found}")
    return found


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


def draw_overlay(frame, det: classify.Detection, model: classify.Classifier) -> None:
    import cv2

    classify.draw_box_and_zone(frame, det, model.zone, model.grab)
    label = str(det)
    color = classify.COLORS.get(det.command, (180, 180, 180))
    cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def main():
    args = parse_args()
    show = CFG.display and not args.no_display

    print(f"RoboSort vision  [{CFG.describe()}]")

    # The camera first, so a missing one is waited for before the model
    # load - seconds of full CPU on the Pi.
    source = capture.open_source(wait_for_camera(args.source), args.capture)
    # Only trash at the grab spot sends B/N - the Mega starts a pickup the
    # moment it gets one, and the arm reaches that one spot. Elsewhere in
    # the zone it is F/L/R, a step toward it.
    model = classify.Classifier(args.model, args.conf, args.infer_size,
                                zone=CFG.pickup_zone, threads=CFG.threads,
                                grab=CFG.grab_spot)
    pilot = Pilot()
    link = serial_link.open_link(
        args.port, CFG.baud_rate, CFG.serial_retries, dry_run=args.dry_run
    )

    if show:
        import cv2

    print("Running. Ctrl+C to stop." if source.is_live else "Replaying images.")

    warmed = False
    try:
        for label, frame in source.frames():
            # Mid-pickup nothing can be sent, so skip the model: less heat.
            if link.busy:
                time.sleep(0.25)
                continue

            if CFG.max_temp and (temp := cpu_temp()) is not None:
                # The warm-up is the heaviest frame (~2 s of full load; it
                # once took the Pi from 94 C to 102 C), so it starts cool.
                hot = (temp >= CFG.max_temp if warmed
                       else temp > CFG.max_temp - COOL_MARGIN)
                if hot:
                    cool_down(link, temp)
                    continue
                if temp >= CFG.max_temp - SLOW_MARGIN:
                    time.sleep(SLOW_REST_S)
                    continue                 # a fresh frame, and a fresh reading

            if not warmed:
                # First inference allocates buffers and takes ~2 s. Burn it
                # before the loop so the rover is not blind on frame one.
                model.warmup(frame)
                warmed = True

            det = model.classify(frame)

            command = pilot.command(det.command, link.last_command)
            sent = link.send(command)
            if sent:
                pilot.sent(command)
            if sent and source.is_live:
                note = "" if command == det.command else "  - stopping to look again"
                if command == classify.NONE and det.command != classify.NONE:
                    note = (f"  - gave up on it ({pilot.tries} pickups, "
                            f"{pilot.steps} steps); X until nothing is in reach")
                print(f"  sent {command}  ({det}){note}")
            if sent and command in classify.APPROACH and source.is_live:
                time.sleep(STEP_SETTLE_S)    # the next frame shows the step done

            if show:
                draw_overlay(frame, det, model)
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
