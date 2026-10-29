

import argparse
import statistics
import time

import capture
import classify
from config import CFG, ROOT

PI_SLOWDOWN = 20        # conservative middle of the 10-30x range
TARGET_FPS = 2.0        # the rover is slow; 2 classifications/sec is workable


def parse_args():
    p = argparse.ArgumentParser(description="RoboSort inference benchmark")
    p.add_argument("--source", default=str(ROOT / "data" / "test" / "images"),
                   help="image file or folder to benchmark over")
    p.add_argument("--model", default=CFG.model_path)
    p.add_argument("--conf", type=float, default=CFG.confidence)
    p.add_argument("--limit", type=int, default=100,
                   help="max images to run (0 = all)")
    p.add_argument("--infer-size", type=int, default=CFG.infer_size,
                   help="long side fed to the model (0 = source resolution)")
    return p.parse_args()


def percentile(values: list[float], pct: float) -> float:
    ordered = sorted(values)
    idx = min(int(len(ordered) * pct), len(ordered) - 1)
    return ordered[idx]


def main():
    args = parse_args()

    print(f"RoboSort benchmark  [{CFG.profile}]")
    print(f"  model:  {args.model}")
    print(f"  source: {args.source}")
    print(f"  infer:  long side {args.infer_size or 'camera native'}")
    print()

    load_start = time.perf_counter()
    model = classify.Classifier(args.model, args.conf, args.infer_size)
    print(f"  model load: {(time.perf_counter() - load_start):.1f} s")

    source = capture.open_source(args.source, CFG.capture_size)
    frames = list(source.frames())
    if args.limit:
        frames = frames[: args.limit]
    if not frames:
        raise SystemExit("No frames to benchmark.")

    h, w = model.imgsz_for(frames[0][1])
    fh, fw = frames[0][1].shape[:2]
    print(f"  source frames: {fw}x{fh}   model input: {w}x{h}")

    warm = model.warmup(frames[0][1])
    print(f"  warmup frame: {warm:.0f} ms (discarded)")
    print(f"  benchmarking {len(frames)} frames...\n")

    latencies: list[float] = []
    counts = {classify.BIO: 0, classify.NON_BIO: 0, classify.NONE: 0}

    for label, frame in frames:
        det = model.classify(frame)
        latencies.append(det.latency_ms)
        counts[det.command] += 1

    median = statistics.median(latencies)
    p95 = percentile(latencies, 0.95)
    fps = 1000 / median

    print("Latency (per frame)")
    print(f"  median   {median:8.1f} ms   ({fps:.1f} FPS)")
    print(f"  mean     {statistics.mean(latencies):8.1f} ms")
    print(f"  p95      {p95:8.1f} ms")
    print(f"  min/max  {min(latencies):.1f} / {max(latencies):.1f} ms")
    print()

    print("Detections")
    print(f"  bio      {counts[classify.BIO]}")
    print(f"  non-bio  {counts[classify.NON_BIO]}")
    print(f"  none     {counts[classify.NONE]}")
    print()

    if CFG.profile == "pi":
        verdict = "OK" if fps >= TARGET_FPS else "TOO SLOW"
        print(f"Measured on the Pi: {fps:.1f} FPS - {verdict} "
              f"(target {TARGET_FPS} FPS)")
    else:
        projected = fps / PI_SLOWDOWN
        print(f"Projected on Orange Pi One (~{PI_SLOWDOWN}x slower): "
              f"{median * PI_SLOWDOWN:.0f} ms, {projected:.2f} FPS")
        print(f"  Projection holds only for this model input size ({w}x{h}).")
        print("  A desktop CPU is overhead-bound at small inputs - raising the")
        print("  input size costs it almost nothing, and costs the Cortex-A7")
        print("  roughly its pixel count. Benchmark the size you intend to ship.")
        if projected < TARGET_FPS:
            print(f"  WARNING: below the {TARGET_FPS} FPS target. Shrink imgsz, "
                  f"export to NCNN, or use a smaller model before porting.")
        else:
            print(f"  Headroom looks fine against the {TARGET_FPS} FPS target.")

    source.close()


if __name__ == "__main__":
    main()
