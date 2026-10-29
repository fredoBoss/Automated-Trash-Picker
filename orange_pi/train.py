"""
RoboSort - YOLO11n Training Script (run on the PC, never on the Orange Pi)
Trains an object-detection model on the Roboflow TRASH export, drops the
weights where config.py looks for them, then exports to NCNN for fast ARM
inference on the Orange Pi One.

Classes (from the Roboflow export's data.yaml):
  Bio     -> Leaves, Paper
  Non-bio -> Metal, Plastic

The export's labels are polygons. A detect model is still the right choice:
Ultralytics turns each polygon into its bounding box, and the pipeline only
needs a box and a class - a -seg model would be slower on the Pi for masks
nothing uses.

Usage:
    python train.py                         # full run, default dataset
    python train.py --epochs 1              # quick check that everything loads
    python train.py --data "D:/other/data.yaml"
"""

import argparse
import shutil
from pathlib import Path

import torch
import yaml
from ultralytics import YOLO

HERE = Path(__file__).resolve().parent

DATASET_YAML = "C:/Users/alfre/Projects/YOLO DATA/TRASH.v1i.yolov11/data.yaml"
EPOCHS       = 100
IMAGE_SIZE   = 320      # must match infer_size in config.py
BATCH        = 16
PROJECT_DIR  = "C:/Users/alfre/Projects/RoboSort/runs/train"
RUN_NAME     = "robosort_yolo11"

# Where config.py looks first - the orange_pi/ folder is what goes to the Pi.
DEPLOY_PT   = HERE / "models" / "robosort.pt"
DEPLOY_NCNN = HERE / "models" / "robosort_ncnn_model"


def parse_args():
    p = argparse.ArgumentParser(description="RoboSort YOLO11n training")
    p.add_argument("--data", default=DATASET_YAML, help="path to data.yaml")
    p.add_argument("--epochs", type=int, default=EPOCHS)
    return p.parse_args()


def resolve_dataset(data_yaml: str) -> str:
    """
    Roboflow writes the splits as ../train/images. Ultralytics resolves that
    against the folder ABOVE the export, and only falls back to the export's
    own folder if nothing is there. Here something is: YOLO DATA/train holds
    the old 9-class dataset, so training silently used those images and died
    on class ids 4-8 with a CUDA device-side assert. Pin every split to the
    export's own folder and write the result next to the training runs.
    """
    src = Path(data_yaml).resolve()
    cfg = yaml.safe_load(src.read_text())
    root = Path(cfg.pop("path", src.parent))
    if not root.is_absolute():
        root = src.parent / root

    for split in ("train", "val", "test"):
        if split not in cfg:
            continue
        rel = str(cfg[split]).replace("\\", "/")
        if Path(rel).is_absolute():
            p = Path(rel)
        else:
            while rel.startswith("../"):
                rel = rel[3:]
            p = root / rel
        if not p.exists():
            raise FileNotFoundError(f"{split} split not found: {p}")
        cfg[split] = str(p)

    out = Path(PROJECT_DIR) / "data.resolved.yaml"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(yaml.safe_dump(cfg, sort_keys=False))
    print(f"Dataset: {cfg['names']}")
    for split in ("train", "val", "test"):
        if split in cfg:
            print(f"  {split:5s} {cfg[split]}")
    return str(out)


def train(data: str, epochs: int):
    model = YOLO("yolo11n.pt")  # pretrained YOLO11 nano - lightest, best for the Pi

    results = model.train(
        data=resolve_dataset(data),
        epochs=epochs,
        imgsz=IMAGE_SIZE,
        batch=BATCH,
        project=PROJECT_DIR,
        name=RUN_NAME,
        patience=20,
        device=0 if torch.cuda.is_available() else "cpu",
        workers=4,
    )

    print(f"\nTraining complete.")
    print(f"Best weights: {results.save_dir}/weights/best.pt")
    return results


def deploy_pt(weights_path: Path) -> None:
    DEPLOY_PT.parent.mkdir(exist_ok=True)
    shutil.copy2(weights_path, DEPLOY_PT)
    print(f"Copied: {DEPLOY_PT}")


def export(weights_path: Path) -> None:
    model = YOLO(str(weights_path))

    # NCNN: fastest inference on Orange Pi's ARM Cortex-A7 (no GPU)
    ncnn_dir = Path(model.export(format="ncnn", imgsz=IMAGE_SIZE))
    if DEPLOY_NCNN.exists():
        shutil.rmtree(DEPLOY_NCNN)
    shutil.copytree(ncnn_dir, DEPLOY_NCNN)
    print(f"Exported: NCNN -> {DEPLOY_NCNN}")

    # ONNX: portable fallback
    model.export(format="onnx", imgsz=IMAGE_SIZE, simplify=True)
    print("Exported: ONNX")


if __name__ == "__main__":
    args = parse_args()
    results = train(args.data, args.epochs)
    best_weights = Path(results.save_dir) / "weights" / "best.pt"
    # Copy before exporting, so a failed export still leaves a usable model.
    deploy_pt(best_weights)
    export(best_weights)
