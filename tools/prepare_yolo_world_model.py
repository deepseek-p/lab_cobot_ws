#!/usr/bin/env python3
"""Prepare the slim YOLO-World model for lab perception."""
from pathlib import Path
import hashlib
import time


CLASSES = ("white box", "blue cylinder", "yellow box")
SOURCE_MODEL = "yolov8s-worldv2.pt"
OUTPUT_PATH = Path("~/lab_cobot_models/yolo_world_lab_slim.pt").expanduser()


def _md5(path: Path) -> str:
    digest = hashlib.md5()  # noqa: S324 - Artifact fingerprint only.
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _model_names(model) -> list[str]:
    names = model.names
    if isinstance(names, dict):
        return [names[index] for index in sorted(names)]
    return list(names)


def main() -> None:
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)

    from ultralytics import YOLO

    model = YOLO(SOURCE_MODEL)
    model.set_classes(list(CLASSES))
    if hasattr(model.model, "clip_model"):
        del model.model.clip_model
    model.save(str(OUTPUT_PATH))

    size_mb = OUTPUT_PATH.stat().st_size / (1024.0 * 1024.0)
    start = time.perf_counter()
    reloaded = YOLO(str(OUTPUT_PATH))
    load_ms = (time.perf_counter() - start) * 1000.0
    names = _model_names(reloaded)
    if names != list(CLASSES):
        raise RuntimeError(f"unexpected model names: {names}")

    print(f"output={OUTPUT_PATH}")
    print(f"classes={list(CLASSES)}")
    print(f"size_mb={size_mb:.2f}")
    print(f"md5={_md5(OUTPUT_PATH)}")
    print(f"cold_load_ms={load_ms:.1f}")


if __name__ == "__main__":
    main()
