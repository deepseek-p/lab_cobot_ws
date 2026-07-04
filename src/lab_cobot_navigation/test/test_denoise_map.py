import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


def test_denoise_map_uses_yaml_origin_for_start_pixel(tmp_path):
    package_dir = Path(__file__).resolve().parents[1]
    script = package_dir / "maps" / "denoise_map.py"
    real_map = package_dir / "maps" / "map.pgm"

    image_path = tmp_path / "stub.pgm"
    yaml_path = tmp_path / "map.yaml"
    output_path = tmp_path / "stub_denoised.pgm"

    image = np.full((4, 4), 254, dtype=np.uint8)
    image[1, 2] = 0
    image[0, 0] = 0
    Image.fromarray(image).save(image_path)
    yaml_path.write_text(
        yaml.safe_dump({
            "image": image_path.name,
            "resolution": 0.5,
            "origin": [-1.0, -1.0, 0.0],
        }),
        encoding="utf-8",
    )

    real_map_backup = real_map.read_bytes()
    try:
        result = subprocess.run(
            [
                sys.executable,
                str(script),
                "--map-yaml",
                str(yaml_path),
                "--output",
                str(output_path),
                "--clear-radius",
                "0",
                "--min-cluster-size",
                "0",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
    finally:
        real_map.write_bytes(real_map_backup)

    assert result.returncode == 0, result.stderr
    assert output_path.exists()

    unchanged_input = np.array(Image.open(image_path))
    assert int(unchanged_input[1, 2]) == 0

    denoised = np.array(Image.open(output_path))
    assert int(denoised[1, 2]) == 254
    assert int(denoised[0, 0]) == 0
    assert "起点(2,1)值" in result.stdout
