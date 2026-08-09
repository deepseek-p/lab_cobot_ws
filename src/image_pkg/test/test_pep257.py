"""Run pep257 over the migrated perception package."""
from pathlib import Path

from ament_pep257.main import main
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LINT_PATHS = [
    str(PACKAGE_ROOT / "image_pkg"),
    str(PACKAGE_ROOT / "launch"),
    str(PACKAGE_ROOT / "test"),
    str(PACKAGE_ROOT / "setup.py"),
]


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Require public docstrings to pass the ROS 2 pep257 profile."""
    assert main(argv=LINT_PATHS) == 0
