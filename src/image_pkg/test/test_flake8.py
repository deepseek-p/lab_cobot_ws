"""Run flake8 over the migrated perception package."""
from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LINT_PATHS = [
    str(PACKAGE_ROOT / "image_pkg"),
    str(PACKAGE_ROOT / "launch"),
    str(PACKAGE_ROOT / "test"),
    str(PACKAGE_ROOT / "setup.py"),
]


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Require the package to pass the ROS 2 flake8 profile."""
    result, errors = main_with_errors(argv=LINT_PATHS)
    assert result == 0, (
        "Found %d code style errors / warnings:\n" % len(errors)
        + "\n".join(errors)
    )
