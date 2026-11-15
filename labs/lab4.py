"""Compatibility launcher for labs/lab4/task1_lab4.py.

The trilateration implementation now lives in control/MyRobot.py.
All existing --localize, --diagnose, and calibration arguments are supported.
"""

from pathlib import Path
import runpy


def main():
    runpy.run_path(
        str(Path(__file__).resolve().parent / "lab4" / "task1_lab4.py"),
        run_name="__main__",
    )


if __name__ == "__main__":
    main()
