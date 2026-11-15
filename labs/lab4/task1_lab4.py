"""Run Lab 4 Task 1 trilateration or stationary sensor diagnostics.

From the hambot_lab directory on the HamBot:
    python3 labs/lab4/task1_lab4.py --localize --hues 30 0 80 110 \
        --aim-x 180 214 213 210 --radius-m 0.04 --pixels-per-degree 30
    python3 labs/lab4/task1_lab4.py --diagnose --hue 80

Add --trace during localization to print measurement decisions and save photos.
Hues and aim columns are ordered yellow, red, green, blue; calibrate on site.
"""

import argparse
import math
from pathlib import Path
import sys


def main():
    """Validate calibration arguments before connecting to the robot."""
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--diagnose", action="store_true",
                       help="read the HamBot sensors without driving the wheels")
    modes.add_argument("--localize", action="store_true",
                       help="rotate the HamBot to measure three cones")
    parser.add_argument("--hue", type=int, choices=range(180), metavar="0-179")
    parser.add_argument("--hues", type=int, nargs=4,
                        metavar=("YELLOW", "RED", "GREEN", "BLUE"))
    parser.add_argument("--aim-x", type=float, nargs="+",
                        help="one image column or four in yellow, red, green, blue order")
    parser.add_argument("--radius-m", type=float, help="cone radius at LiDAR height in meters")
    parser.add_argument("--pixels-per-degree", type=float, default=30.0,
                        help="local camera-to-LiDAR scale in pixels per degree")
    parser.add_argument("--seconds", type=int)
    parser.add_argument("--trace", action="store_true",
                        help="report camera alignment and LiDAR checks during localization")
    args = parser.parse_args()
    if args.seconds is not None and args.seconds <= 0:
        parser.error("--seconds must be positive.")
    if args.diagnose:
        if args.hue is None:
            parser.error("--diagnose requires --hue.")
    else:
        if not math.isfinite(args.pixels_per_degree) or args.pixels_per_degree <= 0:
            parser.error("--pixels-per-degree must be finite and positive.")
        if args.hues is None or any(hue not in range(180) for hue in args.hues):
            parser.error("--localize requires four --hues in the range 0 to 179.")
        if (args.aim_x is None or len(args.aim_x) not in (1, 4)
                or any(not math.isfinite(x) or x < 0 for x in args.aim_x)
                or args.radius_m is None or not math.isfinite(args.radius_m)
                or args.radius_m <= 0):
            parser.error("--localize requires a valid --aim-x and positive --radius-m.")

    # Resolve control/MyRobot.py even when this script is run by its file path.
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from control.MyRobot import MyRobot

    robot = MyRobot()
    try:
        if args.diagnose:
            robot.diagnose(args.hue, args.seconds or 10)
        else:
            hues = dict(zip(robot.LANDMARKS, args.hues))
            columns = args.aim_x * 4 if len(args.aim_x) == 1 else args.aim_x
            robot.localize(
                hues, dict(zip(robot.LANDMARKS, columns)), args.radius_m,
                args.seconds or 90, trace=args.trace,
                pixels_per_degree=args.pixels_per_degree,
            )
    finally:
        robot.disconnect_robot()


if __name__ == "__main__":
    main()
