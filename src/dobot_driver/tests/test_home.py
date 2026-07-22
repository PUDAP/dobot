#!/usr/bin/env python3
"""Connect to the Dobot M1 Pro and move it to the configured home pose.

The Dobot controller accepts only one TCP client at a time. Stop the edge
service first if it is running::

    docker stop dobot-edge

Run from the repo root::

    DOBOT_IP=192.168.2.6 uv run python src/dobot_driver/tests/test_home.py
"""

from __future__ import annotations

import argparse
import logging
import os
import sys

from dobot_driver import M1Pro

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

DEFAULT_HOME_POSITION = [200, 0, 240, 20]
TEST_MOVE_POSITION = {"x": 131, "y": -231, "z": 240, "r": -20}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Home the Dobot M1 Pro.")
    parser.add_argument(
        "--ip",
        default=os.environ.get("DOBOT_IP", "192.168.2.6"),
        help="Dobot controller IP address (default: DOBOT_IP env or 192.168.2.6).",
    )
    parser.add_argument(
        "--speed-factor",
        type=float,
        default=0.2,
        help="Motion speed scaling factor in (0.0, 1.0] (default: 0.2).",
    )
    parser.add_argument(
        "--home-position",
        nargs=4,
        type=float,
        metavar=("X", "Y", "Z", "R"),
        default=DEFAULT_HOME_POSITION,
        help="Robot-frame home pose as X Y Z R (default: 200 0 240 20).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logger.info("Connecting to Dobot at %s", args.ip)

    try:
        with M1Pro(
            dobot_ip=args.ip,
            home_position=args.home_position,
        ) as arm:
            arm.set_speed_factor(args.speed_factor)

            pose_before = arm.get_pose(frame="robot")
            logger.info("Pose before home: %s", pose_before)

            home_pose = arm.home(speed_factor=args.speed_factor)
            logger.info("Home command completed")

            pose_after = arm.get_pose(frame="robot")
            logger.info("Pose after home:  %s", pose_after)
            logger.info("Target home:      %s", home_pose)

            move_pose = arm.safe_move(
                TEST_MOVE_POSITION,
                frame="robot",
                speed_factor_lateral=args.speed_factor,
            )
            logger.info("Move command completed")

            pose_after_move = arm.get_pose(frame="robot")
            logger.info("Pose after move:  %s", pose_after_move)
            logger.info("Target move:      %s", move_pose)

            joints = arm.get_joint_angles()
            logger.info("Joint angles:     %s", joints)
    except KeyboardInterrupt:
        logger.warning("Interrupted")
        return 130
    except Exception:
        logger.exception("Failed to home Dobot")
        return 1

    logger.info("Done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
