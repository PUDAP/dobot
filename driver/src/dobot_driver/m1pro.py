from __future__ import annotations

import logging
import math
import time
from typing import Any, Mapping, Sequence

from .dobot_client import (
    PoseXYZR,
    DobotDeviceClient,
    NUMBER_PATTERN,
)

logger = logging.getLogger(__name__)
DEFAULT_HOME_POSITION = [[300, 0, 200], [-33, 0, 0]]
DEFAULT_CALIBRATED_OFFSET = [[-374, 496.75, 254.2], [-89.11611149, 0, 0]]
DEFAULT_TOOL_OFFSET = [[0, 0, -232], [122.11611149, 0, 0]]
DEFAULT_SAFE_HEIGHT = 240.0

class M1Pro:
    """Standalone M1 Pro controller using Dobot's TCP/IP API directly."""

    # Public API

    def __init__(
        self,
        dobot_ip: str,
        *,
        home_position: Any = DEFAULT_HOME_POSITION,
        calibrated_offset: Any = DEFAULT_CALIBRATED_OFFSET,
        scale: float = 1.0,
        tool_offset: Any = DEFAULT_TOOL_OFFSET,
        safe_height: float = DEFAULT_SAFE_HEIGHT,
        home_waypoints: Sequence[Any] | None = None,
        timeout: float = 10.0,
    ) -> None:
        """Initialize an M1 Pro controller.

        Args:
            dobot_ip (required): IP address or hostname for the Dobot controller.
            home_position (optional): Robot-frame pose used as the configured home position.
            calibrated_offset (optional): Work-frame calibration offset applied during frame transforms.
            scale (optional): Scalar factor between work coordinates and robot coordinates.
            tool_offset (optional): Tool pose offset applied between tool and robot frames.
            safe_height (optional): Z height used for clearance moves before lateral travel.
            home_waypoints (optional): Intermediate robot-frame waypoints visited before the final home pose.
            timeout (optional): Socket timeout in seconds for device communication.
        """
        self._scale = float(scale)
        self._safe_height = float(safe_height)
        self._home_position = self._to_pose_xyzr(home_position, default_r=-33.0)
        self._calibrated_offset = self._to_pose_xyzr(calibrated_offset)
        self._tool_offset = self._to_pose_xyzr(tool_offset)
        self._home_waypoints = [
            self._to_pose_xyzr(waypoint, default_r=self._home_position.r)
            for waypoint in (home_waypoints or [])
        ]
        self._logger = logger.getChild(self.__class__.__name__)
        self._logger.setLevel(logging.DEBUG)
        self._device = DobotDeviceClient(
            dobot_ip,
            timeout=timeout,
        )
        self._speed_factor = 0.2
        self._validate_robot_pose(self._home_position)
        self._connect()

    def __enter__(self) -> "M1Pro":
        """Connect the device for use in a context manager.

        Args:
            None.

        Returns:
            M1Pro: The connected controller instance.
        """
        self._connect()
        return self

    def open_gripper(self) -> str:
        """Open the gripper.

        Args:
            None.

        Returns:
            str: The raw device response.

        Raises:
            RuntimeError: If the dashboard API is not connected.
        """
        self._logger.debug("Opening gripper")
        return self._device.DOExecute(1, 1)

    def close_gripper(self) -> str:
        """Close the gripper.

        Args:
            None.

        Returns:
            str: The raw device response.

        Raises:
            RuntimeError: If the dashboard API is not connected.
        """
        self._logger.debug("Closing gripper")
        return self._device.DOExecute(1, 0)

    def reset(self) -> None:
        """Reset the underlying device controller.

        Args:
            None.
        """
        self._device.reset()

    def set_speed_factor(self, speed_factor: float) -> None:
        """Set the motion speed scaling factor.

        Args:
            speed_factor (required): Motion speed multiplier in the range `(0.0, 1.0]`.
        """
        if not 0.0 < speed_factor <= 1.0:
            raise ValueError("speed_factor must be in the range (0.0, 1.0].")
        self._speed_factor = float(speed_factor)
        self._device.SpeedFactor(max(1, min(100, int(round(speed_factor * 100)))))

    def get_pose(self, *, frame: str = "robot") -> PoseXYZR:
        """Get the current Cartesian pose.

        Args:
            frame (optional): Coordinate frame for the returned pose. Use `"robot"` or `"work"`.

        Returns:
            PoseXYZR: The current pose in the requested frame.
        """
        robot_pose = self._parse_pose_response(self._device.GetPose())
        if frame == "robot":
            return robot_pose
        if frame == "work":
            return self._robot_to_work_tool(robot_pose)
        raise ValueError("frame must be either 'robot' or 'work'.")

    def get_joint_angles(self) -> tuple[float, float, float, float, float, float]:
        """Get the current robot joint angles.

        Args:
            None.

        Returns:
            tuple[float, float, float, float, float, float]: The six reported joint angles.
        """
        values = self._parse_numeric_response(self._device.GetAngle(), minimum=6, take_last=6)
        return tuple(values[-6:])  # type: ignore[return-value]

    def safe_move(
        self,
        position: Mapping[str, float],
        *,
        frame: str = "robot",
        speed_factor_lateral: float | None = None,
        speed_factor_up: float | None = None,
        speed_factor_down: float | None = None,
        blocking: bool = True,
    ) -> PoseXYZR:
        """Move to a target pose using a vertical-lateral-vertical path.

        Args:
            target (required): Destination pose as a mapping, sequence, or `PoseXYZR`.
            frame (optional): Coordinate frame for `target`. Use `"robot"` or `"work"`.
            speed_factor_lateral (optional): Speed scaling used for the horizontal travel segment.
            speed_factor_up (optional): Speed scaling used for the initial upward move.
            speed_factor_down (optional): Speed scaling used for the final downward move.
            blocking (optional): Whether to wait for each move segment to complete before returning.

        Returns:
            PoseXYZR: The resolved destination in robot coordinates.
        """
        if position["y"] > 0:
            self._device.SetArmOrientation(right_handed=False)
        else:
            self._device.SetArmOrientation(right_handed=True)
        robot_target = self._resolve_robot_target(position, frame=frame)
        current = self.get_pose(frame="robot")
        travel_z = max(self._safe_height, current.z, robot_target.z)

        if current.z < travel_z:
            self._move(
                PoseXYZR(current.x, current.y, travel_z, current.r),
                frame="robot",
                speed_factor=speed_factor_up,
                blocking=blocking,
            )

        self._move(
            PoseXYZR(robot_target.x, robot_target.y, travel_z, robot_target.r),
            frame="robot",
            speed_factor=speed_factor_lateral,
            blocking=blocking,
        )

        if not math.isclose(robot_target.z, travel_z):
            self._move(
                robot_target,
                frame="robot",
                speed_factor=speed_factor_down,
                blocking=blocking,
            )

        return robot_target

    def home(self, *, speed_factor: float | None = None) -> PoseXYZR:
        """Move the robot to its configured home pose.

        Args:
            speed_factor (optional): Speed scaling applied to the safe-height move, any home waypoints, and the final home move.

        Returns:
            PoseXYZR: The final home pose in robot coordinates.
        """
        self._move_to_safe_height(speed_factor=speed_factor)
        self.open_gripper()
        for waypoint in self._home_waypoints:
            self._move(waypoint, frame="robot", speed_factor=speed_factor)
        return self._move(self._home_position, frame="robot", speed_factor=speed_factor)

    def pick_from(
        self,
        *,
        position: Mapping[str, float],
    ) -> Mapping[str, float]:
        """Move to a pick position, lift 30 mm, and close the gripper.

        Args:
            position (required): Pick pose as a mapping

        Returns:
            Mapping[str, float]: The final lifted pose as a mapping.
        """
        pick_pose = self.safe_move(position, frame="robot")
        self.open_gripper()
       
        lifted_pose = PoseXYZR(pick_pose.x, pick_pose.y, pick_pose.z - 30.0, pick_pose.r)
        final_pose = self._move(lifted_pose, frame="robot", speed_factor=0.05)
        self.close_gripper()
        
        return final_pose

    def place_to(
        self,
        *,
        position: Mapping[str, float],
    ) -> Mapping[str, float]:
        """Safe-move above a place position, descend slowly, and open the gripper.

        Args:
            position (required): Place pose as a mapping

        Returns:
            Mapping[str, float]: The final placed pose as a mapping.
        """
        above_pose = self.safe_move(position, frame="robot")
        target_pose = PoseXYZR(above_pose.x, above_pose.y, above_pose.z - 30.0, above_pose.r)
        final_pose = self._move(target_pose, frame="robot", speed_factor=0.05)
        self.open_gripper()
        return final_pose

    def __exit__(self, exc_type, exc, tb) -> None:
        """Disconnect the device when leaving a context manager.

        Args:
            exc_type (optional): Exception type raised inside the `with` block, if any.
            exc (optional): Exception instance raised inside the `with` block, if any.
            tb (optional): Traceback associated with the exception, if any.
        """
        self._disconnect()

    # Private helpers

    def _connect(self, *, retry_delay: float = 5.0) -> None:
        while True:
            try:
                self._device.connect()
                self.set_speed_factor(self._speed_factor)
                return
            except Exception as e:
                self._logger.warning(
                    "Connection failed: %s. Retrying in %.1f s...", e, retry_delay
                )
                self._device.close()
                time.sleep(retry_delay)

    def _disconnect(self) -> None:
        self._device.disconnect()

    def _move(
        self,
        target: Mapping[str, float] | Sequence[Any] | PoseXYZR,
        *,
        frame: str = "robot",
        speed_factor: float | None = None,
        blocking: bool = True,
    ) -> PoseXYZR:
        robot_target = self._resolve_robot_target(target, frame=frame)
        if speed_factor is not None:
            original_speed = self._speed_factor
            self.set_speed_factor(speed_factor)
        self._logger.debug("Moving to %s in %s frame", robot_target, frame)
        self._device.MovJ(*robot_target.as_tuple())
        if blocking:
            self._device.Sync()
        if speed_factor is not None:
            self.set_speed_factor(original_speed)
        return robot_target

    def _move_to_safe_height(self, *, speed_factor: float | None = None) -> PoseXYZR:
        current = self.get_pose(frame="robot")
        if current.z >= self._safe_height:
            return current
        return self._move(
            PoseXYZR(current.x, current.y, self._safe_height, current.r),
            frame="robot",
            speed_factor=speed_factor,
        )

    def _resolve_robot_target(
        self,
        target: Mapping[str, float] | Sequence[Any] | PoseXYZR,
        *,
        frame: str,
    ) -> PoseXYZR:
        default_r = self.get_pose(frame=frame).r if self._device.connected else self._home_position.r
        pose = self._to_pose_xyzr(target, default_r=default_r)
        if frame == "robot":
            robot_pose = pose
        elif frame == "work":
            robot_pose = self._work_tool_to_robot(pose)
        else:
            raise ValueError("frame must be either 'robot' or 'work'.")
        self._validate_robot_pose(robot_pose)
        return robot_pose

    def _work_tool_to_robot(self, work_tool_pose: PoseXYZR) -> PoseXYZR:
        robot_tool_pose = self._transform_work_to_robot(
            work_tool_pose, self._calibrated_offset, self._scale
        )
        return self._transform_tool_to_robot(robot_tool_pose, self._tool_offset)

    def _robot_to_work_tool(self, robot_pose: PoseXYZR) -> PoseXYZR:
        robot_tool_pose = self._transform_robot_to_tool(robot_pose, self._tool_offset)
        return self._transform_robot_to_work(robot_tool_pose, self._calibrated_offset, self._scale)

    @staticmethod
    def _to_pose_xyzr(value: Any, *, default_r: float = 0.0) -> PoseXYZR:
        if isinstance(value, PoseXYZR):
            return value

        if isinstance(value, Mapping):
            return PoseXYZR(
                float(value["x"]),
                float(value["y"]),
                float(value["z"]),
                float(value.get("r", default_r)),
            )

        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise TypeError("Pose must be a mapping, sequence, or PoseXYZR.")

        if len(value) == 4 and all(isinstance(item, (int, float)) for item in value):
            return PoseXYZR(float(value[0]), float(value[1]), float(value[2]), float(value[3]))

        if len(value) == 3 and all(isinstance(item, (int, float)) for item in value):
            return PoseXYZR(float(value[0]), float(value[1]), float(value[2]), float(default_r))

        if len(value) == 2:
            coordinates, rotation = value
            if (
                isinstance(coordinates, Sequence)
                and not isinstance(coordinates, (str, bytes))
                and len(coordinates) == 3
                and isinstance(rotation, Sequence)
                and not isinstance(rotation, (str, bytes))
                and len(rotation) >= 1
            ):
                return PoseXYZR(
                    float(coordinates[0]),
                    float(coordinates[1]),
                    float(coordinates[2]),
                    float(rotation[0]),
                )

        raise ValueError("Unsupported pose format.")

    @staticmethod
    def _rotate_xy(x: float, y: float, degrees: float) -> tuple[float, float]:
        angle = math.radians(degrees)
        cos_theta = math.cos(angle)
        sin_theta = math.sin(angle)
        return (x * cos_theta - y * sin_theta, x * sin_theta + y * cos_theta)

    @classmethod
    def _transform_work_to_robot(
        cls,
        external_pose: PoseXYZR,
        offset: PoseXYZR,
        scale: float,
    ) -> PoseXYZR:
        scaled_x = external_pose.x / scale
        scaled_y = external_pose.y / scale
        scaled_z = external_pose.z / scale
        rotated_x, rotated_y = cls._rotate_xy(scaled_x, scaled_y, -offset.r)
        return PoseXYZR(
            rotated_x - offset.x,
            rotated_y - offset.y,
            scaled_z - offset.z,
            external_pose.r - offset.r,
        )

    @classmethod
    def _transform_robot_to_work(
        cls,
        internal_pose: PoseXYZR,
        offset: PoseXYZR,
        scale: float,
    ) -> PoseXYZR:
        translated_x = offset.x + internal_pose.x
        translated_y = offset.y + internal_pose.y
        rotated_x, rotated_y = cls._rotate_xy(translated_x, translated_y, offset.r)
        return PoseXYZR(
            scale * rotated_x,
            scale * rotated_y,
            scale * (offset.z + internal_pose.z),
            offset.r + internal_pose.r,
        )

    @staticmethod
    def _transform_tool_to_robot(external_pose: PoseXYZR, offset: PoseXYZR) -> PoseXYZR:
        return PoseXYZR(
            external_pose.x - offset.x,
            external_pose.y - offset.y,
            external_pose.z - offset.z,
            external_pose.r - offset.r,
        )

    @staticmethod
    def _transform_robot_to_tool(internal_pose: PoseXYZR, offset: PoseXYZR) -> PoseXYZR:
        return PoseXYZR(
            internal_pose.x + offset.x,
            internal_pose.y + offset.y,
            internal_pose.z + offset.z,
            internal_pose.r + offset.r,
        )

    @staticmethod
    def _within_workspace(pose: PoseXYZR) -> bool:
        x, y, z, _ = pose.as_tuple()
        if not (5 <= z <= 245):
            return False
        if x >= 0:
            radius = math.sqrt(x**2 + y**2)
            return 153 <= radius <= 400
        if abs(y) < 115:
            return False
        radius = math.sqrt(x**2 + (abs(y) - 200) ** 2)
        return radius <= 200

    def _validate_robot_pose(self, pose: PoseXYZR) -> None:
        if not self._within_workspace(pose):
            raise ValueError(f"Target pose is outside the M1 Pro workspace: {pose}")

    @staticmethod
    def _parse_numeric_response(
        payload: str | None,
        *,
        minimum: int,
        take_last: int | None = None,
    ) -> list[float]:
        if payload is None:
            raise RuntimeError("Dobot returned no data.")
        values = [float(value) for value in NUMBER_PATTERN.findall(payload)]
        if len(values) < minimum:
            raise RuntimeError(f"Unable to parse Dobot response: {payload}")
        if take_last is not None:
            return values[-take_last:]
        return values

    @classmethod
    def _parse_pose_response(cls, payload: str | None) -> PoseXYZR:
        values = cls._parse_numeric_response(payload, minimum=6, take_last=6)
        return PoseXYZR(values[0], values[1], values[2], values[3])
