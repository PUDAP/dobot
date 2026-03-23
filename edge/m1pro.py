from __future__ import annotations

import logging
import math
from typing import Any, Mapping, Sequence

from dobot_client import (
    CartesianPose,
    DobotDeviceClient,
    NUMBER_PATTERN,
)

logger = logging.getLogger(__name__)

DEFAULT_HOME_POSITION = [[300, 0, 200], [-33, 0, 0]]
DEFAULT_CALIBRATED_OFFSET = [[-374, 496.75, 254.2], [-89.11611149, 0, 0]]
DEFAULT_TOOL_OFFSET = [[0, 0, -232], [122.11611149, 0, 0]]
DEFAULT_SAVED_POSITIONS = {
    "zone1_jar_tray_1_center": [[88.7, -29, 60], [57, 0, 0]],
    "zone1_jar_tray_2_center": [[88.7, 122, 60], [57, 0, 0]],
    "zone1_jar_tray_4_center": [[-13.4, 122, 60], [57, 0, 0]],
    "zone2_jar_tray_center": [[636.4, 246.7, 60], [-33, 0, 0]],
}


class M1ProArm:
    """Standalone M1 Pro controller using Dobot's TCP/IP API directly."""

    def __init__(
        self,
        dobot_ip: str,
        *,
        home_position: Any = DEFAULT_HOME_POSITION,
        calibrated_offset: Any = DEFAULT_CALIBRATED_OFFSET,
        scale: float = 1.0,
        tool_offset: Any = DEFAULT_TOOL_OFFSET,
        safe_height: float = 240.0,
        saved_positions: Mapping[str, Any] | None = None,
        home_waypoints: Sequence[Any] | None = None,
        timeout: float = 10.0,
        verbose: bool = True,
        simulation: bool = False,
        auto_connect: bool = True,
    ) -> None:
        self.host = dobot_ip
        self.verbose = verbose
        self.simulation = simulation
        self.scale = float(scale)
        self.safe_height = float(safe_height)
        self.home_position = self._coerce_pose(home_position, default_r=-33.0)
        self.calibrated_offset = self._coerce_pose(calibrated_offset)
        self.tool_offset = self._coerce_pose(tool_offset)
        self.saved_positions = dict(DEFAULT_SAVED_POSITIONS if saved_positions is None else saved_positions)
        self.home_waypoints = [
            self._coerce_pose(waypoint, default_r=self.home_position.r) for waypoint in (home_waypoints or [])
        ]
        self._logger = logger.getChild(self.__class__.__name__)
        if self.verbose:
            self._logger.setLevel(logging.DEBUG)
        self.device = DobotDeviceClient(
            dobot_ip,
            timeout=timeout,
            simulation=simulation,
            verbose=verbose,
        )
        self.device.set_sim_pose(self.home_position)
        self._speed_factor = 0.2
        self._validate_robot_pose(self.home_position)
        if auto_connect:
            self._connect()

    def _connect(self) -> None:
        self.device.connect()
        self.set_speed_factor(self._speed_factor)

    def _disconnect(self) -> None:
        self.device.disconnect()

    def open_gripper(self) -> str | None:
        """Open the gripper (DOExecute index=1, status=0)."""
        self._logger.debug("Opening gripper")
        return self.device.DOExecute(1, 1)

    def close_gripper(self) -> str | None:
        """Close the gripper (DOExecute index=1, status=1)."""
        self._logger.debug("Closing gripper")
        return self.device.DOExecute(1, 0)

    def reset(self) -> None:
        self.device.reset()

    def set_speed_factor(self, speed_factor: float) -> None:
        if not 0.0 < speed_factor <= 1.0:
            raise ValueError("speed_factor must be in the range (0.0, 1.0].")
        self._speed_factor = float(speed_factor)
        self.device.SpeedFactor(max(1, min(100, int(round(speed_factor * 100)))))

    def get_pose(self, *, frame: str = "robot") -> CartesianPose:
        robot_pose = self._parse_pose_response(self.device.GetPose())
        if frame == "robot":
            return robot_pose
        if frame == "work":
            return self._robot_to_work_tool(robot_pose)
        raise ValueError("frame must be either 'robot' or 'work'.")

    def get_joint_angles(self) -> tuple[float, float, float, float, float, float]:
        values = self._parse_numeric_response(self.device.GetAngle(), minimum=6, take_last=6)
        return tuple(values[-6:])  # type: ignore[return-value]

    def preview_target(
        self,
        target: str | Mapping[str, float] | Sequence[Any] | CartesianPose,
        *,
        frame: str = "robot",
    ) -> dict[str, Any]:
        """Preview how an input target resolves into raw robot coordinates."""
        resolved_target = self.saved_positions[target] if isinstance(target, str) else target
        input_pose = self._coerce_pose(
            resolved_target,
            default_r=self.get_pose(frame=frame).r if self.device.connected else self.home_position.r,
        )
        robot_pose = input_pose if frame == "robot" else self._work_tool_to_robot(input_pose)
        return {
            "frame": frame,
            "input_pose": input_pose,
            "robot_pose": robot_pose,
            "within_workspace": self._within_workspace(robot_pose),
        }

    def _move(
        self,
        target: str | Mapping[str, float] | Sequence[Any] | CartesianPose,
        *,
        frame: str = "robot",
        speed_factor: float | None = None,
        blocking: bool = True,
    ) -> CartesianPose:
        robot_target = self._resolve_robot_target(target, frame=frame)
        if speed_factor is not None:
            self.set_speed_factor(speed_factor)
        self._logger.debug("Moving to %s in %s frame", robot_target, frame)
        self.device.MovJ(*robot_target.as_tuple())
        if blocking:
            self.device.Sync()
        return robot_target

    def safe_move(
        self,
        target: str | Mapping[str, float] | Sequence[Any] | CartesianPose,
        *,
        frame: str = "robot",
        speed_factor_lateral: float | None = None,
        speed_factor_up: float | None = None,
        speed_factor_down: float | None = None,
        blocking: bool = True,
    ) -> CartesianPose:
        robot_target = self._resolve_robot_target(target, frame=frame)
        current = self.get_pose(frame="robot")
        travel_z = max(self.safe_height, current.z, robot_target.z)

        if current.z < travel_z:
            self._move(
                CartesianPose(current.x, current.y, travel_z, current.r),
                frame="robot",
                speed_factor=speed_factor_up,
                blocking=blocking,
            )

        self._move(
            CartesianPose(robot_target.x, robot_target.y, travel_z, robot_target.r),
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

    def move_to_safe_height(self, *, speed_factor: float | None = None) -> CartesianPose:
        current = self.get_pose(frame="robot")
        if current.z >= self.safe_height:
            return current
        return self._move(
            CartesianPose(current.x, current.y, self.safe_height, current.r),
            frame="robot",
            speed_factor=speed_factor,
        )

    def home(self, *, speed_factor: float | None = None) -> CartesianPose:
        self.move_to_safe_height(speed_factor=speed_factor)
        for waypoint in self.home_waypoints:
            self._move(waypoint, frame="robot", speed_factor=speed_factor)
        return self._move(self.home_position, frame="robot", speed_factor=speed_factor)

    def __enter__(self) -> "M1ProArm":
        self._connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self._disconnect()

    def _resolve_robot_target(
        self,
        target: str | Mapping[str, float] | Sequence[Any] | CartesianPose,
        *,
        frame: str,
    ) -> CartesianPose:
        target_name = target if isinstance(target, str) else None
        if isinstance(target, str):
            if target not in self.saved_positions:
                raise KeyError(f"Unknown saved position: {target}")
            target = self.saved_positions[target]

        default_r = self.get_pose(frame=frame).r if self.device.connected else self.home_position.r
        pose = self._coerce_pose(target, default_r=default_r)
        if frame == "robot":
            robot_pose = pose
        elif frame == "work":
            robot_pose = self._work_tool_to_robot(pose)
        else:
            raise ValueError("frame must be either 'robot' or 'work'.")
        try:
            self._validate_robot_pose(robot_pose)
        except ValueError as exc:
            if target_name is not None and frame == "robot":
                raise ValueError(
                    f"{exc} Named saved positions like '{target_name}' are usually configured in work coordinates. "
                    "Use frame='work' unless the saved value is already a raw robot pose."
                ) from exc
            raise
        return robot_pose

    def _work_tool_to_robot(self, work_tool_pose: CartesianPose) -> CartesianPose:
        robot_tool_pose = self._transform_work_to_robot(work_tool_pose, self.calibrated_offset, self.scale)
        return self._transform_tool_to_robot(robot_tool_pose, self.tool_offset)

    def _robot_to_work_tool(self, robot_pose: CartesianPose) -> CartesianPose:
        robot_tool_pose = self._transform_robot_to_tool(robot_pose, self.tool_offset)
        return self._transform_robot_to_work(robot_tool_pose, self.calibrated_offset, self.scale)

    @staticmethod
    def _coerce_pose(value: Any, *, default_r: float = 0.0) -> CartesianPose:
        if isinstance(value, CartesianPose):
            return value

        if isinstance(value, Mapping):
            return CartesianPose(
                float(value["x"]),
                float(value["y"]),
                float(value["z"]),
                float(value.get("r", default_r)),
            )

        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise TypeError("Pose must be a mapping, sequence, or CartesianPose.")

        if len(value) == 4 and all(isinstance(item, (int, float)) for item in value):
            return CartesianPose(float(value[0]), float(value[1]), float(value[2]), float(value[3]))

        if len(value) == 3 and all(isinstance(item, (int, float)) for item in value):
            return CartesianPose(float(value[0]), float(value[1]), float(value[2]), float(default_r))

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
                return CartesianPose(
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
        external_pose: CartesianPose,
        offset: CartesianPose,
        scale: float,
    ) -> CartesianPose:
        scaled_x = external_pose.x / scale
        scaled_y = external_pose.y / scale
        scaled_z = external_pose.z / scale
        rotated_x, rotated_y = cls._rotate_xy(scaled_x, scaled_y, -offset.r)
        return CartesianPose(
            rotated_x - offset.x,
            rotated_y - offset.y,
            scaled_z - offset.z,
            external_pose.r - offset.r,
        )

    @classmethod
    def _transform_robot_to_work(
        cls,
        internal_pose: CartesianPose,
        offset: CartesianPose,
        scale: float,
    ) -> CartesianPose:
        translated_x = offset.x + internal_pose.x
        translated_y = offset.y + internal_pose.y
        rotated_x, rotated_y = cls._rotate_xy(translated_x, translated_y, offset.r)
        return CartesianPose(
            scale * rotated_x,
            scale * rotated_y,
            scale * (offset.z + internal_pose.z),
            offset.r + internal_pose.r,
        )

    @staticmethod
    def _transform_tool_to_robot(external_pose: CartesianPose, offset: CartesianPose) -> CartesianPose:
        return CartesianPose(
            external_pose.x - offset.x,
            external_pose.y - offset.y,
            external_pose.z - offset.z,
            external_pose.r - offset.r,
        )

    @staticmethod
    def _transform_robot_to_tool(internal_pose: CartesianPose, offset: CartesianPose) -> CartesianPose:
        return CartesianPose(
            internal_pose.x + offset.x,
            internal_pose.y + offset.y,
            internal_pose.z + offset.z,
            internal_pose.r + offset.r,
        )

    @staticmethod
    def _within_workspace(pose: CartesianPose) -> bool:
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

    def _validate_robot_pose(self, pose: CartesianPose) -> None:
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
    def _parse_pose_response(cls, payload: str | None) -> CartesianPose:
        values = cls._parse_numeric_response(payload, minimum=6, take_last=6)
        return CartesianPose(values[0], values[1], values[2], values[3])


class M1ProTemp(M1ProArm):
    """Backward-compatible alias for the old edge wrapper name."""
