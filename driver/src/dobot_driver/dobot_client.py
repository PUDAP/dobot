from __future__ import annotations

from dataclasses import dataclass
import logging
import re
from typing import Any

from .dobot_api import (
    DobotApiDashboard,
    DobotApiMove,
)

logger = logging.getLogger(__name__)

DASHBOARD_PORT = 29999
MOVE_PORT = 30003

NUMBER_PATTERN = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?")


@dataclass(frozen=True)
class CartesianPose:
    x: float
    y: float
    z: float
    r: float = 0.0

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.r)

    def with_z(self, z: float) -> "CartesianPose":
        return CartesianPose(self.x, self.y, z, self.r)


class DobotDeviceClient:
    """Thin device wrapper based on Dobot's TCP/IP dashboard and move APIs."""

    def __init__(
        self,
        host: str,
        *,
        timeout: float = 10.0,
        simulation: bool = False,
    ) -> None:
        self.host = host
        self.timeout = timeout
        self.simulation = simulation
        self.dashboard_api: DobotApiDashboard | None = None
        self.move_api: DobotApiMove | None = None
        self.connected = False
        self._sim_pose = CartesianPose(300.0, 0.0, 240.0, -33.0)
        self._logger = logger.getChild(self.__class__.__name__)

    def connect(self) -> None:
        if self.connected:
            return
        if self.simulation:
            self.connected = True
            return
        self.dashboard_api = DobotApiDashboard(self.host, DASHBOARD_PORT)
        self.move_api = DobotApiMove(self.host, MOVE_PORT)
        self.reset()
        if self.dashboard_api is not None:
            self.dashboard_api.User(0)
            self.dashboard_api.Tool(0)
        self.connected = True

    def disconnect(self) -> None:
        if not self.connected:
            return
        try:
            self.ResetRobot()
            self.DisableRobot()
        finally:
            self.close()
            self.connected = False

    def close(self) -> None:
        if self.dashboard_api is not None:
            self.dashboard_api.close()
            self.dashboard_api = None
        if self.move_api is not None:
            self.move_api.close()
            self.move_api = None

    def set_sim_pose(self, pose: CartesianPose) -> None:
        self._sim_pose = pose

    def reset(self) -> None:
        self.DisableRobot()
        self.ClearError()
        self.EnableRobot()

    def ClearError(self) -> str | None:
        if self.simulation:
            return "simulation:ClearError"
        return self.dashboard_api.ClearError() if self.dashboard_api is not None else None

    def DisableRobot(self) -> str | None:
        if self.simulation:
            return "simulation:DisableRobot"
        return self.dashboard_api.DisableRobot() if self.dashboard_api is not None else None

    def EnableRobot(self, *args: Any) -> str | None:
        if self.simulation:
            return "simulation:EnableRobot"
        return self.dashboard_api.EnableRobot(*args) if self.dashboard_api is not None else None

    def ResetRobot(self) -> str | None:
        if self.simulation:
            return "simulation:ResetRobot"
        return self.dashboard_api.ResetRobot() if self.dashboard_api is not None else None

    def SpeedFactor(self, speed_factor: int) -> str | None:
        if self.simulation:
            return f"simulation:SpeedFactor({speed_factor})"
        return self.dashboard_api.SpeedFactor(speed_factor) if self.dashboard_api is not None else None

    def GetAngle(self) -> str | None:
        if self.simulation:
            return "{0,0,0,0,0,0}"
        return self.dashboard_api.GetAngle() if self.dashboard_api is not None else None

    def GetPose(self) -> str | None:
        if self.simulation:
            x, y, z, r = self._sim_pose.as_tuple()
            return f"{{{x},{y},{z},{r},0,0}}"
        return self.dashboard_api.GetPose() if self.dashboard_api is not None else None

    def SetArmOrientation(self, right_handed: bool) -> str | None:
        if self.simulation:
            return f"simulation:SetArmOrientation({int(right_handed)})"
        if self.dashboard_api is None:
            return None
        return self.dashboard_api.SetArmOrientation(int(right_handed))

    def MovJ(self, x: float, y: float, z: float, r: float, *args: Any) -> str | None:
        if self.simulation:
            self._sim_pose = CartesianPose(x, y, z, r)
            return f"simulation:MovJ({x},{y},{z},{r})"
        return self.move_api.MovJ(x, y, z, r, *args) if self.move_api is not None else None

    def Sync(self) -> str | None:
        if self.simulation:
            return "simulation:Sync"
        return self.move_api.Sync() if self.move_api is not None else None

    def DOExecute(self, index: int, status: int) -> str | None:
        if self.simulation:
            return f"simulation:DOExecute({index},{status})"
        if self.dashboard_api is None:
            raise RuntimeError("dashboard_api is not connected")
        return self.dashboard_api.DOExecute(index, status)
