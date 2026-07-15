from __future__ import annotations

import logging
import unittest
from unittest.mock import patch

from dobot_driver.dobot_client import DobotDeviceClient, PoseXYZR
from dobot_driver.m1pro import M1Pro


class FakeDevice:
    connected = True

    def __init__(self, poses: list[str]) -> None:
        self._poses = iter(poses)
        self.movj_calls: list[tuple[float, float, float, float]] = []

    def GetPose(self) -> str:
        return next(self._poses)

    def MovJ(self, x: float, y: float, z: float, r: float) -> str:
        self.movj_calls.append((x, y, z, r))
        return "0,{},MovJ();"

    def Sync(self) -> str:
        return "0,{},Sync();"

    def RobotMode(self) -> str:
        return "0,{4},RobotMode();"

    def GetErrorID(self) -> str:
        return "0,{17},GetErrorID();"


class StopAfterOrientation(RuntimeError):
    pass


class OrientationProbeDevice:
    def __init__(self) -> None:
        self.orientation_calls: list[bool] = []

    def SetArmOrientation(self, right_handed: bool) -> str:
        self.orientation_calls.append(right_handed)
        raise StopAfterOrientation


class ResetProbeDevice:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def ClearError(self) -> str:
        self.calls.append("clear")
        return "0,{},ClearError();"

    def DisableRobot(self) -> str:
        self.calls.append("disable")
        return "0,{},DisableRobot();"

    def EnableRobot(self) -> str:
        self.calls.append("enable")
        return "0,{},EnableRobot();"


class GripperProbeDevice:
    def __init__(self) -> None:
        self.calls: list[tuple[int, int]] = []

    def DOExecute(self, index: int, status: int) -> str:
        self.calls.append((index, status))
        return f"0,{{}},DOExecute({index},{status});"


class VerifiedMotionTests(unittest.TestCase):
    def test_gripper_output_polarity_matches_physical_hardware(self) -> None:
        arm = M1Pro.__new__(M1Pro)
        device = GripperProbeDevice()
        arm._device = device
        arm._logger = logging.getLogger("test")

        arm.open_gripper()
        arm.close_gripper()

        self.assertEqual(device.calls, [(1, 0), (1, 1)])

    def test_safe_move_lifts_before_switching_target_orientation(self) -> None:
        arm = M1Pro.__new__(M1Pro)
        trace: list[tuple[str, object]] = []

        class TraceDevice:
            def SetArmOrientation(self, right_handed: bool) -> str:
                trace.append(("orientation", right_handed))
                return "0,{},SetArmOrientation();"

        arm._device = TraceDevice()
        arm._safe_height = 240.0
        arm._resolve_robot_target = lambda target, frame="robot": PoseXYZR(57, 275, 21, -20)
        arm.get_pose = lambda frame="robot": PoseXYZR(131, -231, 14, -20)

        def record_move(target, **kwargs):
            trace.append(("move", target))
            return target

        arm._move = record_move
        arm.safe_move({"x": 57, "y": 275, "z": 21, "r": -20})

        self.assertEqual(
            trace,
            [
                ("orientation", False),
                ("move", PoseXYZR(131, -231, 240, -20)),
                ("orientation", True),
                ("move", PoseXYZR(57, 275, 240, -20)),
                ("move", PoseXYZR(57, 275, 21, -20)),
            ],
        )

    def test_safe_move_defaults_all_segments_to_075_speed_factor(self) -> None:
        arm = M1Pro.__new__(M1Pro)
        speed_factors: list[float | None] = []

        class TraceDevice:
            def SetArmOrientation(self, right_handed: bool) -> str:
                return "0,{},SetArmOrientation();"

        arm._device = TraceDevice()  # type: ignore[assignment]
        arm._safe_height = 240.0
        arm._resolve_robot_target = lambda target, frame="robot": PoseXYZR(131, -231, 14, -20)
        arm.get_pose = lambda frame="robot": PoseXYZR(200, 0, 200, 20)

        def record_move(target, **kwargs):
            speed_factors.append(kwargs.get("speed_factor"))
            return target

        arm._move = record_move
        arm.safe_move({"x": 131, "y": -231, "z": 14, "r": -20})

        self.assertEqual(speed_factors, [0.75, 0.75, 0.75])

    def test_negative_y_safe_move_selects_left_handed_orientation(self) -> None:
        arm = M1Pro.__new__(M1Pro)
        device = OrientationProbeDevice()
        arm._device = device
        arm._safe_height = 240.0
        arm._resolve_robot_target = lambda target, frame="robot": PoseXYZR(131, -231, 240, -20)
        arm.get_pose = lambda frame="robot": PoseXYZR(200, 0, 240, 20)

        with self.assertRaises(StopAfterOrientation):
            arm.safe_move({"x": 131, "y": -231, "z": 240, "r": -20})

        self.assertEqual(device.orientation_calls, [False])

    def test_reset_clears_controller_error_before_disabling_robot(self) -> None:
        client = DobotDeviceClient.__new__(DobotDeviceClient)
        probe = ResetProbeDevice()
        client.ClearError = probe.ClearError
        client.DisableRobot = probe.DisableRobot
        client.EnableRobot = probe.EnableRobot

        with patch("time.sleep") as sleep:
            client.reset()

        self.assertEqual(probe.calls, ["clear", "disable", "enable"])
        sleep.assert_called_once_with(0.5)

    def test_blocking_move_raises_when_measured_pose_does_not_reach_target(self) -> None:
        arm = M1Pro.__new__(M1Pro)
        arm._device = FakeDevice(
            [
                "0,{200,0,240,20,0,0},GetPose();",
                "0,{200,0,240,20,0,0},GetPose();",
            ]
        )
        arm._speed_factor = 0.2
        arm._logger = logging.getLogger("test")

        with self.assertRaisesRegex(RuntimeError, "did not reach target") as raised:
            arm._move(PoseXYZR(131, -231, 240, -20), blocking=True)

        message = str(raised.exception)
        self.assertIn("movj_response='0,{},MovJ();'", message)
        self.assertIn("sync_response='0,{},Sync();'", message)
        self.assertIn("robot_mode='0,{4},RobotMode();'", message)
        self.assertIn("error_ids='0,{17},GetErrorID();'", message)


if __name__ == "__main__":
    unittest.main()
