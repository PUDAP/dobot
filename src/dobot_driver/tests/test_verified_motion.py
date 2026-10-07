from __future__ import annotations

import logging
import threading
import unittest
from unittest.mock import call, patch

from dobot_driver.dobot_api import DobotApiDashboard
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

    def test_set_arm_orientation_accepts_one_and_sends_right_handed(self) -> None:
        arm = M1Pro.__new__(M1Pro)

        class OrientationDevice:
            def __init__(self) -> None:
                self.calls: list[bool] = []

            def SetArmOrientation(self, right_handed: bool) -> str:
                self.calls.append(right_handed)
                return "0,{},SetArmOrientation();"

        device = OrientationDevice()
        arm._device = device

        result = arm.set_arm_orientation(1)

        self.assertEqual(result, "0,{},SetArmOrientation();")
        self.assertEqual(device.calls, [True])

    def test_set_arm_orientation_rejects_values_other_than_zero_or_one(self) -> None:
        arm = M1Pro.__new__(M1Pro)

        with self.assertRaisesRegex(ValueError, "orientation must be 0 or 1"):
            arm.set_arm_orientation(2)

    def test_rotate_r_preserves_xyz_and_issues_one_direct_move(self) -> None:
        arm = M1Pro.__new__(M1Pro)
        current = PoseXYZR(240.5, -77.5, 25.0, -180.0)
        moves: list[tuple[PoseXYZR, dict[str, object]]] = []
        arm.get_pose = lambda frame="robot": current

        def record_move(target, **kwargs):
            moves.append((target, kwargs))
            return target

        arm._move = record_move

        result = arm.rotate_r(r=-170.0, speed_factor=0.25)

        self.assertEqual(result, PoseXYZR(240.5, -77.5, 25.0, -170.0))
        self.assertEqual(
            moves,
            [
                (
                    PoseXYZR(240.5, -77.5, 25.0, -170.0),
                    {"frame": "robot", "speed_factor": 0.25, "blocking": True},
                )
            ],
        )

    def test_move_vertical_to_preserves_xyr_and_issues_one_direct_move(self) -> None:
        arm = M1Pro.__new__(M1Pro)
        current = PoseXYZR(127.0, -330.5, 10.0, -22.5)
        moves: list[tuple[PoseXYZR, dict[str, object]]] = []
        arm.get_pose = lambda frame="robot": current

        def record_move(target, **kwargs):
            moves.append((target, kwargs))
            return target

        arm._move = record_move

        result = arm.move_vertical_to(z=30.0, speed_factor=0.25)

        self.assertEqual(result, PoseXYZR(127.0, -330.5, 30.0, -22.5))
        self.assertEqual(
            moves,
            [
                (
                    PoseXYZR(127.0, -330.5, 30.0, -22.5),
                    {"frame": "robot", "speed_factor": 0.25, "blocking": True},
                )
            ],
        )

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

    def test_safe_move_defaults_to_075_lateral_and_025_vertical_speed_factors(self) -> None:
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

        self.assertEqual(speed_factors, [0.25, 0.75, 0.25])

    def test_pick_from_uses_safe_move_result_without_an_extra_speed_override(self) -> None:
        arm = M1Pro.__new__(M1Pro)
        safe_pose = PoseXYZR(131, -231, 14, -20)
        gripper_calls: list[str] = []
        arm.safe_move = lambda position, frame="robot": safe_pose  # type: ignore[method-assign]
        arm.open_gripper = lambda: gripper_calls.append("open")  # type: ignore[method-assign]
        arm.close_gripper = lambda: gripper_calls.append("close")  # type: ignore[method-assign]

        def unexpected_move(*args, **kwargs):
            raise AssertionError("pick_from must not add a move outside safe_move")

        arm._move = unexpected_move
        result = arm.pick_from(position={"x": 131, "y": -231, "z": 14, "r": -20})

        self.assertEqual(result, safe_pose)
        self.assertEqual(gripper_calls, ["open", "close"])

    def test_place_to_uses_safe_move_result_without_an_extra_speed_override(self) -> None:
        arm = M1Pro.__new__(M1Pro)
        safe_pose = PoseXYZR(57, 275, 21, -20)
        gripper_calls: list[str] = []
        arm.safe_move = lambda position, frame="robot": safe_pose  # type: ignore[method-assign]
        arm.open_gripper = lambda: gripper_calls.append("open")  # type: ignore[method-assign]

        def unexpected_move(*args, **kwargs):
            raise AssertionError("place_to must not add a move outside safe_move")

        arm._move = unexpected_move
        result = arm.place_to(position={"x": 57, "y": 275, "z": 21, "r": -20})

        self.assertEqual(result, safe_pose)
        self.assertEqual(gripper_calls, ["open"])

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

    def test_pick_and_place_uses_approved_composite_sequence_and_speeds(self) -> None:
        arm = M1Pro.__new__(M1Pro)
        arm._safe_height = 240.0
        trace: list[tuple[str, object]] = []

        class TraceDevice:
            connected = True

            def SetArmOrientation(self, right_handed: bool) -> str:
                trace.append(("orientation", right_handed))
                return "0,{},SetArmOrientation();"

        arm._device = TraceDevice()  # type: ignore[assignment]
        poses = iter([
            PoseXYZR(200, 0, 100, -22.5),
            PoseXYZR(131.33, -211.3333333, 109, -22.5),
        ])
        arm.get_pose = lambda frame="robot": next(poses)
        arm.close_gripper = lambda: (trace.append(("gripper", "close")) or "ok")  # type: ignore[method-assign]
        arm.open_gripper = lambda: (trace.append(("gripper", "open")) or "ok")  # type: ignore[method-assign]

        def record_move(target, **kwargs):
            trace.append(("move", (target, kwargs.get("speed_factor"))))
            return target

        arm._move = record_move
        source = {"x": 213.332206541, "y": -90.224425275, "z": 27, "r": -22.5}
        destination = {"x": 131.33, "y": -211.3333333, "z": 9, "r": -22.5}

        with patch("dobot_driver.m1pro.time.sleep") as sleep:
            result = arm.pick_and_place(source=source, destination=destination)

        self.assertEqual(result, PoseXYZR(131.33, -211.3333333, 109, -22.5))
        self.assertEqual(sleep.call_args_list, [call(1.0)] * 3)
        self.assertEqual(
            trace,
            [
                ("move", (PoseXYZR(200, 0, 240, -22.5), 0.50)),
                ("orientation", False),
                ("move", (PoseXYZR(213.332206541, -90.224425275, 240, -22.5), 0.90)),
                ("move", (PoseXYZR(213.332206541, -90.224425275, 127, -22.5), 0.50)),
                ("move", (PoseXYZR(213.332206541, -90.224425275, 27, -22.5), 0.05)),
                ("gripper", "close"),
                ("move", (PoseXYZR(213.332206541, -90.224425275, 127, -22.5), 0.05)),
                ("move", (PoseXYZR(213.332206541, -90.224425275, 240, -22.5), 0.50)),
                ("orientation", False),
                ("move", (PoseXYZR(131.33, -211.3333333, 240, -22.5), 0.90)),
                ("move", (PoseXYZR(131.33, -211.3333333, 109, -22.5), 0.50)),
                ("move", (PoseXYZR(131.33, -211.3333333, 9, -22.5), 0.05)),
                ("gripper", "open"),
                ("move", (PoseXYZR(131.33, -211.3333333, 109, -22.5), 0.05)),
            ],
        )

    def test_clear_error_accepts_minus_one_when_controller_reports_no_existing_error(self) -> None:
        dashboard = DobotApiDashboard.__new__(DobotApiDashboard)
        dashboard.ip = "192.0.2.1"
        dashboard.port = 29999
        dashboard.socket_dobot = 0
        object.__setattr__(dashboard, "_DobotApi__globalLock", threading.Lock())
        with (
            patch.object(dashboard, "send_data"),
            patch.object(dashboard, "wait_reply", return_value="-1,{},ClearError();"),
        ):
            self.assertEqual(dashboard.ClearError(), "-1,{},ClearError();")

    def test_clear_error_rejects_other_nonzero_error_codes(self) -> None:
        dashboard = DobotApiDashboard.__new__(DobotApiDashboard)
        dashboard.ip = "192.0.2.1"
        dashboard.port = 29999
        dashboard.socket_dobot = 0
        object.__setattr__(dashboard, "_DobotApi__globalLock", threading.Lock())
        with (
            patch.object(dashboard, "send_data"),
            patch.object(dashboard, "wait_reply", return_value="-2,{},ClearError();"),
        ):
            with self.assertRaisesRegex(RuntimeError, "rejected 'ClearError\\(\\)'"):
                dashboard.ClearError()

    def test_other_dashboard_commands_still_reject_minus_one(self) -> None:
        dashboard = DobotApiDashboard.__new__(DobotApiDashboard)
        dashboard.ip = "192.0.2.1"
        dashboard.port = 29999
        dashboard.socket_dobot = 0
        object.__setattr__(dashboard, "_DobotApi__globalLock", threading.Lock())
        with (
            patch.object(dashboard, "send_data"),
            patch.object(dashboard, "wait_reply", return_value="-1,{},DisableRobot();"),
        ):
            with self.assertRaisesRegex(RuntimeError, "rejected 'DisableRobot\\(\\)'"):
                dashboard.DisableRobot()

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
