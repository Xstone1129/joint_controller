#!/usr/bin/env python3
"""Offline tests for the standalone EtherCAT console configuration and ABI."""

from __future__ import annotations

import ctypes
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import ethercat_hardware_test as console  # noqa: E402


class EthercatHardwareConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = console.load_config(console.CONFIG_PATH)

    def test_heavy_v1_master_mapping(self) -> None:
        ethercat = console.mapping(self.config)
        self.assertEqual(ethercat["masters"]["left_arm"]["index"], 0)
        self.assertEqual(ethercat["masters"]["left_arm"]["interface"], "enp3s0")
        self.assertEqual(ethercat["masters"]["right_arm"]["index"], 1)
        self.assertEqual(ethercat["masters"]["right_arm"]["interface"], "enp4s0")
        self.assertEqual(ethercat["masters"]["lift"]["index"], 2)
        self.assertEqual(ethercat["masters"]["lift"]["interface"], "enp5s0")

    def test_shared_memory_abi_matches_igh_driver(self) -> None:
        self.assertEqual(ctypes.sizeof(console.AxisCommand), 20)
        self.assertEqual(ctypes.sizeof(console.AxisFeedback), 28)
        self.assertEqual(ctypes.sizeof(console.DesireRegion), 480)
        self.assertEqual(ctypes.sizeof(console.RealRegion), 648)

    def test_arm_slots_cover_both_arms(self) -> None:
        slots = self.config["arm"]["axis_slots"]
        self.assertEqual(list(slots["left_arm"]), list(range(7)))
        self.assertEqual(list(slots["right_arm"]), list(range(7, 14)))

    def test_arm_position_conversion_uses_radians(self) -> None:
        units_per_rad = console.arm_units_per_rad(self.config)
        self.assertEqual(console.arm_rad_to_units(1.0, units_per_rad), 83443)
        self.assertEqual(console.arm_rad_to_units(-0.1, units_per_rad), -8344)
        self.assertEqual(self.config["arm"]["jog_step_rad"], 0.005)
        self.assertEqual(self.config["arm"]["max_speed_rad_s"], 0.05)
        self.assertEqual(self.config["arm"]["limit_stall_timeout_s"], 0.5)

    def test_arm_logical_position_applies_zero_offset(self) -> None:
        self.assertAlmostEqual(console.arm_logical_position(100, 40, 20.0), 3.0)

    def test_arm_feedback_requires_powerstate_and_axis_feedback(self) -> None:
        real = console.RealRegion()
        slots = list(range(14))
        self.assertFalse(console.arm_feedback_ready(real, slots, 0x08))
        real.ec_powerstate = 1
        for slot in slots:
            real.axis_state[slot].ec_modestate = 0x08
            real.axis_state[slot].ec_ctrstate = 0x21
        self.assertTrue(console.arm_feedback_ready(real, slots, 0x08))

    def test_arm_position_conversion_rejects_non_finite_values(self) -> None:
        with self.assertRaises(ValueError):
            console.arm_rad_to_units(float("nan"), 83443.0)

    def test_driver_log_uses_active_runtime_root(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            runtime_root = Path(directory) / "runtime"
            with mock.patch.dict(
                os.environ,
                {"JOINT_CONTROLLER_LOG_ROOT": str(runtime_root)},
                clear=False,
            ):
                path = console.allocate_driver_log("/tmp/heavy_v1_igh_driver.log")
            self.assertEqual(path.parent, runtime_root)
            self.assertTrue(path.name.startswith("heavy_v1_igh_driver-"))
            self.assertEqual(
                (runtime_root / "heavy_v1_igh_driver.latest.log").resolve(), path
            )

    def test_driver_log_fallback_uses_marker_session(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "ros" / "log"
            marker = Path(directory) / ".junior_runtime_session_name"
            marker.write_text("2026-09-08-21-52-14-junior-runtime\n", encoding="utf-8")
            with mock.patch.dict(
                os.environ,
                {
                    "JUNIOR_LOG_ROOT": str(root),
                    "JUNIOR_RUNTIME_LOG_USER": "user",
                    "JOINT_CONTROLLER_SESSION_NAME_FILE": str(marker),
                },
                clear=False,
            ):
                os.environ.pop("JOINT_CONTROLLER_LOG_ROOT", None)
                path = console.allocate_driver_log("/tmp/heavy_v1_igh_driver.log")
            self.assertEqual(
                path.parent,
                root / "2026-09-08-21-52-14-junior-runtime" / "runtime",
            )

    def test_driver_log_dynamic_server_refreshes_marker(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "ros" / "log"
            marker = Path(directory) / ".junior_runtime_session_name"
            marker.write_text("new-runtime\n", encoding="utf-8")
            with mock.patch.dict(
                os.environ,
                {
                    "JUNIOR_LOG_ROOT": str(root),
                    "JOINT_CONTROLLER_LOG_ROOT": str(root / "old-runtime" / "runtime"),
                    "JUNIOR_RUNTIME_LOG_DYNAMIC": "1",
                    "JUNIOR_RUNTIME_LOG_USER": "user",
                    "JOINT_CONTROLLER_SESSION_NAME_FILE": str(marker),
                },
                clear=False,
            ):
                path = console.allocate_driver_log("heavy_v1_igh_driver.log")
            self.assertEqual(path.parent, root / "new-runtime" / "runtime")

    def test_lift_binding_uses_standalone_cli(self) -> None:
        lift = self.config["lift"]
        self.assertEqual(lift["cli_binary"], "../lift_ethercat_cli")
        self.assertEqual(lift["master"], 2)
        self.assertEqual(lift["interface"], "enp5s0")
        self.assertEqual(lift["slave_alias"], 0)
        self.assertEqual(lift["slave_position"], 0)
        self.assertEqual(lift["min_position_m"], -1.0)
        self.assertEqual(lift["max_position_m"], 0.0)

    def test_etherlab_values_are_replaced_without_duplicate_keys(self) -> None:
        text = 'MASTER0_DEVICE="old0"\nMASTER1_DEVICE="old1"\n'
        updated = console.set_config_value(text, "MASTER0_DEVICE", "enp3s0")
        self.assertIn('MASTER0_DEVICE="enp3s0"', updated)
        self.assertEqual(updated.count("MASTER0_DEVICE="), 1)

    def test_ethercat_slave_counts_parses_multiline_master_output(self) -> None:
        output = """Master0\n  Phase: Idle\n  Slaves: 7\nMaster1\n  Slaves: 7\nMaster2\n  Slaves: 1\n"""
        completed = console.subprocess.CompletedProcess(
            ["ethercat", "master"], 0, output, ""
        )
        with mock.patch.object(console, "run", return_value=completed):
            self.assertEqual(console.ethercat_slave_counts(), {0: 7, 1: 7, 2: 1})


if __name__ == "__main__":
    unittest.main()
