#!/usr/bin/env python3
"""Offline tests for the standalone EtherCAT console configuration and ABI."""

from __future__ import annotations

import ctypes
import os
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
import sys
import tempfile
import time
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

    def test_tool_travel_bounds_match_the_runtime_lift_config(self) -> None:
        """The console's travel bounds must equal the runtime's.

        They are duplicated on purpose: this console is ROS-free and reads its own
        hardware_io.yaml, so it cannot import the runtime parameters.  They had
        already drifted once -- hardware_io.yaml allowed max_position_m 1.0, one
        metre above the top of travel, which lets the console drive the carriage
        into the mechanical stop (the LD3M then latches a protective state that
        only a drive power cycle clears).  Compare them so the next divergence
        fails here instead of on the machine.
        """
        import yaml

        root = Path(__file__).resolve().parents[2]
        tool = yaml.safe_load((Path(__file__).resolve().parent / "hardware_io.yaml").read_text(
            encoding="utf-8"))["lift"]
        runtime = yaml.safe_load((
            root / "src" / "joint_hardware" / "config" / "lift_hardware.yaml").read_text(
                encoding="utf-8"))
        self.assertEqual(tool["min_position_m"], runtime["position_min_m"])
        self.assertEqual(tool["max_position_m"], runtime["position_max_m"])

    def test_error_name_tables_do_not_drift(self) -> None:
        """The lift console and this console must name drive codes alike.

        Both keep their own error-code table: they are separate ROS-free tools and
        cannot share a module cleanly.  The two tables agree today, so this is a
        latent drift hazard rather than a current defect -- compare them so that a
        change to one is caught here instead of quietly making the two consoles
        describe the same drive fault with different words.
        """
        import importlib.util

        here = Path(__file__).resolve().parent
        spec = importlib.util.spec_from_file_location(
            "lift_console_error_names", here / "lift_console.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertTrue(module.LIFT_ERROR_NAMES)
        self.assertEqual(module.LIFT_ERROR_NAMES, console.AXIS_ERROR_NAMES)

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


class _FakeAxis:
    def __init__(self, state: int, position: int, error: int) -> None:
        self.ec_ctrstate = state
        self.ec_modestate = 0x08
        self.axis_position = position
        self.axis_velocity = 0
        self.axis_error_code = error


class _FakeReal:
    def __init__(self, axes: list[_FakeAxis]) -> None:
        self.ec_powerstate = 1
        self.axis_state = axes


class _FakeDesire:
    ec_poweron = 1


class ArmPageTest(unittest.TestCase):
    """The arm pages must match the upper remote console page for page."""

    UNITS_PER_RAD = 83443.02680376362

    def make_real(self) -> _FakeReal:
        return _FakeReal(
            [
                _FakeAxis(0x27, 83_443, 0x0000),
                _FakeAxis(0x27, -83_443, 0x3110),
                _FakeAxis(0x40, 0, 0x0000),
            ]
        )

    def test_summary_page_has_deh_keys_and_no_space_key(self) -> None:
        output = StringIO()
        with redirect_stdout(output):
            console.render_arm_selection(
                self.make_real(), [0, 1, 2], 1, self.UNITS_PER_RAD, {0: 0, 1: 0, 2: 0}, ""
            )
        text = output.getvalue()
        self.assertIn(
            "Up/Down SELECT  Enter CONTROL  E ENABLE ALL  D DISABLE ALL  "
            "H HOME ALL  C CLEAR ERR  Q BACK",
            text,
        )
        self.assertIn("Select axis with Up/Down, Enter to control, Q to back", text)
        self.assertNotIn("Space", text)
        self.assertIn("ARM | SELECTED 2/3: ljoint2", text)
        self.assertIn("No.  Joint     Enable  Status", text)

    def test_error_latch_keeps_a_recent_code_visible_after_it_clears(self) -> None:
        # The ZeroLegacy drives report 0x603F in ~100 ms bursts, so one frame
        # only lists whichever axes are non-zero at that instant; the summary
        # page must keep recently seen codes visible with their age.
        latch = console.ArmErrorLatch(window_s=5.0)
        output = StringIO()
        with redirect_stdout(output):
            console.render_arm_selection(
                _FakeReal([_FakeAxis(0x21, 0, 0x3220)]),
                [0],
                0,
                self.UNITS_PER_RAD,
                {0: 0},
                "",
                latch,
            )
        self.assertIn("0x3220 (母线欠压)", output.getvalue())

        output = StringIO()
        with redirect_stdout(output):
            console.render_arm_selection(
                _FakeReal([_FakeAxis(0x21, 0, 0x0000)]),
                [0],
                0,
                self.UNITS_PER_RAD,
                {0: 0},
                "",
                latch,
            )
        text = output.getvalue()
        self.assertIn("0x3220 (母线欠压)", text)
        self.assertIn("s前", text)
        self.assertNotIn("0x0000 (无错误)", text)

        self.assertEqual(latch.clear(), 1)
        self.assertEqual(latch.cell("ljoint1", 0x0000, time.monotonic()), "0x0000 (无错误)")

    def test_error_latch_expires_after_its_window(self) -> None:
        latch = console.ArmErrorLatch(window_s=0.05)
        latch.update("ljoint2", 0x3220, time.monotonic() - 1.0)
        self.assertEqual(latch.cell("ljoint2", 0x0000, time.monotonic()), "0x0000 (无错误)")

    def test_summary_page_shows_enable_state_and_chinese_error_names(self) -> None:
        output = StringIO()
        with redirect_stdout(output):
            console.render_arm_selection(
                self.make_real(), [0, 1, 2], 0, self.UNITS_PER_RAD, {0: 0, 1: 0, 2: 0}, ""
            )
        text = output.getvalue()
        self.assertIn("使能", text)
        self.assertIn("失能", text)
        self.assertIn("0x3110 (主电源过压)", text)
        self.assertIn("0x0000 (无错误)", text)
        self.assertIn("Enabled: 2/3", text)
        self.assertIn("OPERATION_ENABLED", text)
        self.assertIn("SWITCH_ON_DISABLED", text)

    def test_joint_page_uses_wsdeh_keys(self) -> None:
        output = StringIO()
        with redirect_stdout(output):
            console.render_arm_control(
                self.make_real(),
                _FakeDesire(),
                1,
                self.UNITS_PER_RAD,
                0,
                0,
                0.05,
                "message here",
            )
        text = output.getvalue()
        self.assertIn("ARM | CONTROL ljoint2", text)
        self.assertIn("W +STEP  S -STEP  D DISABLE  E ENABLE  H HOME  Q BACK", text)
        self.assertIn("Error: 0x3110 (主电源过压)", text)
        self.assertIn("Enabled: 使能", text)
        self.assertNotIn("Space", text)
        self.assertIn("message here", text)

    def test_interactive_pages_have_no_space_or_zero_mark_binding(self) -> None:
        source = Path(console.__file__).read_text(encoding="utf-8")
        self.assertNotIn('key == " "', source)
        self.assertNotIn('key == "z"', source)

    def test_display_padding_counts_cjk_as_two_cells(self) -> None:
        self.assertEqual(console.display_width("使能"), 4)
        self.assertEqual(console.display_width("ljoint1"), 7)
        self.assertEqual(console.pad_display("使能", 8), "使能    ")
        self.assertEqual(console.pad_display("ljoint1", 10), "ljoint1   ")


class BusVoltageTest(unittest.TestCase):
    """The read-only 0x6079 / undervoltage-limit reporting."""

    UNITS_PER_RAD = 83443.02680376362

    def test_parse_ethercat_upload_reads_the_decimal_column(self) -> None:
        self.assertEqual(console.parse_ethercat_upload("0x00009a96 39574"), 39574)
        self.assertEqual(console.parse_ethercat_upload("0x0000abe0"), 44000)
        self.assertIsNone(console.parse_ethercat_upload(""))
        self.assertIsNone(console.parse_ethercat_upload("no value here"))

    def test_bus_status_line_reports_spread_and_per_family_limit(self) -> None:
        bus = {
            "voltage_mv": {
                "ljoint1": 39607,
                "rjoint3": 39541,
                "ljoint2": 39052,
                "rjoint6": 39179,
            },
            "limits": {
                "ljoint1": {"family": "EYOU", "min_mv": 14000, "max_mv": 75000},
                "rjoint3": {"family": "EYOU", "min_mv": 14000, "max_mv": 75000},
                "ljoint2": {"family": "ZeroErr", "min_mv": 44000, "max_mv": 55000},
                "rjoint6": {"family": "ZeroErr", "min_mv": 44000, "max_mv": 55000},
            },
        }
        self.assertEqual(
            console.bus_status_line(bus),
            "Bus 39.05-39.61V | EYOU 2x min 14.0V | ZeroErr 2x min 44.0V LOW",
        )

    def test_bus_status_line_is_silent_without_readings(self) -> None:
        self.assertIsNone(console.bus_status_line(None))
        self.assertIsNone(console.bus_status_line({}))
        self.assertIsNone(console.bus_status_line({"voltage_mv": {}}))

    def test_bus_status_line_tolerates_axes_without_a_limit_probe(self) -> None:
        bus = {"voltage_mv": {"ljoint1": 39607}, "limits": {}}
        self.assertEqual(console.bus_status_line(bus), "Bus 39.61-39.61V")

    def test_monitor_probes_each_drive_family_once(self) -> None:
        # 0x3B6F answers only on ZeroErr drives, 0x202D:01 only on EYOU drives.
        answers = {
            (0x6079, 0x00): {"ljoint1": 39500, "ljoint2": 39000},
            (0x3B6F, 0x00): {"ljoint2": 44000},
            (0x3B6E, 0x00): {"ljoint2": 55000},
            (0x202D, 0x01): {"ljoint1": 14000},
            (0x202D, 0x03): {"ljoint1": 75000},
        }
        calls = []

        def fake_read(master, position, index, subindex=0x00, timeout_s=1.0):
            joint = "ljoint1" if position == 0 else "ljoint2"
            calls.append((joint, index, subindex))
            return answers.get((index, subindex), {}).get(joint)

        monitor = console.BusVoltageMonitor({"ljoint1": (0, 0), "ljoint2": (0, 1)})
        with mock.patch.object(console, "read_ethercat_object", side_effect=fake_read):
            monitor.sample_once()
            snapshot = monitor.snapshot()
            probe_calls = len(calls)
            monitor.sample_once()

        self.assertEqual(snapshot["voltage_mv"], {"ljoint1": 39500, "ljoint2": 39000})
        self.assertEqual(snapshot["limits"]["ljoint1"]["family"], "EYOU")
        self.assertEqual(snapshot["limits"]["ljoint1"]["min_mv"], 14000)
        self.assertEqual(snapshot["limits"]["ljoint2"]["family"], "ZeroErr")
        self.assertEqual(snapshot["limits"]["ljoint2"]["min_mv"], 44000)
        # The limits are stable parameters: the second sample only reads 0x6079.
        self.assertEqual(len(calls) - probe_calls, 2)

    def test_summary_page_prints_the_bus_line(self) -> None:
        output = StringIO()
        with redirect_stdout(output):
            console.render_arm_selection(
                _FakeReal([_FakeAxis(0x21, 0, 0x0000)]),
                [0],
                0,
                self.UNITS_PER_RAD,
                {0: 0},
                "",
                None,
                {"voltage_mv": {"ljoint1": 39000}, "limits": {}},
            )
        self.assertIn("Bus 39.00-39.00V", output.getvalue())


if __name__ == "__main__":
    unittest.main()
