#!/usr/bin/env python3
"""Run YAML robot workflows through ROS 2 topics and services."""

from __future__ import annotations

import argparse
import ast
import json
import math
import select
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any

import yaml

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from robot_control_msg.msg import ArmMotionStatus, Robotarmjoint
from robot_control_msg.srv import CartesianIncrementControl


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_STATES = ROOT / "joint_state.md"
DEFAULT_WORKFLOW = ROOT / "workflows" / "example_box_workflow.yaml"

SHORT_KEYS = [f"l{i}" for i in range(1, 8)] + [f"r{i}" for i in range(1, 8)]
JOINT_FIELDS = [f"ljoint{i}" for i in range(1, 8)] + [f"rjoint{i}" for i in range(1, 8)]
JOINT_INDEX_BY_FIELD = {field: index for index, field in enumerate(JOINT_FIELDS)}
JOINT_STATE_NAMES = JOINT_FIELDS
CARTESIAN_FIELDS = [
    "lx", "ly", "lz", "lroll", "lpitch", "lyaw", "lqx", "lqy", "lqz", "lqw",
    "rx", "ry", "rz", "rroll", "rpitch", "ryaw", "rqx", "rqy", "rqz", "rqw",
]


class WorkflowInterrupted(Exception):
    """Raised when the user asks the workflow to stop."""


class WorkflowRunner(Node):
    def __init__(self, config: dict[str, Any], *, dry_run: bool = False) -> None:
        super().__init__("workflow_runner")
        self.config = config
        self.dry_run = dry_run
        self.defaults = config.get("defaults", {})
        self.topics = config.get("topics", {})
        self.service_names = config.get("services", {})

        self.power_topic = self.topics.get("power", "/robot_poweron")
        self.joint_topic = self.topics.get("joint_absolute", "/arm_joint_absolute_cmd")
        self.joint_state_topic = self.topics.get("joint_state", "/arm/joint_states")
        self.motion_status_topic = self.topics.get(
            "motion_status", "/arm/arm_controller/motion_status"
        )
        self.cartesian_increment_service = self.service_names.get(
            "cartesian_increment", "/cartesian_increment_control"
        )

        self.power_pub = self.create_publisher(Bool, self.power_topic, 10)
        self.joint_pub = self.create_publisher(Robotarmjoint, self.joint_topic, 10)
        self.cartesian_increment_client = self.create_client(
            CartesianIncrementControl, self.cartesian_increment_service
        )
        self.joint_state_sub = self.create_subscription(
            JointState, self.joint_state_topic, self._on_joint_state, 50
        )
        self.motion_status_sub = self.create_subscription(
            ArmMotionStatus, self.motion_status_topic, self._on_motion_status, 10
        )

        self._lock = threading.Lock()
        self._joint_positions: dict[str, float] = {}
        self._last_motion_status: ArmMotionStatus | None = None
        self._last_motion_status_time = 0.0
        self._stop_requested = False
        self._hard_stop_requested = False
        self._stop_raise_after = 0.0
        self._emergency_hold_pending = False

    def request_stop(self, *, power_off: bool = False) -> None:
        self._stop_requested = True
        if power_off:
            self._hard_stop_requested = True
            self.publish_power(False, reason="CtrlC x2 emergency poweroff")
            return

        self._emergency_hold_pending = True
        self._stop_raise_after = time.monotonic() + float(
            self.defaults.get("interrupt_poweroff_window", 1.0)
        )

    def _check_stop(self) -> None:
        if self._hard_stop_requested:
            raise WorkflowInterrupted("emergency poweroff requested")
        if self._stop_requested:
            if self._emergency_hold_pending:
                self._emergency_hold_pending = False
                self.publish_current_hold()
            if time.monotonic() < self._stop_raise_after:
                return
            raise WorkflowInterrupted("stop requested")

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._joint_positions[name] = float(position)

    def _on_motion_status(self, msg: ArmMotionStatus) -> None:
        with self._lock:
            self._last_motion_status = msg
            self._last_motion_status_time = time.monotonic()

    def publish_power(self, enabled: bool, *, reason: str = "") -> None:
        label = "ON" if enabled else "OFF"
        suffix = f" ({reason})" if reason else ""
        print(f"[power] publish {self.power_topic}: {label}{suffix}", flush=True)
        if self.dry_run:
            return

        self._wait_for_subscribers(self.power_pub, self.power_topic)
        repeats = int(self.defaults.get("power_publish_repeats", 5))
        interval = float(self.defaults.get("power_publish_interval", 0.1))
        msg = Bool()
        msg.data = bool(enabled)
        for _ in range(max(1, repeats)):
            self.power_pub.publish(msg)
            time.sleep(interval)

    def publish_joint_pose(self, pose: dict[str, float], *, vel: float, acc: float) -> None:
        msg = Robotarmjoint()
        for field in JOINT_FIELDS:
            setattr(msg, field, float(pose[field]))
        msg.vel = float(vel)
        msg.acc = float(acc)

        printable = ", ".join(f"{field}={getattr(msg, field):.4f}" for field in JOINT_FIELDS)
        print(f"[joint] publish {self.joint_topic}: {printable}, vel={vel}, acc={acc}", flush=True)
        if self.dry_run:
            return

        self._wait_for_subscribers(self.joint_pub, self.joint_topic)
        repeats = int(self.defaults.get("joint_publish_repeats", 3))
        interval = float(self.defaults.get("joint_publish_interval", 0.05))
        for _ in range(max(1, repeats)):
            self._check_stop()
            self.joint_pub.publish(msg)
            time.sleep(interval)

    def publish_current_hold(self) -> None:
        with self._lock:
            missing = [name for name in JOINT_STATE_NAMES if name not in self._joint_positions]
            if missing:
                print(f"[interrupt] cannot hold current pose; missing joint states: {missing}", flush=True)
                return
            pose = {name: self._joint_positions[name] for name in JOINT_STATE_NAMES}

        msg = Robotarmjoint()
        for field in JOINT_FIELDS:
            setattr(msg, field, float(pose[field]))
        msg.vel = float(self.defaults.get("emergency_hold_vel", self.defaults.get("joint_vel", 0.5)))
        msg.acc = float(self.defaults.get("emergency_hold_acc", self.defaults.get("joint_acc", 0.5)))

        printable = ", ".join(f"{field}={getattr(msg, field):.4f}" for field in JOINT_FIELDS)
        print(f"[interrupt] hold current pose on {self.joint_topic}: {printable}", flush=True)

        repeats = int(self.defaults.get("emergency_hold_repeats", 5))
        interval = float(self.defaults.get("joint_publish_interval", 0.05))
        for _ in range(max(1, repeats)):
            self.joint_pub.publish(msg)
            time.sleep(interval)

    def _wait_for_subscribers(self, publisher: Any, topic: str) -> None:
        timeout = float(self.defaults.get("publisher_match_timeout", 5.0))
        deadline = time.monotonic() + timeout
        while publisher.get_subscription_count() <= 0:
            self._check_stop()
            if time.monotonic() > deadline:
                raise TimeoutError(f"no subscriber matched for topic: {topic}")
            time.sleep(0.05)

    def wait_for_joint_pose(self, pose: dict[str, float], timeout: float, tolerance: float) -> bool:
        if self.dry_run:
            return True

        deadline = time.monotonic() + timeout
        saw_motion = False
        print(
            f"[wait] motion/position on {self.motion_status_topic}, tolerance={tolerance:.3f} rad",
            flush=True,
        )

        while time.monotonic() < deadline:
            self._check_stop()
            if self._pose_reached(pose, tolerance):
                return True

            with self._lock:
                status = self._last_motion_status
                status_age = time.monotonic() - self._last_motion_status_time

            if status is not None and status_age < 2.0:
                saw_motion = saw_motion or bool(status.is_moving)
                if saw_motion and not status.is_moving and status.goal_reached:
                    return True

            time.sleep(0.05)

        error_text = self._pose_error_summary(pose)
        with self._lock:
            status = self._last_motion_status
            status_age = time.monotonic() - self._last_motion_status_time
        if status is None:
            status_text = "motion_status=none"
        else:
            status_text = (
                f"motion_status is_moving={status.is_moving}, "
                f"goal_reached={status.goal_reached}, age={status_age:.2f}s"
            )
        print(f"[wait] timeout: {error_text}; {status_text}", flush=True)
        return False

    def _pose_reached(self, pose: dict[str, float], tolerance: float) -> bool:
        with self._lock:
            if not all(name in self._joint_positions for name in JOINT_STATE_NAMES):
                return False
            current = {name: self._joint_positions[name] for name in JOINT_STATE_NAMES}

        for field, target in pose.items():
            error = abs(current[field] - target)
            if error > math.pi:
                error = 2 * math.pi - error
            if error > tolerance:
                return False
        return True

    def _pose_error_summary(self, pose: dict[str, float]) -> str:
        with self._lock:
            current = dict(self._joint_positions)

        if not all(name in current for name in JOINT_STATE_NAMES):
            missing = [name for name in JOINT_STATE_NAMES if name not in current]
            return f"missing_joint_states={missing}"

        worst_field = ""
        worst_error = -1.0
        worst_current = 0.0
        worst_target = 0.0
        for field, target in pose.items():
            error = abs(current[field] - target)
            if error > math.pi:
                error = 2 * math.pi - error
            if error > worst_error:
                worst_field = field
                worst_error = error
                worst_current = current[field]
                worst_target = target

        return (
            f"max_error {worst_field}={worst_error:.4f} "
            f"(current={worst_current:.4f}, target={worst_target:.4f})"
        )

    def call_cartesian_increment(self, values: dict[str, float], timeout: float) -> None:
        printable = ", ".join(f"{key}: {value}" for key, value in values.items())
        print(f"[cartesian_increment] call {self.cartesian_increment_service}: {{{printable}}}", flush=True)
        if self.dry_run:
            return

        deadline = time.monotonic() + timeout
        while not self.cartesian_increment_client.wait_for_service(timeout_sec=0.2):
            self._check_stop()
            if time.monotonic() > deadline:
                raise TimeoutError(f"service not available: {self.cartesian_increment_service}")

        req = CartesianIncrementControl.Request()
        for field, value in values.items():
            setattr(req, field, float(value))

        future = self.cartesian_increment_client.call_async(req)
        while rclpy.ok() and not future.done():
            self._check_stop()
            if time.monotonic() > deadline:
                raise TimeoutError(f"service call timed out: {self.cartesian_increment_service}")
            time.sleep(0.05)

        response = future.result()
        if response is None:
            raise RuntimeError(f"service call failed: {self.cartesian_increment_service}")
        print(f"[cartesian_increment] response success={response.success}: {response.message}", flush=True)
        if not response.success:
            raise RuntimeError(response.message or "cartesian increment failed")

    def sleep_checked(self, seconds: float, label: str = "delay") -> None:
        if seconds <= 0:
            return
        print(f"[{label}] wait {seconds:.2f}s", flush=True)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._check_stop()
            time.sleep(min(0.1, deadline - time.monotonic()))


class DryWorkflowRunner:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.dry_run = True
        self.defaults = config.get("defaults", {})
        self.topics = config.get("topics", {})
        self.service_names = config.get("services", {})
        self.power_topic = self.topics.get("power", "/robot_poweron")
        self.joint_topic = self.topics.get("joint_absolute", "/arm_joint_absolute_cmd")
        self.motion_status_topic = self.topics.get(
            "motion_status", "/arm/arm_controller/motion_status"
        )
        self.cartesian_increment_service = self.service_names.get(
            "cartesian_increment", "/cartesian_increment_control"
        )
        self._stop_requested = False
        self._hard_stop_requested = False
        self._stop_raise_after = 0.0
        self._emergency_hold_pending = False

    def request_stop(self, *, power_off: bool = False) -> None:
        self._stop_requested = True
        if power_off:
            self._hard_stop_requested = True
            self.publish_power(False, reason="CtrlC x2 emergency poweroff")
            return
        self._stop_raise_after = time.monotonic() + float(
            self.defaults.get("interrupt_poweroff_window", 1.0)
        )

    def _check_stop(self) -> None:
        if self._hard_stop_requested:
            raise WorkflowInterrupted("emergency poweroff requested")
        if self._stop_requested:
            if time.monotonic() < self._stop_raise_after:
                return
            raise WorkflowInterrupted("stop requested")

    def publish_power(self, enabled: bool, *, reason: str = "") -> None:
        label = "ON" if enabled else "OFF"
        suffix = f" ({reason})" if reason else ""
        print(f"[dry-run][power] publish {self.power_topic}: {label}{suffix}", flush=True)

    def publish_joint_pose(self, pose: dict[str, float], *, vel: float, acc: float) -> None:
        printable = ", ".join(f"{field}={pose[field]:.4f}" for field in JOINT_FIELDS)
        print(f"[dry-run][joint] publish {self.joint_topic}: {printable}, vel={vel}, acc={acc}", flush=True)

    def wait_for_joint_pose(self, pose: dict[str, float], timeout: float, tolerance: float) -> bool:
        del pose, timeout
        print(
            f"[dry-run][wait] would wait on {self.motion_status_topic}, tolerance={tolerance:.3f} rad",
            flush=True,
        )
        return True

    def call_cartesian_increment(self, values: dict[str, float], timeout: float) -> None:
        del timeout
        printable = ", ".join(f"{key}: {value}" for key, value in values.items())
        print(
            f"[dry-run][cartesian_increment] call {self.cartesian_increment_service}: {{{printable}}}",
            flush=True,
        )

    def sleep_checked(self, seconds: float, label: str = "delay") -> None:
        if seconds > 0:
            print(f"[dry-run][{label}] wait {seconds:.2f}s", flush=True)


def load_joint_states(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}

    text = path.read_text(encoding="utf-8")
    if "```python" in text:
        start = text.index("```python") + len("```python")
        end = text.index("```", start)
        text = text[start:end]

    tree = ast.parse(text)
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "box_motion_joint_states":
                    return ast.literal_eval(node.value)
    return {}


def load_workflow(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        if path.suffix.lower() == ".json":
            data = json.load(f)
        else:
            data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError("workflow must be a mapping")
    if "steps" not in data or not isinstance(data["steps"], list):
        raise ValueError("workflow must contain a list field named 'steps'")
    return data


def normalize_pose(raw: dict[str, Any]) -> dict[str, float]:
    if not isinstance(raw, dict):
        raise ValueError("joint pose must be a mapping")

    pose: dict[str, float] = {}
    for short, field in zip(SHORT_KEYS, JOINT_FIELDS):
        if field in raw:
            pose[field] = float(raw[field])
        elif short in raw:
            pose[field] = float(raw[short])
        else:
            raise ValueError(f"joint pose missing {short}/{field}")
    return pose


def apply_joint_overrides(base: dict[str, float], raw: dict[str, Any]) -> dict[str, float]:
    if not isinstance(raw, dict):
        raise ValueError("joint overrides must be a mapping")

    pose = dict(base)
    for key, value in raw.items():
        if key in JOINT_FIELDS:
            pose[key] = float(value)
            continue
        if key in SHORT_KEYS:
            pose[JOINT_FIELDS[SHORT_KEYS.index(key)]] = float(value)
            continue
        raise ValueError(f"unsupported joint field: {key}")
    return pose


def resolve_pose(step: dict[str, Any], poses: dict[str, Any], current_pose: dict[str, float]) -> dict[str, float]:
    if "pose" in step:
        pose_name = step["pose"]
        if pose_name not in poses:
            raise KeyError(f"unknown pose: {pose_name}")
        pose = normalize_pose(poses[pose_name])
    else:
        pose = dict(current_pose)

    if "joints" in step:
        return apply_joint_overrides(pose, step["joints"])

    if "pose" in step:
        return pose

    raise ValueError("joint step must contain 'pose' or 'joints'")


def cartesian_values(step: dict[str, Any], defaults: dict[str, Any]) -> dict[str, float]:
    values = {field: float(step.get(field, 0.0)) for field in CARTESIAN_FIELDS}
    if "dz" in step:
        values["lz"] = float(step["dz"])
        values["rz"] = float(step["dz"])
    if "dy_in" in step:
        amount = float(step["dy_in"])
        values["ly"] = -amount
        values["ry"] = amount
    if "request" in step:
        for key, value in step["request"].items():
            if key not in CARTESIAN_FIELDS and key not in ("vel", "acc"):
                raise ValueError(f"unsupported cartesian field: {key}")
            values[key] = float(value)
    values["vel"] = float(step.get("vel", values.get("vel", defaults.get("cartesian_vel", 0.02))))
    values["acc"] = float(step.get("acc", values.get("acc", defaults.get("cartesian_acc", 0.05))))
    return values


def expand_joint_group(group: Any) -> list[str]:
    if isinstance(group, int):
        return [f"ljoint{group}", f"rjoint{group}"]
    if isinstance(group, str):
        if group.startswith("lrjoint"):
            number = int(group.removeprefix("lrjoint"))
            return [f"ljoint{number}", f"rjoint{number}"]
        if group.startswith("joint"):
            number = int(group.removeprefix("joint"))
            return [f"ljoint{number}", f"rjoint{number}"]
        if group in JOINT_INDEX_BY_FIELD:
            return [group]
    if isinstance(group, list):
        fields: list[str] = []
        for item in group:
            fields.extend(expand_joint_group(item))
        return fields
    raise ValueError(f"unsupported joint order group: {group!r}")


def should_confirm(step: dict[str, Any], defaults: dict[str, Any], args: argparse.Namespace) -> bool:
    if args.no_confirm:
        return False
    if args.confirm_all:
        return True
    default_confirm = bool(defaults.get("confirm_after", False))
    return bool(step.get("confirm_after", default_confirm))


def wait_for_enter(runner: WorkflowRunner, text: str) -> None:
    print(f"[confirm] {text} Press Enter to continue...", end="", flush=True)
    while True:
        runner._check_stop()
        readable, _, _ = select.select([sys.stdin], [], [], 0.1)
        if readable:
            sys.stdin.readline()
            print("", flush=True)
            runner._check_stop()
            return


def run_joint_step(
    runner: WorkflowRunner,
    step: dict[str, Any],
    poses: dict[str, Any],
    current_pose: dict[str, float],
) -> dict[str, float]:
    defaults = runner.defaults
    target_pose = resolve_pose(step, poses, current_pose)
    vel = float(step.get("vel", defaults.get("joint_vel", 0.5)))
    acc = float(step.get("acc", defaults.get("joint_acc", 0.5)))
    wait_motion = bool(step.get("wait_motion", defaults.get("wait_motion", True)))
    motion_timeout = float(step.get("motion_timeout", defaults.get("motion_timeout", 30.0)))
    tolerance = float(step.get("position_tolerance", defaults.get("position_tolerance", 0.05)))
    ordered_delay = float(step.get("ordered_delay", defaults.get("ordered_delay", 0.0)))

    order = step.get("order")
    if not order:
        runner.publish_joint_pose(target_pose, vel=vel, acc=acc)
        if wait_motion and not runner.wait_for_joint_pose(target_pose, motion_timeout, tolerance):
            raise TimeoutError(f"joint target not reached: {step.get('name', step.get('pose', 'joint'))}")
        return target_pose

    staged_pose = dict(current_pose)
    for group_index, group in enumerate(order, 1):
        fields = expand_joint_group(group)
        for field in fields:
            staged_pose[field] = target_pose[field]
        group_label = ",".join(fields)
        print(f"[joint] ordered group {group_index}/{len(order)}: {group_label}", flush=True)
        runner.publish_joint_pose(staged_pose, vel=vel, acc=acc)
        if wait_motion and not runner.wait_for_joint_pose(staged_pose, motion_timeout, tolerance):
            raise TimeoutError(f"joint ordered group not reached: {group_label}")
        runner.sleep_checked(ordered_delay, label="ordered_delay")

    return staged_pose


def run_step(
    index: int,
    runner: WorkflowRunner,
    step: dict[str, Any],
    poses: dict[str, Any],
    current_pose: dict[str, float],
    args: argparse.Namespace,
) -> dict[str, float]:
    action = step.get("action", step.get("type", "joint"))
    name = step.get("name") or step.get("pose") or f"step_{index}"
    print(f"\n==> [{index}/{len(runner.config['steps'])}] {name} ({action})", flush=True)

    if action == "power":
        runner.publish_power(bool(step.get("enable", True)))
    elif action == "joint":
        current_pose = run_joint_step(runner, step, poses, current_pose)
    elif action == "cartesian_increment":
        values = cartesian_values(step, runner.defaults)
        timeout = float(step.get("service_timeout", runner.defaults.get("service_timeout", 60.0)))
        runner.call_cartesian_increment(values, timeout)
    elif action == "sleep":
        runner.sleep_checked(float(step.get("seconds", step.get("delay", 0.0))))
    else:
        raise ValueError(f"unsupported step action: {action}")

    if "delay_after" in step or "sleep" in step:
        delay = float(step.get("delay_after", step.get("sleep", 0.0)))
    elif action == "cartesian_increment":
        delay = float(runner.defaults.get("cartesian_delay_after", runner.defaults.get("delay_after", 0.0)))
    elif action == "joint":
        delay = float(runner.defaults.get("joint_delay_after", runner.defaults.get("delay_after", 0.0)))
    else:
        delay = float(runner.defaults.get("delay_after", 0.0))
    runner.sleep_checked(delay, label="delay_after")

    if should_confirm(step, runner.defaults, args):
        wait_for_enter(runner, step.get("confirm_text", f"{name} complete."))

    return current_pose


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a robot motion workflow.")
    parser.add_argument("workflow", nargs="?", type=Path, default=DEFAULT_WORKFLOW)
    parser.add_argument("--states", type=Path, default=DEFAULT_STATES)
    parser.add_argument("--dry-run", action="store_true", help="Print actions without publishing/calling.")
    parser.add_argument("--list-states", action="store_true", help="List saved joint states and exit.")
    parser.add_argument("--no-power", action="store_true", help="Skip automatic power_on_at_start.")
    parser.add_argument("--power-off-at-end", action="store_true", help="Force power off after success.")
    parser.add_argument("--confirm-all", action="store_true", help="Require Enter after every step.")
    parser.add_argument("--no-confirm", action="store_true", help="Ignore confirm_after in workflow.")
    args = parser.parse_args()

    states = load_joint_states(args.states)
    if args.list_states:
        for name in states:
            print(name)
        return 0

    workflow = load_workflow(args.workflow)
    poses = dict(states)
    poses.update(workflow.get("poses", {}))
    defaults = workflow.get("defaults", {})
    current_pose = normalize_pose(workflow.get("initial_pose", {key: 0.0 for key in SHORT_KEYS}))

    print(f"Workflow: {args.workflow}", flush=True)
    print(f"Name: {workflow.get('name', args.workflow.stem)}", flush=True)
    print("Ctrl+C once: stop workflow. Ctrl+C twice: publish power off.", flush=True)

    executor: MultiThreadedExecutor | None = None
    runner: WorkflowRunner | DryWorkflowRunner
    if args.dry_run:
        runner = DryWorkflowRunner(workflow)
    else:
        rclpy.init(args=None)
        runner = WorkflowRunner(workflow, dry_run=False)
        executor = MultiThreadedExecutor()
        executor.add_node(runner)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

    interrupt_count = 0
    previous_handler = signal.getsignal(signal.SIGINT)

    def on_sigint(signum: int, frame: Any) -> None:
        nonlocal interrupt_count
        del signum, frame
        interrupt_count += 1
        if interrupt_count == 1:
            print("\n[interrupt] stop requested. Press Ctrl+C again to power off.", flush=True)
            runner.request_stop(power_off=False)
            return
        print("\n[interrupt] emergency power off.", flush=True)
        runner.request_stop(power_off=True)

    signal.signal(signal.SIGINT, on_sigint)

    try:
        if workflow.get("power_on_at_start", workflow.get("power_on", True)) and not args.no_power:
            runner.publish_power(True, reason="workflow start")

        for index, step in enumerate(workflow["steps"], 1):
            runner._check_stop()
            current_pose = run_step(index, runner, step, poses, current_pose, args)

        if args.power_off_at_end or workflow.get("power_off_at_end", False):
            runner.publish_power(False, reason="workflow end")

        print("\nWorkflow finished.", flush=True)
        return 0
    except WorkflowInterrupted as exc:
        print(f"\nWorkflow stopped: {exc}", file=sys.stderr, flush=True)
        return 130
    finally:
        signal.signal(signal.SIGINT, previous_handler)
        if executor is not None:
            executor.shutdown()
        if isinstance(runner, WorkflowRunner):
            runner.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (KeyError, ValueError, TimeoutError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
