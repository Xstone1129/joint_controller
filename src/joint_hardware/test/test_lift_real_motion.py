#!/usr/bin/env python3
"""
Small real-lift smoke test using only the installed ros2 command line.

The controller and hardware must already be running.  This intentionally does
not import rclpy, so it can be copied to a test machine with just ROS 2 CLI.
"""

import argparse
import shutil
import signal
import subprocess
import sys
import time


def run(args):
    result = subprocess.run(args, text=True, capture_output=True)
    if result.returncode:
        raise RuntimeError("{} failed: {}".format(" ".join(args), result.stderr.strip()))
    return result.stdout


def service_call(name, srv_type, value):
    output = run(["ros2", "service", "call", name, srv_type, "{data: " + str(value).lower() + "}"])
    if "success: true" not in output.lower() and value:
        raise RuntimeError("service refused enable request: " + output.strip())
    print(output.strip())


def main():
    parser = argparse.ArgumentParser(description="bounded lift real-hardware motion smoke test")
    parser.add_argument("--direction", choices=("up", "down"), default="up")
    parser.add_argument(
        "--duration", type=float, default=1.0,
        help="motion duration in seconds (max 10)")
    parser.add_argument("--speed-rpm", type=float, default=10.0, help="jog speed (max 60 rpm)")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument(
        "--yes", action="store_true",
        help="skip the interactive safety confirmation")
    parser.add_argument(
        "--no-enable", action="store_true",
        help="do not call /lift_brake_command before motion")
    args = parser.parse_args()
    if shutil.which("ros2") is None:
        parser.error("ros2 command not found; source the ROS 2 environment first")
    if not (
        0.05 <= args.duration <= 10.0
        and 0.1 <= args.speed_rpm <= 60.0
        and 10 <= args.rate <= 100
    ):
        parser.error("duration must be 0.05..10 s, speed 0.1..60 rpm, rate 10..100 Hz")
    if not args.yes:
        answer = input("REAL LIFT WILL MOVE {} at {:.1f} rpm for {:.2f}s. Type MOVE: ".format(
            args.direction, args.speed_rpm, args.duration))
        if answer.strip() != "MOVE":
            print("aborted")
            return 2

    velocity = args.speed_rpm * (10.0 / 3.0) / 60000.0
    if args.direction == "down":
        velocity = -velocity
    publisher = None
    try:
        if not args.no_enable:
            service_call("/lift_brake_command", "std_srvs/srv/SetBool", True)
        publisher = subprocess.Popen([
            "ros2", "topic", "pub", "-r", str(args.rate),
            "/joint/lift/jog_velocity", "std_msgs/msg/Float64",
            "{data: " + str(velocity) + "}",
        ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        time.sleep(args.duration)
    except (OSError, RuntimeError) as error:
        print("ERROR:", error, file=sys.stderr)
        return 1
    finally:
        if publisher is not None and publisher.poll() is None:
            publisher.send_signal(signal.SIGINT)
            try:
                publisher.wait(timeout=2)
            except subprocess.TimeoutExpired:
                publisher.kill()
        # Zero jog is fail-safe even if the publisher failed to start.
        subprocess.run([
            "ros2", "topic", "pub", "--once", "/joint/lift/jog_velocity",
            "std_msgs/msg/Float64", "{data: 0.0}",
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run([
            "ros2", "service", "call", "/joint/lift/stop", "std_srvs/srv/Trigger", "{}",
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if not args.no_enable:
            subprocess.run([
                "ros2", "service", "call", "/lift_brake_command", "std_srvs/srv/SetBool",
                "{data: false}",
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("motion test finished; stop and brake-lock requested")
    return 0


if __name__ == "__main__":
    sys.exit(main())
