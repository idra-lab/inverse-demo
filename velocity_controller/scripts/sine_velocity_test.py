#!/usr/bin/env python3
"""Manually test Cartesian velocity control, one base-frame axis at a time."""

import argparse
import math
import time

import rclpy
from inverse_msgs.msg import CartesianTrajectoryPoint
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args


def positive_float(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return result


def waveform(elapsed, period, pause, axes, linear_amplitude, angular_amplitude):
    """Return (axis index, velocity); None denotes a zero-command pause."""
    slot = int(elapsed / (period + pause))
    if slot >= len(axes):
        return None, 0.0
    phase = elapsed - slot * (period + pause)
    if phase >= period:
        return None, 0.0
    axis = axes[slot]
    amplitude = linear_amplitude if axis < 3 else angular_amplitude
    return axis, amplitude * math.sin(2.0 * math.pi * phase / period)


class SineVelocityTest(Node):
    def __init__(self, topic):
        super().__init__("sine_velocity_test")
        self.publisher = self.create_publisher(CartesianTrajectoryPoint, topic, 10)
        self.start_time = time.monotonic_ns()
        self.last_stamp = -1

    def publish_velocity(self, axis=None, velocity=0.0):
        msg = CartesianTrajectoryPoint()
        # Even velocity mode normalizes the quaternion in its command callback.
        msg.pose.orientation.w = 1.0
        if axis is not None:
            vector = msg.velocity.linear if axis < 3 else msg.velocity.angular
            setattr(vector, ("x", "y", "z")[axis % 3], velocity)
        stamp = max(time.monotonic_ns() - self.start_time, self.last_stamp + 1)
        self.last_stamp = stamp
        msg.time_from_start.sec, msg.time_from_start.nanosec = divmod(
            stamp, 1_000_000_000
        )
        self.publisher.publish(msg)

    def stream_zeros(self, duration, rate):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / rate)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/cartesian_velocity_controller/commands")
    parser.add_argument("--linear-amplitude", type=positive_float, default=0.010,
                        help="peak translation velocity in m/s (default: 0.005)")
    parser.add_argument("--angular-amplitude", type=positive_float, default=0.03,
                        help="peak rotation velocity in rad/s (default: 0.01)")
    parser.add_argument("--period", type=positive_float, default=4.0,
                        help="seconds per complete sine cycle (default: 4)")
    parser.add_argument("--pause", type=positive_float, default=0.5,
                        help="zero-command seconds between axes (default: 0.5)")
    parser.add_argument("--rate", type=positive_float, default=100.0,
                        help="publishing frequency in Hz (default: 100)")
    rotation_options = parser.add_mutually_exclusive_group()
    rotation_options.add_argument("--rotation", dest="rotation", action="store_true",
                                  help="append angular X/Y/Z (enabled by default)")
    rotation_options.add_argument("--translation-only", dest="rotation",
                                  action="store_false",
                                  help="skip angular checks; test translation only")
    parser.set_defaults(rotation=True)
    parser.add_argument("--repeat", action="store_true",
                        help="repeat until Ctrl+C; otherwise run once")
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    if options.rate < 50.0:
        parser.error("--rate must be at least 50 Hz for the default 0.1 s timeout")

    # Keep Python's Ctrl+C handling so the ROS context remains alive for zeros.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = SineVelocityTest(options.topic)
        node.get_logger().warning(
            "Motion test: use velocity mode with P_gain=0, verify base/tip frames, "
            "clear the workspace, and activate the controller manually."
        )
        deadline = time.monotonic() + 5.0
        while node.publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError("No command subscriber found within 5 seconds")
            rclpy.spin_once(node, timeout_sec=0.1)

        node.stream_zeros(0.5, options.rate)
        axes = list(range(6 if options.rotation else 3))
        sequence_duration = len(axes) * (options.period + options.pause)
        start = time.monotonic()
        previous_axis = None
        while rclpy.ok():
            elapsed = time.monotonic() - start
            if not options.repeat and elapsed >= sequence_duration:
                break
            axis, velocity = waveform(
                elapsed % sequence_duration, options.period, options.pause,
                axes, options.linear_amplitude, options.angular_amplitude,
            )
            if axis is not None and axis != previous_axis:
                label = ("linear " if axis < 3 else "angular ") + "xyz"[axis % 3]
                node.get_logger().info(f"Testing {label}")
            previous_axis = axis
            node.publish_velocity(axis, velocity)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(1.0 / options.rate)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        if node is not None:
            node.get_logger().error(str(error))
        return 1
    finally:
        if node is not None:
            try:
                if rclpy.ok():
                    node.get_logger().info("Publishing zero velocity for 0.5 seconds")
                    node.stream_zeros(0.5, options.rate)
            except KeyboardInterrupt:
                pass
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
