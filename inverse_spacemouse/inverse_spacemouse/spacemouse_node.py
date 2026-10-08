"""Publish SpaceMouse input (from hidraw) as CartesianTrajectoryPoint commands."""

import os
import select
import threading

from inverse_msgs.msg import CartesianTrajectoryPoint
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from inverse_spacemouse.hidraw import find_device, parse_report


class SpaceMouseNode(Node):

    def __init__(self):
        super().__init__("spacemouse")
        # Empty: auto-detect the first SpaceMouse under /sys/class/hidraw.
        self.declare_parameter("device", "")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate", 200.0)
        self.declare_parameter("full_scale", 350.0)
        self.declare_parameter("deadzone", 0.05)
        self.declare_parameter("max_linear", 0.1)
        self.declare_parameter("max_angular", 0.5)
        # Output i (linear x, y, z, angular x, y, z) = axis_sign[i] * raw[axis_map[i]].
        # The default maps the HID frame to ROS (x forward, y left, z up).
        self.declare_parameter("axis_map", [1, 0, 2, 4, 3, 5])
        self.declare_parameter("axis_sign", [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0])
        self.declare_parameter("num_buttons", 2)

        p = self.get_parameter
        self.device = p("device").value
        self.frame_id = p("frame_id").value
        self.full_scale = p("full_scale").value
        self.deadzone = p("deadzone").value
        self.max_linear = p("max_linear").value
        self.max_angular = p("max_angular").value
        self.axis_map = list(p("axis_map").value)
        self.axis_sign = list(p("axis_sign").value)
        self.num_buttons = p("num_buttons").value
        if len(self.axis_map) != 6 or len(self.axis_sign) != 6:
            raise ValueError("axis_map and axis_sign must have 6 entries")

        self.lock = threading.Lock()
        self.raw = [0] * 6
        self.buttons = 0
        self.last_stamp = -1

        self.command_pub = self.create_publisher(
            CartesianTrajectoryPoint, "spacemouse/command", 10
        )
        self.joy_pub = self.create_publisher(Joy, "spacemouse/joy", 10)
        self.create_timer(1.0 / p("publish_rate").value, self._publish)

        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _open(self):
        found = (self.device, self.device) if self.device else find_device()
        if found is None:
            return None, "no SpaceMouse found under /sys/class/hidraw"
        path, name = found
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except PermissionError:
            return None, (
                f"no permission to read {path}; install the udev rule from "
                "share/inverse_spacemouse/udev and replug the SpaceMouse"
            )
        except OSError as e:
            return None, f"cannot open {path}: {e}"
        self.get_logger().info(f"Reading {name} from {path}")
        return fd, None

    def _read_loop(self):
        fd = None
        last_error = None
        while not self._stop.is_set():
            if fd is None:
                fd, error = self._open()
                if fd is None:
                    if error != last_error:
                        self.get_logger().warning(f"{error}, retrying every second")
                        last_error = error
                    self._stop.wait(1.0)
                    continue
                last_error = None

            ready, _, _ = select.select([fd], [], [], 0.1)
            if not ready:
                continue
            try:
                data = os.read(fd, 64)
            except BlockingIOError:
                continue
            except OSError:
                data = b""
            if not data:
                self.get_logger().warning("SpaceMouse disconnected, publishing zeros")
                os.close(fd)
                fd = None
                with self.lock:
                    self.raw = [0] * 6
                    self.buttons = 0
                self._stop.wait(1.0)
                continue
            with self.lock:
                self.buttons = parse_report(data, self.raw, self.buttons)
        if fd is not None:
            os.close(fd)

    def _publish(self):
        with self.lock:
            raw = list(self.raw)
            buttons = self.buttons

        axes = []
        for i in range(6):
            v = self.axis_sign[i] * raw[self.axis_map[i]] / self.full_scale
            v = max(-1.0, min(1.0, v))
            axes.append(0.0 if abs(v) < self.deadzone else v)

        now = self.get_clock().now()
        cmd = CartesianTrajectoryPoint()
        # The controller normalizes the quaternion even in velocity mode.
        cmd.pose.orientation.w = 1.0
        lin, ang = cmd.velocity.linear, cmd.velocity.angular
        lin.x, lin.y, lin.z = (a * self.max_linear for a in axes[:3])
        ang.x, ang.y, ang.z = (a * self.max_angular for a in axes[3:])
        # time_from_start carries the current time; it must strictly advance.
        stamp = max(now.nanoseconds, self.last_stamp + 1)
        self.last_stamp = stamp
        cmd.time_from_start.sec, cmd.time_from_start.nanosec = divmod(
            stamp, 1_000_000_000
        )
        self.command_pub.publish(cmd)

        joy = Joy()
        joy.header.stamp = now.to_msg()
        joy.header.frame_id = self.frame_id
        joy.axes = axes
        joy.buttons = [(buttons >> i) & 1 for i in range(self.num_buttons)]
        self.joy_pub.publish(joy)

    def destroy_node(self):
        self._stop.set()
        self._reader.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
