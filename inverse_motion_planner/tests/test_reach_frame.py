"""CLI unit tests; no running ROS graph or robot required."""

import argparse
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

loader = SourceFileLoader(
    "reach_frame", str(Path(__file__).resolve().parents[1] / "scripts/reach_frame")
)
spec = spec_from_loader(loader.name, loader)
cli = module_from_spec(spec)
loader.exec_module(cli)


class ReachFrameTests(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.buffer = Mock()
        patcher = patch.object(cli.rclpy, "ok", return_value=True)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_argument_defaults(self):
        args = cli.parse_args(["reach_frame", "homing"])
        self.assertEqual(args.target, "homing")
        self.assertEqual(args.velocity, 0.05)
        self.assertEqual(args.service, "/reach_position")

    def test_invalid_velocity(self):
        for value in ["0", "-1", "nan", "inf"]:
            with self.subTest(value=value):
                with self.assertRaises(argparse.ArgumentTypeError):
                    cli.positive_float(value)

    def test_direct_frame_has_priority(self):
        self.buffer.all_frames_as_yaml.return_value = (
            "homing: {parent: base}\nvia(homing): {parent: base}\n"
            "obs(homing): {parent: base}\n"
        )
        targets = cli.resolve_targets(self.node, self.buffer, "homing", 0)
        self.assertEqual(targets, ["homing"])
        self.buffer.lookup_transform.assert_not_called()

    def test_root_frame_is_available(self):
        self.buffer.all_frames_as_yaml.return_value = "homing: {parent: base}\n"
        self.assertEqual(cli.resolve_targets(self.node, self.buffer, "base", 0),
                         ["base"])

    def test_fallback_order(self):
        self.buffer.all_frames_as_yaml.return_value = (
            "via(part): {parent: base}\nobs(part): {parent: disconnected}\n"
        )
        targets = cli.resolve_targets(self.node, self.buffer, "part", 0)
        self.assertEqual(targets, ["via(part)", "obs(part)"])
        self.buffer.lookup_transform.assert_not_called()

    def test_missing_fallback_fails_before_returning_route(self):
        self.buffer.all_frames_as_yaml.return_value = "via(part): {parent: base}\n"
        with self.assertRaisesRegex(RuntimeError, "must exist"):
            cli.resolve_targets(self.node, self.buffer, "part", 0)
        self.node.create_client.assert_not_called()

    def test_spins_until_direct_frame_arrives(self):
        self.buffer.all_frames_as_yaml.side_effect = [
            "{}", "homing: {parent: base}\n"
        ]
        with patch.object(cli.rclpy, "spin_once") as spin:
            targets = cli.resolve_targets(self.node, self.buffer, "homing", 5)
        spin.assert_called_once()
        self.assertEqual(targets, ["homing"])

    def test_enqueue_preserves_queue(self):
        client = Mock()
        future = client.call_async.return_value
        future.done.return_value = True
        future.result.return_value = Mock(success=True, motion_ids=[42])
        with patch.object(cli.rclpy, "spin_until_future_complete"):
            cli.enqueue(self.node, client, "homing", 0.05, 5)
        request = client.call_async.call_args.args[0]
        self.assertEqual(request.desired_pos.header.frame_id, "homing")
        self.assertEqual(request.desired_pos.pose.position.x, 0.0)
        self.assertEqual(request.desired_pos.pose.position.y, 0.0)
        self.assertEqual(request.desired_pos.pose.position.z, 0.0)
        self.assertEqual(request.desired_pos.pose.orientation.w, 1.0)
        self.assertEqual(request.max_vel, 0.05)
        self.assertFalse(request.immediate_execution)

    def test_rejection(self):
        client = Mock()
        client.call_async.return_value.done.return_value = True
        client.call_async.return_value.result.return_value = Mock(success=False)
        with patch.object(cli.rclpy, "spin_until_future_complete"):
            with self.assertRaisesRegex(RuntimeError, "rejected"):
                cli.enqueue(self.node, client, "homing", 0.05, 5)

    def test_timeout_warns_about_unknown_queue_state(self):
        client = Mock()
        client.call_async.return_value.done.return_value = False
        with patch.object(cli.rclpy, "spin_until_future_complete"):
            with self.assertRaisesRegex(RuntimeError, "may already be queued"):
                cli.enqueue(self.node, client, "homing", 0.05, 5)
        client.call_async.assert_called_once()


if __name__ == "__main__":
    unittest.main()
