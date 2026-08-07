#!/usr/bin/env python3
"""Joystick-driven rosbag2 recorder.

Watches /joy for a toggle button and starts/stops a `ros2 bag record`
subprocess. The recorder is stopped with SIGINT so rosbag2 runs its normal
shutdown path and writes metadata.yaml; a bag killed with SIGKILL has no
metadata and will not open without a hand-written one.

Recording state is mirrored on /arwun/recording_status (std_msgs/Bool) with
transient-local durability, so a node that subscribes mid-session still
learns the current state.
"""

import os
import signal
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

# How long to wait for rosbag2 to finalize after SIGINT before escalating.
# Finalization is a metadata write, not a data flush, so this is generous.
SIGINT_GRACE_SEC = 15.0
SIGTERM_GRACE_SEC = 5.0


class RecordController(Node):
    def __init__(self):
        super().__init__('record_controller')

        self.declare_parameter('output_dir', os.path.expanduser('~/arwun_bags'))
        self.declare_parameter('bag_prefix', 'arwun')
        self.declare_parameter('topics', [''])
        self.declare_parameter('toggle_button', -1)
        self.declare_parameter('debounce_sec', 0.4)
        self.declare_parameter('storage_id', 'sqlite3')
        self.declare_parameter('extra_record_args', [''])
        self.declare_parameter('status_topic', '/arwun/recording_status')

        self.output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value)
        self.bag_prefix = self.get_parameter('bag_prefix').value
        self.topics = [t for t in self.get_parameter('topics').value if t]
        self.toggle_button = self.get_parameter('toggle_button').value
        self.debounce_sec = self.get_parameter('debounce_sec').value
        self.storage_id = self.get_parameter('storage_id').value
        self.extra_args = [
            a for a in self.get_parameter('extra_record_args').value if a]
        status_topic = self.get_parameter('status_topic').value

        if self.toggle_button < 0:
            raise RuntimeError(
                "Parameter 'toggle_button' must be set to a valid /joy button "
                'index. Run `ros2 topic echo /joy` to find it.')
        if not self.topics:
            raise RuntimeError(
                "Parameter 'topics' is empty; nothing would be recorded.")

        os.makedirs(self.output_dir, exist_ok=True)

        # Latch status so late subscribers get the current state immediately.
        status_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(Bool, status_topic, status_qos)

        self._proc = None
        self._current_bag = None
        self._prev_pressed = False
        self._last_edge_ns = 0

        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        # Reaps a recorder that exited on its own (disk full, bad topic name).
        self.create_timer(1.0, self._poll_child)

        self._publish_status()
        self.get_logger().info(
            f'record_controller ready. toggle=buttons[{self.toggle_button}], '
            f'{len(self.topics)} topics -> {self.output_dir}')

    # -- joy handling ----------------------------------------------------

    def _on_joy(self, msg: Joy):
        if self.toggle_button >= len(msg.buttons):
            self.get_logger().warn(
                f'toggle_button {self.toggle_button} out of range for a Joy '
                f'message with {len(msg.buttons)} buttons',
                throttle_duration_sec=10.0)
            return

        pressed = bool(msg.buttons[self.toggle_button])
        rising = pressed and not self._prev_pressed
        self._prev_pressed = pressed

        if not rising:
            return

        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_edge_ns) < self.debounce_sec * 1e9:
            return
        self._last_edge_ns = now_ns

        if self.is_recording:
            self.stop_recording()
        else:
            self.start_recording()

    # -- recorder lifecycle ----------------------------------------------

    @property
    def is_recording(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start_recording(self):
        if self.is_recording:
            return

        # ISO 8601 basic format with UTC offset: filesystem-safe (no colons)
        # and unambiguous across DST changes in the field.
        stamp = datetime.now().astimezone().strftime('%Y%m%dT%H%M%S%z')
        bag_path = os.path.join(self.output_dir, f'{self.bag_prefix}_{stamp}')

        cmd = ['ros2', 'bag', 'record',
               '--storage', self.storage_id,
               '--output', bag_path]
        cmd += self.extra_args
        cmd += self.topics

        try:
            # start_new_session puts the recorder in its own process group so
            # we can signal the whole group, and so a Ctrl-C aimed at this
            # node does not race our own SIGINT.
            self._proc = subprocess.Popen(cmd, start_new_session=True)
        except Exception as exc:
            self.get_logger().error(f'failed to start ros2 bag record: {exc}')
            self._proc = None
            self._publish_status()
            return

        self._current_bag = bag_path
        self.get_logger().info(f'recording -> {bag_path} (pid {self._proc.pid})')
        self._publish_status()

    def stop_recording(self):
        if self._proc is None:
            return

        if self._proc.poll() is not None:
            self._finish_stop()
            return

        bag = self._current_bag
        self.get_logger().info(f'stopping recorder for {bag} (SIGINT)')

        if not self._signal_and_wait(signal.SIGINT, SIGINT_GRACE_SEC):
            self.get_logger().warn(
                f'recorder did not exit {SIGINT_GRACE_SEC}s after SIGINT; '
                'escalating to SIGTERM')
            if not self._signal_and_wait(signal.SIGTERM, SIGTERM_GRACE_SEC):
                self.get_logger().error(
                    'recorder still alive after SIGTERM; sending SIGKILL. '
                    f'metadata.yaml for {bag} is likely missing -- recover it '
                    'with `ros2 bag reindex`.')
                self._signal_and_wait(signal.SIGKILL, 2.0)

        self._finish_stop()

    def _signal_and_wait(self, sig, timeout) -> bool:
        """Signal the recorder's process group. True if it exited in time."""
        try:
            os.killpg(os.getpgid(self._proc.pid), sig)
        except ProcessLookupError:
            return True
        except Exception as exc:
            self.get_logger().error(f'failed to signal recorder: {exc}')
            return False

        try:
            self._proc.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            return False

    def _finish_stop(self):
        bag = self._current_bag
        rc = self._proc.poll() if self._proc else None
        self._proc = None
        self._current_bag = None

        if bag and os.path.isfile(os.path.join(bag, 'metadata.yaml')):
            self.get_logger().info(f'finalized {bag} (exit {rc})')
        elif bag:
            self.get_logger().warn(
                f'{bag} has no metadata.yaml (exit {rc}); '
                f'run `ros2 bag reindex {bag}` before using it')
        self._publish_status()

    def _poll_child(self):
        """Detect a recorder that died without us asking it to."""
        if self._proc is not None and self._proc.poll() is not None:
            self.get_logger().error(
                f'recorder exited unexpectedly (code {self._proc.returncode})')
            self._finish_stop()

    # -- status ----------------------------------------------------------

    def _publish_status(self):
        self._status_pub.publish(Bool(data=self.is_recording))

    def destroy_node(self):
        # Never leave an unfinalized bag behind on shutdown.
        if self.is_recording:
            self.get_logger().info('shutting down with recorder active')
            self.stop_recording()
        super().destroy_node()


def _install_sigterm_handler():
    """Make SIGTERM unwind through the same path as Ctrl-C.

    Python's default SIGTERM disposition terminates the process outright, so
    destroy_node() -- and therefore the SIGINT-based finalization of an active
    recorder -- would never run. `ros2 launch` sends SIGTERM to its children
    during shutdown, so without this a bag is left unfinalized every time a
    launch is torn down mid-recording.
    """
    def _raise(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _raise)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        _install_sigterm_handler()
        node = RecordController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
