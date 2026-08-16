#!/usr/bin/env python3
"""Visual indicator for recording state.

Subscribes to /arwun/recording_status and mirrors it onto something you can see
without a screen: a blinking LED on the 40-pin header, plus a terminal banner.

WHY NOT THE CONTROLLER. The obvious place for this indicator is the 8BitDo
itself -- player LEDs and rumble, already in the operator's hand. It is not
available on this rig: `hid_nintendo` does not exist on this L4T kernel
(`modinfo hid_nintendo` -> module not found), so the pad binds to `hid-generic`
and comes up with neither EV_LED nor EV_FF in its capability bits
(EV=10001b: SYN, KEY, ABS, MSC, REP -- no LED at bit 17, no FF at bit 21).
Nothing in userspace can light a pad LED the kernel does not expose. This is
the same shape of problem as the missing hid_sensor_* modules that forced the
source-built librealsense, and it has the same fix if it ever matters enough:
build hid-nintendo out of tree. Be aware that even then, 8BitDo's Switch-mode
emulation is not guaranteed to implement the rumble/LED subcommands.

WHY IT BLINKS RATHER THAN SITS SOLID. A solid LED and a wedged indicator node
look identical. A blink is a liveness signal: if it stops moving, either the
recording stopped or this node died, and both are things you want to walk over
and check. The recorder itself is a separate process, so this node dying does
NOT stop the recording -- see the health note in the README.

THE LED IS OPTIONAL. `led_pin` defaults to 0, meaning no GPIO at all and the
banner only, because driving a header pin that has something else wired to it
is worse than no indicator. Set it once you have an LED on the header.
"""

import signal
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

try:
    import Jetson.GPIO as GPIO
except ImportError:  # not a Jetson, or python3-jetson-gpio missing
    GPIO = None

# ANSI: white-on-red for recording, dim for idle. Terminals that do not
# understand these render the text anyway, which is the point.
_RED = '\033[1;37;41m'
_DIM = '\033[2m'
_OFF = '\033[0m'


class RecordIndicator(Node):
    def __init__(self):
        super().__init__('record_indicator')

        self.declare_parameter('status_topic', '/arwun/recording_status')
        self.declare_parameter('led_pin', 0)
        self.declare_parameter('blink_hz', 2.0)
        self.declare_parameter('banner', True)

        status_topic = self.get_parameter('status_topic').value
        self.led_pin = self.get_parameter('led_pin').value
        blink_hz = self.get_parameter('blink_hz').value
        self.banner = self.get_parameter('banner').value

        self._recording = False
        self._led_state = False
        self._gpio_ready = False

        self._setup_gpio()

        # MUST match the publisher's QoS. record_controller publishes status as
        # RELIABLE + TRANSIENT_LOCAL so that a subscriber joining mid-session
        # is handed the current state instead of waiting for the next toggle.
        # A default (VOLATILE) subscription here is still compatible, but it
        # silently forfeits that latched sample -- so an indicator started
        # after a recording began would sit dark until the operator pressed
        # stop. Keep these two profiles in step.
        status_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, status_topic, self._on_status, status_qos)

        # Free-running blink timer. Driving the LED from the timer rather than
        # from the status callback keeps the blink phase independent of message
        # arrival, so a quiet topic cannot leave the LED stuck mid-blink.
        self.create_timer(0.5 / max(blink_hz, 0.1), self._tick)

        where = (f'GPIO board pin {self.led_pin}'
                 if self._gpio_ready else 'terminal only (no LED pin)')
        self.get_logger().info(
            f'record_indicator ready: {status_topic} -> {where}')

    # -- GPIO ------------------------------------------------------------

    def _setup_gpio(self):
        if not self.led_pin:
            return
        if GPIO is None:
            self.get_logger().warn(
                'led_pin is set but Jetson.GPIO is not importable; '
                'falling back to the terminal banner only')
            return
        try:
            GPIO.setwarnings(False)
            # BOARD numbering: the physical pin number on the 40-pin header,
            # which is what you can actually count with a finger. BCM/TEGRA
            # numbering would be a second translation step at wiring time.
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.led_pin, GPIO.OUT, initial=GPIO.LOW)
            self._gpio_ready = True
        except Exception as exc:
            self.get_logger().warn(
                f'could not claim GPIO board pin {self.led_pin}: {exc}; '
                'falling back to the terminal banner only')

    def _write_led(self, on: bool):
        if not self._gpio_ready:
            return
        try:
            GPIO.output(self.led_pin, GPIO.HIGH if on else GPIO.LOW)
        except Exception as exc:
            self.get_logger().warn(f'GPIO write failed: {exc}',
                                   throttle_duration_sec=10.0)

    # -- status ----------------------------------------------------------

    def _on_status(self, msg: Bool):
        if msg.data == self._recording:
            return
        self._recording = msg.data

        if self.banner:
            if self._recording:
                sys.stdout.write(f'\n{_RED}  ● RECORDING  {_OFF}\n')
            else:
                sys.stdout.write(f'\n{_DIM}  ○ idle  {_OFF}\n')
            sys.stdout.flush()

        # Land on a defined level immediately rather than waiting up to half a
        # blink period, so stopping a take darkens the LED at once.
        if not self._recording:
            self._led_state = False
            self._write_led(False)

    def _tick(self):
        if not self._recording:
            return
        self._led_state = not self._led_state
        self._write_led(self._led_state)

    def destroy_node(self):
        # Leave the pin low and released; a bright LED after shutdown reads as
        # "still recording" to anyone glancing at the rig.
        if self._gpio_ready:
            try:
                GPIO.output(self.led_pin, GPIO.LOW)
                GPIO.cleanup(self.led_pin)
            except Exception:
                pass
        super().destroy_node()


def _install_sigterm_handler():
    """Make SIGTERM unwind through destroy_node() instead of killing us.

    Same reasoning as record_controller, different consequence. `ros2 launch`
    SIGTERMs its children on teardown, and Python's default disposition ends
    the process outright -- so destroy_node() never runs and the LED is left at
    whatever level the blink timer last wrote. Half the time that is HIGH, i.e.
    a rig sitting there advertising a recording that stopped when the launch
    did. Observed on the first bench run of this node.
    """
    def _raise(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _raise)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        _install_sigterm_handler()
        node = RecordIndicator()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is what rclpy's own signal handling raises
        # when the context is torn down under us. It is a normal exit here, not
        # a fault, and letting it propagate would print a traceback on every
        # clean Ctrl-C.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
