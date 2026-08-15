#!/usr/bin/env python3
"""Check what the D435i's IMU actually publishes while sitting still.

WHY: scripts/calibrate_imu.sh writes its fit to the camera's EEPROM, and
librealsense applies those intrinsics inside the sensor -- so /camera/imu
carries corrected values, not raw ones. That makes "did the calibration work?"
a question you can only answer by looking at live data. It is not the same
question as "did the calibration get written?", which `rs-enumerate-devices -c`
answers: the 2026-08-14 run wrote successfully, and was still wrong.

Verifying is far cheaper than calibrating -- 30 seconds against a ten-minute
six-pose procedure -- so run this after any calibration attempt before deciding
whether to accept it.

    # terminal 1
    ros2 launch arwun_bringup record.launch.py joy:=false description:=false
    # terminal 2
    python3 scripts/check_imu_bias.py

Put the camera on a flat surface and leave it alone for the duration. It does
not need to be level: the accelerometer is judged on the magnitude of its
vector, which does not depend on orientation. It does need to be STILL --
stationarity is checked from the gyro spread rather than assumed, and the run
says so when the reading cannot be trusted.

WHAT THE NUMBERS MEAN

Accelerometer, magnitude against 9.807 m/s^2:
    ~9.807          calibration is good
    ~8.949          the fit is not being applied (this camera's uncalibrated
                    reading -- a consistent -8.8% scale error)
    anything else   the fit is being applied but is wrong

    A single stationary pose cannot separate a scale error from a bias error;
    it only tells you the total is off. If the magnitude is bad, re-measure at
    two or three different orientations before concluding anything about which
    term is at fault -- a scale error reads the same everywhere, a bias error
    changes with orientation.

Gyroscope, mean over a stationary run, which is the residual bias:
    ~0              bias correction is being applied
    ~the fit        it is not

    Compared against the fit in calibration.json. Do not expect the residual to
    match the fit exactly even when nothing is being corrected: MEMS gyro bias
    moves between power cycles and with temperature, so 70-130% of the fitted
    value still means "uncorrected".

    As of 2026-08-14 the gyro bias is NOT corrected on this camera, and the
    cause is upstream rather than a bad fit -- see the Status section of
    README.md. A residual near the fitted value is the expected result here,
    not a new problem.

CONTEXT FOR VIO/SLAM: a standing gyro bias of this size is not on its own a
reason to hold off. VIO estimators carry gyro bias as an online state and are
built to absorb it. Accelerometer SCALE error is the more awkward of the two,
since estimators model accel bias but generally not accel scale.
"""

import argparse
import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

G = 9.807
UNCALIBRATED_MAGNITUDE = 8.949  # what this camera read before any calibration

# Gyro spread above which the camera was moving and the mean is meaningless.
# A camera at rest on a table sits around 0.002-0.003 rad/s on this sensor;
# 0.01 is comfortably clear of that without tolerating real motion.
STILLNESS_LIMIT = 0.01

WS_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def mean(xs):
    return sum(xs) / len(xs)


def stdev(xs):
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs))


def load_fitted_gyro_bias(path):
    """Fitted gyro bias from calibration.json, or None if unavailable.

    Only used to label the residual, so a missing or unreadable file degrades
    the report rather than failing the run.
    """
    try:
        with open(path) as fh:
            return json.load(fh)['imus'][0]['gyroscope']['bias']
    except (OSError, KeyError, IndexError, ValueError):
        return None


class Collector(Node):
    def __init__(self, target):
        super().__init__('check_imu_bias')
        self.target = target
        self.gyro = [[], [], []]
        self.accel = [[], [], []]
        # The driver publishes IMU on a best-effort sensor-data profile; a
        # reliable subscription would not match it and would sit silent.
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, '/camera/imu', self.callback, qos)

    def callback(self, msg):
        w, a = msg.angular_velocity, msg.linear_acceleration
        for axis, value in enumerate((w.x, w.y, w.z)):
            self.gyro[axis].append(value)
        for axis, value in enumerate((a.x, a.y, a.z)):
            self.accel[axis].append(value)

        n = len(self.gyro[0])
        if n % 1000 == 0:
            print(f'  {n}/{self.target}', flush=True)
        if n >= self.target:
            raise SystemExit(0)


def report(node, fitted_bias):
    n = len(node.gyro[0])
    gyro_mean = [mean(axis) for axis in node.gyro]
    gyro_std = [stdev(axis) for axis in node.gyro]
    magnitudes = [math.sqrt(x * x + y * y + z * z)
                  for x, y, z in zip(*node.accel)]
    magnitude = mean(magnitudes)

    print(f'\n=== {n} samples ===\n')

    print('ACCELEROMETER')
    print(f'  magnitude    {magnitude:.4f} +/- {stdev(magnitudes):.4f} m/s^2')
    print(f'  error        {magnitude - G:+.4f} m/s^2 '
          f'({(magnitude / G - 1) * 100:+.2f}% against {G})')
    if abs(magnitude - UNCALIBRATED_MAGNITUDE) < 0.05:
        print('  -> matches the UNCALIBRATED reading: the fit is not being '
              'applied.')
    elif abs(magnitude - G) < 0.1:
        print('  -> good (within 1%).')
    else:
        print('  -> the fit is being applied but is off. Re-measure at another '
              'orientation')
        print('     to tell a scale error (same everywhere) from a bias error '
              '(varies).')

    print('\nGYROSCOPE (mean at rest = residual bias)')
    print('  mean         [' + ', '.join(f'{v:+.6f}' for v in gyro_mean)
          + '] rad/s')
    print('  stdev        [' + ', '.join(f'{v:.6f}' for v in gyro_std)
          + '] rad/s')
    if fitted_bias:
        print('  fitted       [' + ', '.join(f'{v:+.6f}' for v in fitted_bias)
              + '] rad/s')
        ratios = [m / f if f else float('nan')
                  for m, f in zip(gyro_mean, fitted_bias)]
        print('  residual/fit [' + ', '.join(f'{r:.2f}' for r in ratios)
              + ']  (~1 = uncorrected, ~0 = corrected)')
    drift = math.sqrt(sum(v * v for v in gyro_mean))
    print(f'  drift        {math.degrees(drift) * 3600:.0f} deg/hour if '
          'integrated open-loop')
    print('               (a VIO estimator absorbs this as an online bias '
          'state)')

    print('\nSTATIONARITY')
    if max(gyro_std) > STILLNESS_LIMIT:
        print(f'  gyro spread {max(gyro_std):.6f} rad/s -- THE CAMERA MOVED.')
        print('  Both readings above are meaningless. Re-run without touching '
              'it.')
        return 1
    print(f'  gyro spread {max(gyro_std):.6f} rad/s -- still, readings valid.')
    return 0


def main():
    parser = argparse.ArgumentParser(
        description='Measure /camera/imu at rest to check the IMU calibration.')
    parser.add_argument('-n', '--samples', type=int, default=6000,
                        help='samples to collect; the IMU runs at 200 Hz, so '
                             'the default is about 30 s (default: %(default)s)')
    parser.add_argument('-c', '--calibration',
                        default=os.path.join(WS_ROOT, 'calibration.json'),
                        help='calibration.json holding the fitted gyro bias '
                             'to compare against (default: %(default)s)')
    args = parser.parse_args()

    fitted_bias = load_fitted_gyro_bias(args.calibration)
    if fitted_bias is None:
        print(f'note: no usable fit at {args.calibration} -- reporting the '
              'gyro residual without a comparison.\n')

    rclpy.init()
    node = Collector(args.samples)
    print(f'Collecting {args.samples} samples from /camera/imu '
          f'(~{args.samples / 200:.0f} s at 200 Hz). Leave the camera alone.',
          flush=True)

    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass

    # Partial runs are still worth reporting -- Ctrl-C after 20 s of a 30 s run
    # gives a perfectly good reading -- but a handful of samples is not.
    if len(node.gyro[0]) < 200:
        print(f'\nonly {len(node.gyro[0])} samples arrived. Is the launch up, '
              'and is /camera/imu')
        print('publishing? Without the source-built librealsense there is no '
              'Motion Module')
        print('and this topic never appears -- see scripts/build_librealsense.sh.')
        return 1

    status = report(node, fitted_bias)
    node.destroy_node()
    rclpy.shutdown()
    return status


if __name__ == '__main__':
    sys.exit(main())
