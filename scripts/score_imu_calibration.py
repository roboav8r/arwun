#!/usr/bin/env python3
"""Score an accelerometer calibration across all six poses it was fitted from.

WHY THIS EXISTS: a stationary reading of /camera/imu tells you the error at ONE
orientation, and that is not enough to judge a calibration. This camera's error
is dominated by BIAS, not scale, so the magnitude it reports depends on which
way it is pointing -- raw, it ranges from 8.95 to 10.31 m/s^2 depending on
pose. Two readings taken at different orientations are not comparable, and
reading a single pose can make a better calibration look worse.

That confusion cost a debugging round on 2026-08-14, and it is also the origin
of this project's long-standing but wrong belief in a "consistent -8.8% scale
error" -- that figure came from one orientation (the -y pose, which reads
8.946). A pure scale error would read the same everywhere.

So judge a calibration the way it will actually be used: worst-case magnitude
error over every orientation. The six-pose raw dumps that calibrate_imu.sh
optionally saves already contain exactly that.

    ./scripts/score_imu_calibration.py accel_2.txt

Compares three options against those samples: no calibration at all, whatever
the camera currently holds, and the fit in calibration.json. Add -r <ref> to
compare against some other saved fit, e.g. a git revision:

    ./scripts/score_imu_calibration.py accel_2.txt -r HEAD~1

The device model is  corrected = S @ raw - b  (bias SUBTRACTED -- see
librealsense src/proc/motion-transform.cpp, correct_motion_helper). Getting
that sign wrong makes a good fit look terrible on paper.

Reads the camera if it is connected and free, so stop the ROS driver first;
without it the current-EEPROM row is skipped and the rest still works.
"""

import argparse
import json
import os
import subprocess
import sys

import numpy as np

G = 9.807
WS_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LRS_LIB = os.path.join(WS_ROOT, 'vendor', 'librealsense', 'lib')
PY_MODULE_DIR = os.path.join(WS_ROOT, 'vendor', 'src', 'librealsense', 'build',
                             'Release')


def _reexec_with_rsusb_library():
    """Re-exec with the RSUSB librealsense on LD_LIBRARY_PATH.

    Without it, `import pyrealsense2` still succeeds -- it just binds to the
    apt librealsense, which on this kernel enumerates the Stereo Module and NO
    Motion Module, so the camera appears to have no IMU to read a calibration
    from. That failure is silent and looks exactly like a disconnected camera,
    which is the same root cause as the missing /camera/imu that
    scripts/build_librealsense.sh exists to solve. The loader only reads
    LD_LIBRARY_PATH at process start, so this has to happen by re-exec.
    """
    if os.environ.get('_ARWUN_RSUSB_BOOTSTRAPPED'):
        return
    env = dict(os.environ)
    env['_ARWUN_RSUSB_BOOTSTRAPPED'] = '1'
    existing = env.get('LD_LIBRARY_PATH', '')
    if LRS_LIB not in existing.split(os.pathsep):
        env['LD_LIBRARY_PATH'] = (
            os.pathsep.join([LRS_LIB, existing]) if existing else LRS_LIB)
    if os.path.isdir(LRS_LIB):
        os.execve(sys.executable, [sys.executable] + sys.argv, env)


_reexec_with_rsusb_library()


def fit_from_json(obj):
    acc = obj['imus'][0]['accelerometer']
    return (np.array(acc['scale_and_alignment']).reshape(3, 3),
            np.array(acc['bias']))


def fit_from_device():
    """(S, b) currently on the camera, or None if it cannot be read."""
    for path in (PY_MODULE_DIR, LRS_LIB):
        if path not in sys.path:
            sys.path.insert(0, path)
    try:
        import pyrealsense2 as rs
    except ImportError:
        return None
    try:
        devices = rs.context().query_devices()
        if not len(devices):
            return None
        for sensor in devices[0].sensors:
            if 'Motion' not in sensor.get_info(rs.camera_info.name):
                continue
            for profile in sensor.get_stream_profiles():
                if (profile.stream_type() == rs.stream.accel
                        and profile.format() == rs.format.motion_xyz32f):
                    data = np.array(profile.as_motion_stream_profile()
                                    .get_motion_intrinsics().data)
                    return data[:, :3], data[:, 3]
    except RuntimeError:
        # Device busy: the ROS driver holds it.
        return None
    print('note: camera found but it reports no Motion Module, so its current\n'
          '      calibration cannot be read. That is the apt-librealsense\n'
          '      symptom -- check that vendor/librealsense/lib exists.')
    return None


def fit_from_git(ref):
    blob = subprocess.check_output(
        ['git', '-C', WS_ROOT, 'show', f'{ref}:calibration.json'])
    return fit_from_json(json.loads(blob))


def bucket_poses(samples):
    """Group samples by which of the six +/-axis directions dominates."""
    poses = {}
    for row in samples:
        axis = int(np.argmax(np.abs(row)))
        poses.setdefault(f'{"+" if row[axis] > 0 else "-"}{"xyz"[axis]}',
                         []).append(row)
    return {k: np.array(v) for k, v in sorted(poses.items())}


def score(name, S, b, poses, verbose):
    errors = []
    if verbose:
        print(f'\n{name}')
        print(f'  {"pose":6s} {"n":>6s} {"raw |a|":>9s} {"corrected |a|":>14s} '
              f'{"error":>9s}')
    for pose, rows in poses.items():
        raw = rows.mean(axis=0)
        magnitude = np.linalg.norm(S @ raw - b)
        errors.append(magnitude - G)
        if verbose:
            print(f'  {pose:6s} {len(rows):6d} {np.linalg.norm(raw):9.4f} '
                  f'{magnitude:14.4f} {magnitude - G:+9.4f}')
    worst = float(np.abs(errors).max())
    if verbose:
        print(f'  worst case {worst:.4f} m/s^2 ({worst / G * 100:.2f}%) over '
              f'{len(errors)} orientations')
    return worst


def main():
    parser = argparse.ArgumentParser(
        description='Score accelerometer calibrations over all six poses.')
    parser.add_argument('samples',
                        help='accel_<footer>.txt saved by calibrate_imu.sh')
    parser.add_argument('-c', '--calibration',
                        default=os.path.join(WS_ROOT, 'calibration.json'),
                        help='fit to evaluate (default: %(default)s)')
    parser.add_argument('-r', '--revision', action='append', default=[],
                        help='also score calibration.json at this git '
                             'revision; repeatable')
    parser.add_argument('-q', '--quiet', action='store_true',
                        help='summary table only')
    args = parser.parse_args()

    samples = np.loadtxt(args.samples, delimiter=',')[:, 1:]
    poses = bucket_poses(samples)
    if len(poses) != 6:
        print(f'warning: {len(poses)} orientations in {args.samples}, expected '
              '6. The\nworst case below only covers the poses present.')

    raw_magnitudes = [np.linalg.norm(rows.mean(axis=0))
                      for rows in poses.values()]
    print(f'{len(samples)} samples, {len(poses)} orientations')
    print(f'raw magnitude ranges {min(raw_magnitudes):.4f} to '
          f'{max(raw_magnitudes):.4f} m/s^2 '
          f'(spread {max(raw_magnitudes) - min(raw_magnitudes):.4f})')
    print('a pure scale error would read the same in every orientation; a '
          'spread means bias')

    verbose = not args.quiet
    results = [('no calibration', score('NO CALIBRATION (S=I, b=0)', np.eye(3),
                                        np.zeros(3), poses, verbose))]

    device = fit_from_device()
    if device is None:
        print('\n(camera not readable -- stop the ROS driver to include what '
              'it currently holds)')
    else:
        results.append(('on the camera',
                        score('CURRENTLY ON THE CAMERA', *device, poses,
                              verbose)))

    for ref in args.revision:
        results.append((f'{ref}:calibration.json',
                        score(f'{ref}:calibration.json', *fit_from_git(ref),
                              poses, verbose)))

    candidate = fit_from_json(json.load(open(args.calibration)))
    results.append((args.calibration,
                    score(args.calibration, *candidate, poses, verbose)))

    print('\nworst-case magnitude error, lower is better:')
    for name, worst in sorted(results, key=lambda r: r[1]):
        print(f'  {worst:7.4f} m/s^2  ({worst / G * 100:5.2f}%)  {name}')
    best = min(results, key=lambda r: r[1])
    print(f'\nbest: {best[0]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
