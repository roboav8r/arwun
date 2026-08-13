#!/usr/bin/env bash
#
# Run Intel's D435i IMU calibration and (optionally) write the result to the
# camera's EEPROM.
#
# WHY: as shipped, this camera's accelerometer reads ~8.95 m/s^2 at rest
# against 9.807 expected -- a systematic -8.8% scale error, not noise. The
# driver reports "IMU Calibration is not available, default intrinsic and
# extrinsic will be used". Recorded bags carry raw values so this is
# correctable in post, but it should not go into VIO/SLAM uncorrected.
#
# The result is written to the camera's own EEPROM, not to this workspace, so
# it survives reflashing the Jetson and follows the camera between rigs. You
# only need to do this once per camera.
#
# THIS IS AN INTERACTIVE, PHYSICAL PROCEDURE. The script cannot be automated:
# it needs the camera held still in six different orientations and prompts you
# between each one.
#
#   1. Mounting screw pointing down, device facing out
#   2. Mounting screw pointing left, device facing out
#   3. Mounting screw pointing up, device facing out
#   4. Mounting screw pointing right, device facing out
#   5. Viewing direction facing down
#   6. Viewing direction facing up
#
# Hold each pose steady -- the script averages accelerometer samples per pose
# and a drifting hand widens the residuals. A flat table and a right-angled
# object to brace against gives noticeably better numbers than freehand.
#
# At the end it asks two questions: whether to save the raw samples (useful if
# you want to re-derive the fit later with -i) and whether to write to the
# camera. Nothing is written until you answer Y.
#
# --tolerance: WHY THIS OPTION EXISTS
# -----------------------------------
# Upstream gates each pose on
#
#     np.linalg.norm(measured_accel - bucket_target) < max_norm
#
# with max_norm = norm([0.5,0.5,0.5]) = 0.866 m/s^2, comparing the RAW vector
# against a nominal target like [0,0,-9.807]. That radius has to cover BOTH the
# orientation error and the magnitude error. On a correctly-scaled camera the
# magnitude term is ~0 and the whole 0.866 goes to orientation, giving about
# 5 degrees of freedom to hold the pose.
#
# This camera reads 8.949 m/s^2 instead of 9.807. That burns 0.858 of the 0.866
# on magnitude alone, before any tilt, leaving an acceptance cone of roughly
# 0.7 degrees -- unreachable by hand. The tool sits in Status.rotate forever,
# printing near-zero direction error and never advancing. The miscalibration
# blocks its own calibration.
#
# Widening the radius is safe because it only decides "which of six poses is
# this", and the six bucket targets are at least g*sqrt(2) = 13.87 m/s^2 apart,
# so anything below ~6.9 keeps them unambiguous. The 1.5 default restores about
# 7.5 degrees of tolerance -- slightly more than upstream intends for a healthy
# camera, and still 4.6x below the point where two poses could overlap.
#
# This does NOT loosen the fit itself; it only decides which samples belong to
# which pose. The least-squares solve downstream is untouched.
#
# Note the -i refit path uses the SAME 0.866 radius (line 597), so saved raw
# data does not escape this on its own -- pass --tolerance there too.
#
# Usage:
#     ./scripts/calibrate_imu.sh              # interactive, this camera
#     ./scripts/calibrate_imu.sh -g           # also plot norms (needs a display)
#     ./scripts/calibrate_imu.sh -i accel.txt gyro.txt   # refit saved samples
#     ./scripts/calibrate_imu.sh --tolerance 0.866       # upstream behaviour
#     ./scripts/calibrate_imu.sh --tolerance 2.0         # looser still

set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LRS_SRC="${WS_ROOT}/vendor/src/librealsense"
SCRIPT="${LRS_SRC}/tools/rs-imu-calibration/rs-imu-calibration.py"

if [[ ! -f "${SCRIPT}" ]]; then
    echo "error: ${SCRIPT} not found -- run scripts/build_librealsense.sh first." >&2
    exit 1
fi

# Pull --tolerance out of the argument list; everything else is forwarded to
# the Python tool untouched.
TOLERANCE="1.5"
ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --tolerance)
            [[ $# -ge 2 ]] || { echo "error: --tolerance needs a value." >&2; exit 1; }
            TOLERANCE="$2"; shift 2 ;;
        --tolerance=*)
            TOLERANCE="${1#*=}"; shift ;;
        *)
            ARGS+=("$1"); shift ;;
    esac
done

if ! awk "BEGIN{exit !(${TOLERANCE} > 0 && ${TOLERANCE} < 6.9)}" 2>/dev/null; then
    echo "error: --tolerance must be between 0 and 6.9 (got '${TOLERANCE}')." >&2
    echo "Above 6.9 the six pose targets overlap and samples land in the wrong" >&2
    echo "bucket, which silently corrupts the fit rather than failing." >&2
    exit 1
fi

# The radius is spelled as norm([x,x,x]) in two places (the interactive state
# machine and the -i refit path), so solve x = tolerance/sqrt(3) and rewrite
# both. Patch a COPY: vendor/src/librealsense is a pinned upstream checkout and
# build_librealsense.sh will re-clone it.
PATCHED_DIR="$(mktemp -d)"
trap 'rm -rf "${PATCHED_DIR}"' EXIT
PATCHED="${PATCHED_DIR}/rs-imu-calibration.py"
COMPONENT="$(awk "BEGIN{printf \"%.9f\", ${TOLERANCE}/sqrt(3)}")"

sed "s/np\.array(\[0\.5, 0\.5, 0\.5\])/np.array([${COMPONENT}, ${COMPONENT}, ${COMPONENT}])/g" \
    "${SCRIPT}" > "${PATCHED}"

SUBS=$(grep -c "${COMPONENT}" "${PATCHED}")
if [[ "${SUBS}" -ne 2 ]]; then
    echo "error: expected to rewrite 2 max_norm definitions, rewrote ${SUBS}." >&2
    echo "Upstream rs-imu-calibration.py has changed -- re-check lines 93 and 597." >&2
    exit 1
fi
set -- "${ARGS[@]+"${ARGS[@]}"}"

# The calibration needs pyrealsense2 built against the RSUSB-backend tree. The
# apt pyrealsense2 (if it were installed) would use the V4L2/HID backend and
# would not see the Motion Module at all on this kernel -- same root cause as
# the missing /camera/imu. Prefer the installed prefix, fall back to the build
# tree, since BUILD_PYTHON_BINDINGS does not always stage into the prefix.
PY_MODULE_DIR=""
for candidate in \
    "$(find "${WS_ROOT}/vendor/librealsense" -name 'pyrealsense2*.so' -printf '%h\n' 2>/dev/null | head -1)" \
    "${LRS_SRC}/build/Release"; do
    if [[ -n "${candidate}" ]] && compgen -G "${candidate}/pyrealsense2*.so" >/dev/null; then
        PY_MODULE_DIR="${candidate}"; break
    fi
done

if [[ -z "${PY_MODULE_DIR}" ]]; then
    echo "error: pyrealsense2 not found. Rebuild with -DBUILD_PYTHON_BINDINGS=true:" >&2
    echo "  cmake -S ${LRS_SRC} -B ${LRS_SRC}/build -DBUILD_PYTHON_BINDINGS=true" >&2
    echo "  cmake --build ${LRS_SRC}/build -j\$(nproc)" >&2
    exit 1
fi

# librealsense claims the USB interface exclusively. If the ROS driver is up it
# owns the camera and this script fails with a confusing enumeration error
# rather than a clear "device busy".
if pgrep -f 'realsense2_camera_node' >/dev/null; then
    echo "error: realsense2_camera_node is running and holds the camera." >&2
    echo "Stop your launch (Ctrl-C) and re-run this script." >&2
    exit 1
fi

export LD_LIBRARY_PATH="${WS_ROOT}/vendor/librealsense/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${PY_MODULE_DIR}:${PYTHONPATH:-}"

echo "pyrealsense2: ${PY_MODULE_DIR}"
echo "pose tolerance: ${TOLERANCE} m/s^2 (upstream default 0.866)"
echo "Make sure the camera is on a cable long enough to rotate through all six"
echo "poses without unplugging -- a mid-run disconnect loses the whole run."
echo

# Not exec: that would replace this shell and discard the EXIT trap, leaking
# the patched copy into /tmp on every run.
python3 "${PATCHED}" "$@"
exit $?
