#!/usr/bin/env bash
#
# Build librealsense from source with the RSUSB (libusb) backend.
#
# WHY THIS EXISTS
# ---------------
# The D435i's IMU is normally exposed to the host as a USB HID sensor, which
# Linux surfaces through the IIO subsystem. That path needs four kernel
# modules: hid-sensor-hub, hid-sensor-iio-common, hid-sensor-accel-3d and
# hid-sensor-gyro-3d. The stock JetPack kernel on this rig (5.15.x-tegra)
# ships none of them -- /sys/bus/iio/devices/ is empty and the only IIO file
# under /lib/modules is industrialio-triggered-buffer.ko, a buffer helper
# rather than a driver.
#
# The consequence is not a crash. The apt build of librealsense comes up,
# streams color and depth perfectly, and reports the IMU part number
# (BMI055) because it reads that from the camera's own descriptor -- but it
# enumerates only two sensors, "Stereo Module" and "RGB Camera". There is no
# "Motion Module", so realsense2_camera logs
#
#     No HID info provided, IMU is disabled
#     HID Motion Sensor Failure (continuing as partial device)
#
# and /camera/imu never publishes. `ros2 bag record` does not treat a
# never-publishing topic as an error, so this shows up only as a missing
# topic in `ros2 bag info` after a collection run -- exactly the failure mode
# arwun_bringup/config/record_params.yaml warns about in its header.
#
# -DFORCE_RSUSB_BACKEND=true swaps the V4L2/HID backend for one that talks to
# the camera over libusb, reading the IMU directly off the USB endpoints and
# bypassing the kernel HID/IIO stack entirely. No kernel rebuild required.
#
# SCOPE
# -----
# Installs to a workspace-local prefix (vendor/librealsense) and does NOT
# touch the apt-installed ros-humble-librealsense2 under /opt/ros/humble.
# Selecting this build at runtime is a separate step -- see the notes at the
# bottom of this file.
#
# Requires: udev rules installed (see scripts/install_realsense_udev.sh),
# otherwise libusb cannot open the device as a non-root user.

set -euo pipefail

# Pin to the version matching the apt ros-humble-realsense2-camera ABI. The
# ROS node links librealsense2.so.2.58; a source build of this tag produces
# that same soname, so it is a drop-in replacement. Bumping this without
# bumping realsense2_camera to match will break that.
LIBREALSENSE_VERSION="${LIBREALSENSE_VERSION:-v2.58.3}"

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="${WS_ROOT}/vendor/src/librealsense"
PREFIX="${WS_ROOT}/vendor/librealsense"
JOBS="${JOBS:-$(( $(nproc) - 1 ))}"

echo "librealsense ${LIBREALSENSE_VERSION} -> ${PREFIX}  (jobs: ${JOBS})"

if [[ ! -d "${SRC_DIR}/.git" ]]; then
    mkdir -p "$(dirname "${SRC_DIR}")"
    git clone --depth 1 --branch "${LIBREALSENSE_VERSION}" \
        https://github.com/IntelRealSense/librealsense.git "${SRC_DIR}"
fi

# Graphical examples and the GLSL extension library pull in GL/GTK and roughly
# double the build time. The ROS node links only librealsense2, so both are
# off. BUILD_TOOLS stays on for rs-enumerate-devices, which is how you confirm
# the Motion Module is actually there.
#
# Python bindings are on solely for scripts/calibrate_imu.sh -- Intel's IMU
# calibration tool is a Python script. They must be built here rather than
# installed from apt: an apt pyrealsense2 would use the V4L2/HID backend and
# would not see the Motion Module on this kernel, which is the whole problem.
# Note that BUILD_PYTHON_BINDINGS does not reliably stage the module into
# CMAKE_INSTALL_PREFIX, so it may only appear under build/Release --
# calibrate_imu.sh looks in both.
#
# BUILD_WITH_CUDA=false is the one setting here worth revisiting. Profiling on
# 2026-08-12 (see record_params.yaml) showed the streaming ceiling on this rig
# is CPU, not USB: align_depth roughly doubles node CPU and pegs a full core at
# 1280x720x30. That alignment is running on the CPU of a Jetson with an idle
# GPU. Turning CUDA on should raise the ceiling, at the cost of a much longer
# build and a CUDA-toolkit dependency. Not tested -- the rig records at
# 640x480x30, which fits comfortably either way.
cmake -S "${SRC_DIR}" -B "${SRC_DIR}/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_SHARED_LIBS=true \
    -DBUILD_TOOLS=true \
    -DBUILD_EXAMPLES=false \
    -DBUILD_GRAPHICAL_EXAMPLES=false \
    -DBUILD_GLSL_EXTENSIONS=false \
    -DBUILD_PYTHON_BINDINGS=true \
    -DBUILD_UNIT_TESTS=false \
    -DBUILD_WITH_CUDA=false \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}"

cmake --build "${SRC_DIR}/build" -j "${JOBS}"
cmake --install "${SRC_DIR}/build"

cat <<EOF

Done. Verify the IMU is now visible:

    ${PREFIX}/bin/rs-enumerate-devices | grep -A2 'Motion Module'

You should get a "Stream Profiles supported by Motion Module" section listing
GYRO and ACCEL profiles. If it is still absent, the udev rules are the usual
cause -- run scripts/install_realsense_udev.sh.

To make the ROS node use this build instead of the apt one, put its lib dir
ahead of /opt/ros/humble on the loader path:

    export LD_LIBRARY_PATH=${PREFIX}/lib:\$LD_LIBRARY_PATH

EOF
