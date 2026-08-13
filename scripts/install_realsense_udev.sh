#!/usr/bin/env bash
#
# Install the RealSense libusb udev rules. Needs root, so run it yourself:
#
#     ./scripts/install_realsense_udev.sh
#
# WHY: with -DFORCE_RSUSB_BACKEND=true librealsense opens the camera through
# libusb rather than V4L2. libusb needs read/write on the raw USB device node
# under /dev/bus/usb, and the default permissions there are root-only. Without
# these rules the source build enumerates the camera as a non-functional
# device (or not at all) for any non-root user, which looks identical to the
# IMU problem it was meant to fix.
#
# The relevant line for the D435i (USB id 8086:0b3a) is:
#
#     SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", \
#         MODE:="0666", GROUP:="plugdev"
#
# The file also carries KERNEL=="iio*" and DRIVER=="hid_sensor*" rules. Those
# are no-ops on this rig -- there is no IIO device to match, which is the very
# reason for the libusb backend -- but they are harmless and left in place so
# the file stays byte-identical to upstream.
#
# The apt package that would normally ship these (librealsense2-udev-rules) is
# not published in the ROS 2 apt repo, only in Intel's own, which is why this
# is a manual step.

set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RULES_SRC="${WS_ROOT}/vendor/src/librealsense/config/99-realsense-libusb.rules"
RULES_DST="/etc/udev/rules.d/99-realsense-libusb.rules"

if [[ ! -f "${RULES_SRC}" ]]; then
    echo "error: ${RULES_SRC} not found." >&2
    echo "Run scripts/build_librealsense.sh first -- it clones the source tree." >&2
    exit 1
fi

echo "Installing ${RULES_SRC}"
echo "        -> ${RULES_DST}"
sudo install -m 0644 "${RULES_SRC}" "${RULES_DST}"

sudo udevadm control --reload-rules
sudo udevadm trigger

echo
echo "Installed. Now UNPLUG AND REPLUG the camera -- udevadm trigger does not"
echo "re-apply MODE/GROUP to a device node that already exists, so an attached"
echo "camera keeps its old root-only permissions until it re-enumerates."
