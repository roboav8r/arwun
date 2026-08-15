# arwun

ROS 2 Humble workspace for the Arwun rover.

This first increment is a **field data-collection rig**: a Jetson Orin Nano
carrying an Intel RealSense D435i, driven by a Bluetooth 8BitDo controller,
writing rosbag2 bags that can be replayed offline. No motors or microcontroller
are in the loop yet.

## Packages

| Package | Build type | What it does |
| --- | --- | --- |
| `arwun_description` | `ament_cmake` | URDF, meshes, and `robot_state_publisher` bringup. Supplies `/tf` and `/tf_static` so recorded bags carry the camera-to-base transform. |
| `arwun_bringup` | `ament_cmake` | `record.launch.py` and the parameter YAML for the whole rig. |
| `arwun_teleop` | `ament_python` | `record_controller`, a joystick-driven rosbag2 recorder. |

## Hardware

- Jetson Orin Nano 8GB, JetPack 6 / L4T R36.5, Ubuntu 22.04, ROS 2 Humble
- Intel RealSense D435i on USB 3
- 8BitDo controller over Bluetooth, in Switch/Pro Controller mode
  (enumerates as `Pro Controller` on `/dev/input/js0`, 16 buttons / 6 axes)

## Setup

### 1. System dependencies

Neither driver ships with the base Humble install:

```bash
sudo apt update
sudo apt install -y ros-humble-realsense2-camera ros-humble-joy-linux
```

Optional, only if you convert the description to xacro:

```bash
sudo apt install -y ros-humble-xacro
```

### 2. Add the URDF

`arwun_description/urdf/` is a scaffold. Drop `arwun_dynamics.urdf` in it —
`description.launch.py` looks for that filename by default. Until it's there,
the launch still comes up but logs a warning and publishes no `/tf`, so **bags
recorded before the URDF lands will not carry the camera-to-base transform.**

### 3. Build

```bash
cd ~/arwun_ws
colcon build --symlink-install
source install/setup.bash
```

## Recording

```bash
ros2 launch arwun_bringup record.launch.py
```

Everything comes up **idle** — nothing is written until you press the button, so
you can sanity-check topics before burning disk.

- **Toggle button: `buttons[7]`** on the 8BitDo (right trigger / ZR in
  Switch mode). Press once to start, press again to stop.
- Recording state is mirrored on `/arwun/recording_status`
  (`std_msgs/Bool`, transient-local, so late subscribers see current state).
- Bags land in `~/arwun_bags/arwun_<ISO8601>/`, e.g.
  `arwun_20260806T143022-0700`. The timestamp is ISO 8601 basic format with a
  UTC offset — no colons (filesystem-safe) and unambiguous across DST.

Watch state from another terminal:

```bash
ros2 topic echo /arwun/recording_status
```

### Launch overrides

```bash
ros2 launch arwun_bringup record.launch.py output_dir:=/media/ssd/bags
ros2 launch arwun_bringup record.launch.py joy_dev:=/dev/input/js1
ros2 launch arwun_bringup record.launch.py camera:=false   # bench-test the toggle
ros2 launch arwun_bringup record.launch.py urdf:=/abs/path/to/other.urdf
```

### Recorded topics

Configured in `arwun_bringup/config/record_params.yaml`. The default set is
colour + depth + the IR stereo pair + IMU + TF:

```
/camera/color/image_raw              /camera/color/camera_info
/camera/depth/image_rect_raw         /camera/depth/camera_info
/camera/aligned_depth_to_color/image_raw
/camera/aligned_depth_to_color/camera_info
/camera/infra1/image_rect_raw        /camera/infra1/camera_info
/camera/infra2/image_rect_raw        /camera/infra2/camera_info
/camera/extrinsics/depth_to_infra1   /camera/extrinsics/depth_to_infra2
/camera/imu                          /camera/extrinsics/depth_to_color
/joy                                 /arwun/recording_status
/tf                                  /tf_static
```

Verified against a real bag on 2026-08-14: every name above lands except `/joy`
(absent only because that run was launched with `joy:=false`) and `/tf`, which
is expected to be missing until the rig grows actuated joints.

**The IR pair is recorded even though nothing consumes it yet.** Calibration,
extrinsics and noise models can all be sorted out after a collection run; a
stream that was never recorded cannot. Those frames are global shutter, unlike
the rolling-shutter colour stream, so they are what any later VIO or offline
stereo work would want. The cost is ~30% more disk (see below) for no
measurable CPU change. Note that as configured they carry the projector's dot
pattern, which is good for depth and bad for feature tracking — the tradeoff
and the two ways out are written up in `record_params.yaml`.

> **Verify these names on first use.** realsense2_camera has changed its topic
> namespacing across releases. Plug the D435i in, launch, and run
> `ros2 topic list | grep camera`, then reconcile the list. `ros2 bag record`
> does *not* error on a name that never publishes — it silently omits it, so a
> typo shows up only as a missing topic in `ros2 bag info` afterwards. Check the
> first bag of a session before trusting the rest.
>
> The same silence applies to the streams themselves. Leaving
> `depth_module.infra_profile` at its 848x480 default while depth runs at
> 640x480 stops depth publishing entirely, with every stream still logged as
> opening normally — keep the two profiles equal.

## Why the recorder uses SIGINT

`record_controller` runs `ros2 bag record` as a subprocess in its own process
group and stops it with **SIGINT**, then waits up to 15 s. rosbag2 writes
`metadata.yaml` on its normal shutdown path; a recorder killed with SIGKILL
leaves a `.db3` with no metadata, and `ros2 bag play` won't open it without a
hand-written one.

The node escalates SIGINT → SIGTERM → SIGKILL only if the recorder hangs, and
logs loudly when it has to. If you ever end up with a bag missing its metadata:

```bash
ros2 bag reindex ~/arwun_bags/arwun_<stamp>
```

Button edges are debounced (`debounce_sec`, default 0.4 s) and only rising
edges count, so a bouncy trigger or a repeated Bluetooth packet can't
start-stop-start a take.

## Finding a different button

The toggle index was probed live off `/dev/input/js0`. To find another:

```bash
ros2 topic echo /joy    # with the launch running
```

or, without ROS running, read the joystick device directly and press the button
you want. Set `record_controller.toggle_button` in the params YAML to the index
that lights up.

## Repository layout

```
arwun_ws/
├── README.md
├── LICENSE
├── .gitignore
└── src/
    ├── arwun_description/   urdf/  meshes/  launch/  rviz/
    ├── arwun_bringup/       launch/record.launch.py  config/record_params.yaml
    └── arwun_teleop/        arwun_teleop/record_controller.py  config/
```

`build/`, `install/`, `log/`, and bag output directories are gitignored.

## Status

Current as of 2026-08-14.

### Working and verified on hardware

- **Colour + depth + aligned depth stream** at the configured 640x480x30. Five
  profiles were benchmarked on a real USB 3 link; the numbers and the reasoning
  live in `arwun_bringup/config/record_params.yaml`. The binding constraint is
  CPU (`align_depth`), not bus bandwidth.
- **`/camera/imu` publishes at 200 Hz.** This needs the source-built
  librealsense from `scripts/build_librealsense.sh` — the apt build enumerates
  no Motion Module on this kernel and the topic silently never appears.
  Confirmed over a 310 s recording: 61603 IMU messages, 198.5 Hz.
- **Topic names check out**, including the IR pair added on 2026-08-14 — the
  recorded list was verified against a real bag rather than just
  `ros2 topic list`. One expected absence: `/tf` will not appear in a bag until
  the rig grows actuated joints — the transform tree is all-fixed, so it goes
  out on `/tf_static`.
- **The global-shutter IR stereo pair is recorded** at 640x480x30, ~29.4 Hz
  each, for the sake of later VIO or offline stereo work. Requires
  `depth_module.infra_profile` to match `depth_profile` — see the warning in
  the recorded-topics section, which cost a debugging round to find.
- **The IMU calibration reached the camera's EEPROM and is being applied.** The
  accel fit in `calibration.json` reads back off the device
  (`rs-enumerate-devices -c`, sensitivity diag `1.017 / 1.030 / 1.008`) and
  visibly changes what `/camera/imu` publishes, so it survives reflashing this
  workspace and follows the camera between rigs. It is not yet *correct* —
  see the two calibration items below.

### Not yet done

- [ ] **`arwun_dynamics.urdf` not added** (package is scaffolded for it).
      Bags recorded now carry no camera-to-base transform.
- [ ] **The accelerometer calibration overcorrects.** Measured 2026-08-14 over
      6000 stationary samples: `/camera/imu` reports a magnitude of
      **10.117 +/- 0.011 m/s^2**, i.e. +3.2% against 9.807, where before
      calibration it read 8.949 (-8.8%). Smaller in absolute terms but the
      wrong sign, and the residual now has a bias component
      (`calibration.json` bias norm is 0.65 m/s^2), so unlike the original
      pure scale error it will read differently at different orientations.
      One stationary pose cannot separate scale from bias — characterising it
      means measuring the magnitude at several orientations.
      Likely cause: the six poses were held freehand. The fit absorbs pose tilt
      into scale and alignment, and `--tolerance 1.5` (needed to get the tool
      to advance at all) widens how far off a pose can be. A re-run braced
      against a right-angled object on a flat table is the first thing to try.
- [ ] **The gyroscope bias correction is not being applied.** Confirmed against
      live data on 2026-08-14, not just inferred from the read-back. At rest
      `/camera/imu` still shows `[-1.6e-3, -2.5e-3, +1.2e-3]` rad/s against a
      fitted bias of `[-2.17e-3, -3.07e-3, +0.97e-3]` — 74-123% of it, i.e.
      substantially uncorrected (the spread is bias instability between runs).
      That is ~660 deg/hour of yaw drift. The read-back explains why: the
      device holds `[-3.8e-5, -5.4e-5, 1.7e-5]`, smaller than what was written
      by almost exactly 180/pi, the signature of a deg/s-vs-rad/s mismatch
      between `rs-imu-calibration.py`'s write path (which writes the bias
      through unconverted, line 695) and librealsense's read path.
      Consequential for VIO/SLAM, not for recording — bags carry raw values, so
      subtracting the fitted bias in post is a valid workaround.
- [ ] **No field storage plan, and this is now the tightest constraint on the
      rig.** With the IR pair recorded the payload measures ~79 MB/s
      (~266 GiB/hour), against 144 GB free on a 233 GB disk — about **32
      minutes** of continuous recording. External media is a prerequisite for a
      real field day, not an upgrade. Dropping `enable_infra1/2` buys back
      roughly a quarter of the bandwidth if endurance matters more.
- [ ] **rosbag2 drops ~1.9% of colour frames under write load** (9106 images
      against 9282 `camera_info` over the same interval). The camera is not
      dropping them; write throughput is the lever if it matters.
- [ ] **No motors, microcontroller, or drive teleop yet.**
- [ ] **No field data collected yet** — `~/arwun_bags` is still empty.

## License

MIT — see [LICENSE](LICENSE).
