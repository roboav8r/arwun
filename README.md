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
| `arwun_teleop` | `ament_python` | `record_controller`, a joystick-driven rosbag2 recorder, and `record_indicator`, its LED/banner mirror. |

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

> **`ros-humble-joy-linux` is easy to miss, and it fails late.** As of
> 2026-08-15 this rig had `ros-humble-joy` installed but not
> `ros-humble-joy-linux` — different package, different executable. They are not
> interchangeable: `joy` provides an SDL-based `joy_node` taking `device_id`,
> while `record.launch.py` asks for `joy_linux/joy_linux_node` taking `dev`.
> Without it the launch aborts on the joystick node, which is why every bag
> recorded before that date was taken with `joy:=false`. Check with:
>
> ```bash
> ros2 pkg executables joy_linux    # expect: joy_linux joy_linux_node
> ```

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

### The recording indicator

`record_indicator` comes up with the launch and mirrors
`/arwun/recording_status` onto a terminal banner and, optionally, an LED on the
40-pin header. It **blinks rather than sitting solid**, so a frozen indicator
and an active one do not look alike.

The LED is off by default (`led_pin: 0`) because driving a header pin that has
something else wired to it is worse than having no LED. To use one, wire an LED
anode through a 220-330 Ω resistor to your chosen pin and the cathode to any
ground pin, then set `record_indicator.led_pin` in the params YAML to the
**physical** pin number:

```bash
ros2 launch arwun_bringup record.launch.py   # led_pin comes from the YAML
```

Board pin 7 is a good default: a plain GPIO with no pinmux conflict on the Orin
Nano, verified drivable from an ordinary user account. No `sudo` is involved —
membership of the `gpio` group grants `/dev/gpiochip*`. The header is 3.3 V
logic and **the pins are not 5 V tolerant**.

> **The indicator cannot live on the controller, and this is a kernel limit.**
> The obvious home for it is the 8BitDo's own player LEDs and rumble, already in
> the operator's hand. `hid_nintendo` does not exist in this L4T build
> (`modinfo hid_nintendo` → module not found), so the pad binds to `hid-generic`
> and enumerates with neither `EV_LED` nor `EV_FF` in its capability bits
> (`EV=10001b`: SYN, KEY, ABS, MSC, REP). No userspace program can light a pad
> LED the kernel does not expose. This is the same shape of problem as the
> missing `hid_sensor_*` modules that forced the source-built librealsense, and
> the same fix applies if it ever matters enough: build `hid-nintendo` out of
> tree. Note that even then, 8BitDo's Switch-mode emulation is not guaranteed to
> implement the rumble/LED subcommands.

**A dark LED does not prove nothing is being written.** The indicator is a
separate process from the recorder on purpose — an indicator that crashes must
not be able to take a collection run down with it — so the authoritative check
is still the recorder itself:

```bash
ros2 topic echo /arwun/recording_status    # what the recorder believes
du -sh ~/arwun_bags/arwun_*                # what is actually growing on disk
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

## Offloading bags

```bash
./scripts/offload_bags.sh --dry-run        # what would move
./scripts/offload_bags.sh                  # copy to the configured destination
./scripts/offload_bags.sh --delete-local   # copy, verify by checksum, free disk
```

**There is no default destination — set one per rig before first use.** This
repository is public, so a committed default of the form `user@10.x.y.z` would
publish an account name and an internal network address to everyone who clones
it, to save one line of setup. Precedence is `--dest`, then `ARWUN_DEST`, then
git config:

```bash
git config --local arwun.offloadDest 'user@workstation:~/arwun_bags/'
```

The git-config form is per-clone and stays out of version control, which is
what you want on a rig. Running with none of the three set prints those options
and exits rather than guessing.

The script only transfers directories that contain `metadata.yaml`. A bag
without one is either still being written or was killed unfinalized, and
copying it yields a truncated file that *looks* complete. Unfinalized
directories are listed rather than silently skipped, since those are the ones
needing `ros2 bag reindex`. `--delete-local` re-verifies with a `--checksum`
pass and deletes nothing if any file still differs.

**The link is wifi and it is the slow part.** This Jetson has no wired
interface up. The payload is produced at ~79 MB/s against a link that will not
sustain anything close to that, so budget offload time in multiples of record
time, not fractions of it — a full disk is a multi-hour transfer. Recording and
offloading at once will cost you frames.

One-time key setup (needs the remote password once):

```bash
ssh-copy-id user@workstation
```

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

## IMU calibration

Done once per camera and stored on the camera's EEPROM, so it follows the D435i
between rigs and survives reflashing the Jetson. `calibration.json` at the
workspace root is the committed record of the fit currently on this camera.

```bash
./scripts/calibrate_imu.sh --tolerance 1.2      # interactive, six poses
./scripts/score_imu_calibration.py accel_2.txt  # how good is the result?
./scripts/check_imu_bias.py                     # 30s live check, needs the launch up
```

Three things worth knowing before redoing it:

- **Brace the camera against a right angle** on a flat table for each of the six
  poses. The fit absorbs pose tilt into scale and alignment, and freehand poses
  are the difference between a 5% and a 1.7% result on this camera.
- **`--tolerance` is not optional.** Upstream gates each pose on a 0.866 m/s²
  radius that has to cover magnitude error *and* orientation error. This camera
  spends most of that budget before any tilt, leaving an acceptance cone too
  small to hit by hand. `calibrate_imu.sh` widens it; the header explains the
  arithmetic and why it doesn't affect the fit itself.
- **Judge the result over all six poses, not one.** The residual error is bias-
  dominated, so the magnitude at rest depends on which way the camera points —
  raw, it ranges from 8.95 to 10.31 m/s² by pose. Two readings at different
  orientations are not comparable, and reading a single one made a better
  calibration look like a regression during the 2026-08-14 session.
  `score_imu_calibration.py` answers the real question: worst case across every
  orientation.

Say Y to saving raw samples when the tool offers — `accel_<footer>.txt` and
`gyro_<footer>.txt` let you refit and re-write without redoing the poses:

```bash
./scripts/calibrate_imu.sh -i accel_2.txt gyro_2.txt --tolerance 1.2
```

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

Current as of 2026-08-15.

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
- **The whole record path runs end to end.** First real bag taken 2026-08-15:
  121 s, 9.3 GiB, 60405 messages, every configured topic present except `/tf`
  (expected — see above). Toggle start, toggle stop, and `metadata.yaml`
  written on the normal shutdown path, confirmed with `ros2 bag info`. Sustained
  **78.5 MB/s**, which matches the 79 MB/s the config comment predicts. Colour
  arrived 20 frames short of its `camera_info` over the run (3617 vs 3637,
  0.55%) — the same rosbag2 write-load drop noted below, milder at this
  duration. IMU held 199.6 Hz.
- **Bags offload to the workstation.** `scripts/offload_bags.sh` rsyncs
  finalized bags over ssh, skipping any directory without `metadata.yaml`, and
  re-verifies by checksum before `--delete-local` frees anything. Key
  authorized 2026-08-15; destination is per-clone config, not committed.
- **`record_indicator` mirrors recording state** to a terminal banner and an
  optional header LED, verified on GPIO board pin 7 including the teardown path
  (pin driven low and released on both Ctrl-C and the SIGTERM that `ros2 launch`
  sends). It cannot use the controller's own LEDs; see the kernel limitation in
  the indicator section.
- **The accelerometer is calibrated**, on the camera's own EEPROM, so it
  survives reflashing this workspace and follows the camera between rigs.
  Worst-case magnitude error across all six poses is **0.167 m/s² (1.70%)**,
  down from 0.861 (8.78%) uncalibrated. `calibration.json` at the workspace
  root is the committed record and matches what the device holds. The
  gyroscope half is a separate story — see below.

### Not yet done

- [ ] **`arwun_dynamics.urdf` not added** (package is scaffolded for it).
      Bags recorded now carry no camera-to-base transform.
- [ ] **The gyroscope bias correction is not being applied.** Confirmed against
      live data, not just inferred from the read-back: at rest `/camera/imu`
      shows `[-2.1e-3, -2.7e-3, +1.0e-3]` rad/s against a fitted bias of
      `[-2.44e-3, -3.12e-3, +0.93e-3]` — 86-113% of it, i.e. essentially
      uncorrected. The read-back explains why: the device holds a bias smaller
      than what was written by almost exactly 180/pi, the signature of a
      deg/s-vs-rad/s mismatch between `rs-imu-calibration.py`'s write path
      (which writes the bias through unconverted, line 695) and librealsense's
      read path. Left alone deliberately — the residual is ~0.12 deg/s, which
      is unremarkable for this sensor and which VIO estimators carry as an
      online state anyway. Subtracting the fitted bias in post is available if
      something needs it.
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
- [ ] **No field data collected yet** — `~/arwun_bags` is empty again. The
      2026-08-15 pipeline check produced a 9.3 GiB bench bag pointed at a desk,
      which proved the path and was then deleted; it was never data.
- [ ] **`ros-humble-joy-linux` is not installed on this rig**, so the launch
      needs `joy:=false` until it is. See the setup warning.
- [ ] **No `hid_nintendo` on this kernel**, so no controller-side LED or rumble
      indicator is possible without an out-of-tree module build.

## License

MIT — see [LICENSE](LICENSE).
