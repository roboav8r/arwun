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
colour + depth + IMU + TF:

```
/camera/color/image_raw              /camera/color/camera_info
/camera/depth/image_rect_raw         /camera/depth/camera_info
/camera/aligned_depth_to_color/image_raw
/camera/aligned_depth_to_color/camera_info
/camera/imu                          /camera/extrinsics/depth_to_color
/joy                                 /arwun/recording_status
/tf                                  /tf_static
```

The IR stereo pair is deliberately excluded to save USB and disk bandwidth;
enable `enable_infra1`/`enable_infra2` and add the topics if you want to redo
stereo or VIO offline.

> **Verify these names on first use.** realsense2_camera has changed its topic
> namespacing across releases. Plug the D435i in, launch, and run
> `ros2 topic list | grep camera`, then reconcile the list. `ros2 bag record`
> does *not* error on a name that never publishes — it silently omits it, so a
> typo shows up only as a missing topic in `ros2 bag info` afterwards. Check the
> first bag of a session before trusting the rest.

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

## Status / not yet done

- [ ] `arwun_dynamics.urdf` not yet added (package is scaffolded for it)
- [ ] Camera topic names unverified against real hardware
- [ ] No motors, microcontroller, or drive teleop yet

## License

MIT — see [LICENSE](LICENSE).
