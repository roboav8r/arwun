# urdf/

Drop `arwun_dynamics.urdf` (from the Drake work) in this directory.

`description.launch.py` looks for `arwun_dynamics.urdf` here by default and
falls back to a warning if it is absent, so the rest of the launch still comes
up. Override with `urdf:=/abs/path/to/other.urdf`.

After adding the file:

    colcon build --packages-select arwun_description
    ros2 launch arwun_description description.launch.py

Then confirm the transform the bags need actually exists:

    ros2 run tf2_ros tf2_echo base_link camera_link
