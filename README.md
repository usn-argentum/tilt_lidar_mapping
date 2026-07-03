# tilt_lidar_mapping

LDS-02 `/scan` + a tilt angle stream -> tilt-compensated `PointCloud2` on
`/tilt_lidar/points`. Feed into RTAB-Map.

## Status

Tested on real hardware with servo simulated: `/scan` ~9.8Hz, angle stream
clean, cloud correct against a real corner in RViz. Not tested with a real
servo yet.

## For Hermes

- Topic: `/lidar_tilt/angle_rad`
- Type: `std_msgs/Float32`
- Value: tilt angle in radians, commanded angle is fine (no encoder needed)
- Rate: 30-50Hz minimum
- Domain ID: match Hermes' current build (should be 0, not the old `8`)
- New timer: bump `rclc_executor_init`'s `num_handles` by one

`servo_sweep_node.py` is a test-only stand-in for this topic. Remove it
from the launch file once Hermes publishes for real; do not run both.

## Assumptions

- No servo feedback, commanded angle only.
- `ld08_driver` must already publish `/scan`, not included here.
- Tilt axis = Y. Change `scan_cb` in `scan_to_cloud_node.py` if different.
- `output_frame` defaults to `base_scan`, matches `ld08_driver`.
- Servo command-to-position lag not compensated.

## Running

```bash
colcon build --packages-select tilt_lidar_mapping
source install/setup.bash

ros2 launch ld08_driver ld08.launch.py       # separate terminal
ros2 launch tilt_lidar_mapping tilt_lidar.launch.py simulate:=true
```

Once Hermes publishes the real topic, run `scan_to_cloud_node` alone
instead of the full launch file.

## Testing

- `ros2 topic echo /lidar_tilt/angle_rad` - smooth, no jumps
- `ros2 topic hz /tilt_lidar/points` - tracks scan rate
- RViz2: PointCloud2 on `/tilt_lidar/points`, Fixed Frame `base_scan`,
  Reliability Best Effort, Decay Time ~3s. Point at a flat wall/corner -
  should look flat and coherent, not smeared or doubled.

## Open items

- Repeat RViz check with a real angle stream from Hermes.
- Servo lag compensation, if precision isn't good enough.
- Confirm tilt axis against actual mount.
