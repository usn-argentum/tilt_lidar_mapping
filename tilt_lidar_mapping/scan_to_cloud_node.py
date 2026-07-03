#!/usr/bin/env python3
"""
scan_to_cloud_node.py

Subscribes to LDS-02 /scan (sensor_msgs/LaserScan) and /lidar_tilt/joint_states
(sensor_msgs/JointState, published by servo_sweep_node.py). Produces a
tilt-compensated 3D point cloud.

WHY NOT JUST USE TF: tf2's Buffer.lookupTransform() already does time
interpolation between buffered transforms, which sounds like a shortcut for
this. In practice, calling lookupTransform() per-point (a full LDS-02 scan
is ~360-500 points) is a synchronous Python call with real overhead, and at
the scan rates you care about this gets slow enough to matter. Instead we
keep our own small ring buffer of (time_ns, angle_rad) from the JointState
topic and do the linear interpolation manually - same math tf2 uses
internally, none of the per-call overhead.

SYNC ASSUMPTION: a single LaserScan message covers a nonzero time window
(scan.time_increment per point). Because the servo is moving continuously
during that window, treating the whole scan as "one angle" (e.g. the angle
at scan.header.stamp) smears the cloud. This node instead computes a
per-point timestamp and interpolates the tilt angle for THAT point
individually. This is the expensive-but-correct version from the original
time estimate (~4-6h item) - if it's too slow on the Jetson, the fallback
is to only sample scans near the sweep extremes where angular velocity
is near zero (removes the need for per-point interpolation, but you lose
scan density in between).

ANGLE SOURCE: subscribes to /lidar_tilt/angle_rad as std_msgs/Float32
(matches Hermes' existing message idiom - Int32/String/Float32 only, no
JointState/array types anywhere in their firmware, so this avoids being
the first thing to need micro-ROS dynamic memory config). Float32 has no
header, so each sample is timestamped on ARRIVAL at this node (self.get_clock().now()),
not on send. At USB serial latency (sub-ms to a few ms) and a multi-second
sweep period, this is well within acceptable error for this application.

Tilt axis assumed = Y (pitch), matching servo_sweep_node.py's TF broadcast.
Change ROTATION_AXIS below if your mount is different.
"""

from collections import deque
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


class TiltScanToCloud(Node):
    def __init__(self):
        super().__init__('tilt_scan_to_cloud')

        self.declare_parameter('angle_topic', '/lidar_tilt/angle_rad')
        self.declare_parameter('output_frame', 'lidar_tilt_base_link')
        self.declare_parameter('angle_buffer_seconds', 2.0)
        self.declare_parameter('max_angle_buffer_len', 500)

        angle_topic = self.get_parameter('angle_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.buffer_window_s = self.get_parameter('angle_buffer_seconds').value
        self.buffer_maxlen = self.get_parameter('max_angle_buffer_len').value

        # deque of (time_ns: int, angle_rad: float), kept sorted by arrival (== time order)
        self.angle_buffer = deque(maxlen=self.buffer_maxlen)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.angle_sub = self.create_subscription(
            Float32, angle_topic, self.angle_cb, 20
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, sensor_qos
        )
        self.cloud_pub = self.create_publisher(PointCloud2, '/tilt_lidar/points', sensor_qos)

        self.dropped_scans = 0
        self.get_logger().info(
            f'tilt_scan_to_cloud ready, waiting for /scan and {angle_topic}'
        )

    def angle_cb(self, msg: Float32):
        t_ns = self.get_clock().now().nanoseconds
        self.angle_buffer.append((t_ns, msg.data))

    def interp_angle(self, t_ns: int):
        """Linear-interpolate tilt angle at t_ns from the buffer. Returns None if out of range."""
        buf = self.angle_buffer
        if len(buf) < 2:
            return None
        # buffer is time-ordered by arrival; find bracketing pair
        if t_ns <= buf[0][0] or t_ns >= buf[-1][0]:
            # outside buffered window - clamp to nearest rather than reject,
            # scan edges will commonly fall slightly outside
            if t_ns <= buf[0][0]:
                return buf[0][1]
            return buf[-1][1]

        # linear scan is fine here - buffer is small (few hundred entries)
        lo, hi = buf[0], buf[-1]
        for i in range(len(buf) - 1):
            if buf[i][0] <= t_ns <= buf[i + 1][0]:
                lo, hi = buf[i], buf[i + 1]
                break
        t0, a0 = lo
        t1, a1 = hi
        if t1 == t0:
            return a0
        frac = (t_ns - t0) / (t1 - t0)
        return a0 + frac * (a1 - a0)

    def scan_cb(self, msg: LaserScan):
        if not self.angle_buffer:
            self.dropped_scans += 1
            if self.dropped_scans % 20 == 1:
                self.get_logger().warn(
                    'No servo angle data yet - is Hermes publishing '
                    f'/lidar_tilt/angle_rad? (dropped {self.dropped_scans} scans)'
                )
            return

        scan_t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        points = []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue

            theta = msg.angle_min + i * msg.angle_increment
            point_t_ns = scan_t_ns + int(i * msg.time_increment * 1e9)
            tilt = self.interp_angle(point_t_ns)
            if tilt is None:
                continue

            # point in the lidar's own scanning plane (z=0, plane normal = tilt axis at rest)
            x_l = r * math.cos(theta)
            y_l = r * math.sin(theta)
            z_l = 0.0

            # rotate about Y axis (pitch) by `tilt` into the base frame.
            # If your mount tilts about a different axis, swap this block.
            cos_t = math.cos(tilt)
            sin_t = math.sin(tilt)
            x = x_l * cos_t + z_l * sin_t
            y = y_l
            z = -x_l * sin_t + z_l * cos_t

            points.append((x, y, z))

        if not points:
            return

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.output_frame

        cloud = pc2.create_cloud_xyz32(header, points)
        self.cloud_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = TiltScanToCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
