#!/usr/bin/env python3
"""
servo_sweep_node.py

Drives the LDS-02 tilt servo back and forth via a PCA9685 I2C PWM board
(NOT raw Jetson GPIO - software PWM jitter on a non-RT OS corrupts the
angle timestamp we need for scan sync).

Publishes the COMMANDED angle as std_msgs/Float32 on /lidar_tilt/angle_rad -
matches the message type Hermes already uses for its drive topics
(Int32/String/Float32 only, no array/dynamic-memory types in their
firmware), so this is a drop-in stand-in for what Hermes will publish once
it owns the servo. This is open-loop (no encoder feedback), so the
published angle is "what we told the servo to do", not measured position.
Standard hobby servos track commanded position closely enough (~few deg
error) that this is fine for a first pass - if you see visible cloud
smearing later, that's the failure mode to suspect.

Also broadcasts the dynamic TF for the tilt joint so RViz / robot_state_publisher
consumers see a moving frame. scan_to_cloud_node does its OWN angle
interpolation from /lidar_tilt/angle_rad directly (does not rely on TF
lookup) for performance reasons - see that node's docstring.

TODO(Teensy migration): this whole node goes away once Hermes publishes
/lidar_tilt/angle_rad itself as one more publisher in their existing
rclc_executor (same pattern as their drive topics). Confirm domain ID
matches whatever Hermes is currently built with (should be 0) before
wiring it in - don't let a copy-pasted snippet reintroduce an old hardcoded
domain ID.
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

try:
    from adafruit_pca9685 import PCA9685
    import board
    import busio
    HAVE_PCA9685 = True
except ImportError:
    HAVE_PCA9685 = False


class TiltServoNode(Node):
    def __init__(self):
        super().__init__('tilt_servo_node')

        # --- parameters ---
        self.declare_parameter('sweep_amplitude_deg', 22.5)   # +/- from center -> 45 deg total sweep
        self.declare_parameter('sweep_period_s', 2.0)         # full back-and-forth cycle
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('pca9685_channel', 0)
        self.declare_parameter('pca9685_freq_hz', 50)
        self.declare_parameter('servo_min_us', 500)            # 0 deg pulse width
        self.declare_parameter('servo_max_us', 2500)           # 180 deg pulse width
        self.declare_parameter('servo_center_deg', 90.0)       # mechanical center offset
        self.declare_parameter('joint_name', 'lidar_tilt_joint')
        self.declare_parameter('base_frame', 'lidar_tilt_base_link')
        self.declare_parameter('tilt_frame', 'lidar_tilt_link')
        self.declare_parameter('simulate', not HAVE_PCA9685)   # auto-fallback if lib missing

        self.amplitude = math.radians(self.get_parameter('sweep_amplitude_deg').value)
        self.period = self.get_parameter('sweep_period_s').value
        rate = self.get_parameter('publish_rate_hz').value
        self.joint_name = self.get_parameter('joint_name').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tilt_frame = self.get_parameter('tilt_frame').value
        self.simulate = self.get_parameter('simulate').value

        if self.simulate:
            self.get_logger().warn(
                'Running WITHOUT PCA9685 hardware (lib missing or simulate:=true). '
                'Publishing commanded angle only, no actual PWM output.'
            )
        else:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c)
            self.pca.frequency = self.get_parameter('pca9685_freq_hz').value
            self.channel = self.get_parameter('pca9685_channel').value

        self.servo_min_us = self.get_parameter('servo_min_us').value
        self.servo_max_us = self.get_parameter('servo_max_us').value
        self.servo_center_deg = self.get_parameter('servo_center_deg').value

        self.angle_pub = self.create_publisher(Float32, '/lidar_tilt/angle_rad', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'Sweeping {self.joint_name} +/-{math.degrees(self.amplitude):.1f} deg, '
            f'period {self.period:.2f}s'
        )

    def angle_at(self, t_s: float) -> float:
        """Commanded tilt angle (rad) at elapsed time t_s. Sine sweep for smooth motion."""
        return self.amplitude * math.sin(2.0 * math.pi * t_s / self.period)

    def set_servo_angle_deg(self, angle_deg: float):
        if self.simulate:
            return
        pulse_us = self.servo_min_us + (angle_deg / 180.0) * (self.servo_max_us - self.servo_min_us)
        pulse_us = max(self.servo_min_us, min(self.servo_max_us, pulse_us))
        duty_cycle = int((pulse_us / 1_000_000.0) * self.get_parameter('pca9685_freq_hz').value * 65535)
        self.pca.channels[self.channel].duty_cycle = duty_cycle

    def tick(self):
        now = self.get_clock().now()
        t_s = (now - self.start_time).nanoseconds * 1e-9
        angle_rad = self.angle_at(t_s)

        # command hardware: sweep angle is relative to mechanical center
        self.set_servo_angle_deg(self.servo_center_deg + math.degrees(angle_rad))

        # publish commanded angle. Float32 has no header - the receiving
        # node (scan_to_cloud_node) timestamps on arrival instead. There is
        # inherent lag between command and physical position, typically
        # 5-20ms for a standard servo over a 45deg range; not compensated here.
        msg = Float32()
        msg.data = angle_rad
        self.angle_pub.publish(msg)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.tilt_frame
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        # tilt about Y axis (pitch) - change if your mount rotates about a different axis
        half = angle_rad / 2.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = math.sin(half)
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = math.cos(half)
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = TiltServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
