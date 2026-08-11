"""
ROS2 節點：把外部定位系統的絕對位姿換算成 REP-105 的 map -> odom 修正量.

外部定位/建圖系統（LIO、VIO 等）通常直接廣播自己那一支 TF，與底盤的
odom -> base_footprint 形成兩棵互不相連的樹。本節點改為訂閱其
nav_msgs/Odometry，扣掉底盤自身的輪速里程計後，只廣播 map -> odom 這一段，
使 TF 樹維持單一根節點：

    map -> odom -> base_footprint -> base_link -> {wheel_*, 買方掛載的感測器}

刻意與 chassis_driver 分離 — 本節點不碰序列埠（見 CLAUDE.md §3.1）。
"""

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

WARN_THROTTLE_SEC = 5.0


def quaternion_multiply(q1: tuple, q2: tuple) -> tuple:
    """Return the Hamilton product of two (x, y, z, w) quaternions."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quaternion_conjugate(q: tuple) -> tuple:
    """Return the conjugate of a unit (x, y, z, w) quaternion, i.e. its inverse."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def quaternion_rotate(q: tuple, v: tuple) -> tuple:
    """Rotate the 3-vector v by the unit quaternion q."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def transform_multiply(a: tuple, b: tuple) -> tuple:
    """Compose two (translation, quaternion) transforms and return a * b."""
    t_a, q_a = a
    t_b, q_b = b
    rx, ry, rz = quaternion_rotate(q_a, t_b)
    return (
        (t_a[0] + rx, t_a[1] + ry, t_a[2] + rz),
        quaternion_multiply(q_a, q_b),
    )


def transform_inverse(a: tuple) -> tuple:
    """Return the inverse of a (translation, quaternion) transform."""
    t, q = a
    q_inv = quaternion_conjugate(q)
    tx, ty, tz = quaternion_rotate(q_inv, t)
    return ((-tx, -ty, -tz), q_inv)


def compute_map_to_odom(
    map_to_sensor: tuple, base_to_sensor: tuple, odom_to_base: tuple
) -> tuple:
    """
    Return the REP-105 map -> odom correction.

    T(map->odom) = T(map->sensor) * T(base->sensor)^-1 * T(odom->base)^-1

    The first two terms give the chassis pose in map; right-multiplying by the
    inverse of the wheel odometry pose leaves only the drift between the two.
    """
    map_to_base = transform_multiply(map_to_sensor, transform_inverse(base_to_sensor))
    return transform_multiply(map_to_base, transform_inverse(odom_to_base))


def _startup_only() -> ParameterDescriptor:
    """Return a descriptor for a parameter that is only read at construction time."""
    return ParameterDescriptor(
        read_only=True,
        description='Read once at startup: set it through YAML or launch, not ros2 param set.',
    )


def _from_pose(pose) -> tuple:
    """Convert a geometry_msgs/Pose into a (translation, quaternion) tuple."""
    p, q = pose.position, pose.orientation
    return ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def _from_transform(transform) -> tuple:
    """Convert a geometry_msgs/Transform into a (translation, quaternion) tuple."""
    t, q = transform.translation, transform.rotation
    return ((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


class MapOdomBridge(Node):
    """Broadcast map -> odom so an external absolute pose source fits REP-105."""

    def __init__(self):
        super().__init__('map_odom_bridge')

        self._declare_parameters()
        self._map_frame = self.get_parameter('map_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._sensor_frame = self.get_parameter('sensor_frame').value

        self._base_to_sensor = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry, 'external_odom', self._external_odom_callback, 10
        )

    def _declare_parameters(self):
        self.declare_parameter('map_frame', 'map', _startup_only())
        self.declare_parameter('odom_frame', 'odom', _startup_only())
        self.declare_parameter('base_frame', 'base_footprint', _startup_only())
        self.declare_parameter('sensor_frame', 'box_link', _startup_only())

    def _external_odom_callback(self, msg: Odometry):
        if msg.header.frame_id != self._map_frame:
            self.get_logger().warn(
                f"external_odom is stamped '{msg.header.frame_id}' but map_frame is "
                f"'{self._map_frame}'; the pose is treated as {self._map_frame} -> "
                f'{self._sensor_frame} regardless',
                throttle_duration_sec=WARN_THROTTLE_SEC,
            )

        base_to_sensor = self._base_to_sensor_extrinsic()
        if base_to_sensor is None:
            return

        odom_to_base = self._lookup_odom_to_base(msg.header.stamp)
        if odom_to_base is None:
            return

        map_to_odom = compute_map_to_odom(
            _from_pose(msg.pose.pose), base_to_sensor, odom_to_base
        )
        self._broadcast(map_to_odom, msg.header.stamp)

    def _base_to_sensor_extrinsic(self):
        """Look the static base -> sensor extrinsic up once and cache it."""
        if self._base_to_sensor is not None:
            return self._base_to_sensor

        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, self._sensor_frame, Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Waiting for the static extrinsic {self._base_frame} -> '
                f'{self._sensor_frame}: {exc}',
                throttle_duration_sec=WARN_THROTTLE_SEC,
            )
            return None

        self._base_to_sensor = _from_transform(tf.transform)
        self.get_logger().info(
            f'Cached the static extrinsic {self._base_frame} -> {self._sensor_frame}'
        )
        return self._base_to_sensor

    def _lookup_odom_to_base(self, stamp):
        """Look the wheel odometry pose up at the stamp of the incoming message."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._odom_frame, self._base_frame, Time.from_msg(stamp)
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'No {self._odom_frame} -> {self._base_frame} at the message stamp: {exc}',
                throttle_duration_sec=WARN_THROTTLE_SEC,
            )
            return None
        return _from_transform(tf.transform)

    def _broadcast(self, map_to_odom: tuple, stamp):
        (tx, ty, tz), (qx, qy, qz, qw) = map_to_odom

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._map_frame
        tf.child_frame_id = self._odom_frame
        tf.transform.translation.x = tx
        tf.transform.translation.y = ty
        tf.transform.translation.z = tz
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
