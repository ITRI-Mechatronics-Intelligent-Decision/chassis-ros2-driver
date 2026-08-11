"""
Unit tests for the pure transform maths behind map_odom_bridge.

No hardware and no ROS graph is required: every function under test is a plain
function over (translation, quaternion) tuples.
"""

import math

from chassis_driver.map_odom_bridge import (
    compute_map_to_odom,
    quaternion_multiply,
    transform_inverse,
    transform_multiply,
)

IDENTITY = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def yaw_transform(x, y, yaw):
    """Build a planar (translation, quaternion) transform from x, y and yaw."""
    return ((x, y, 0.0), (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)))


def assert_transform_close(actual, expected, tol=1e-9):
    """Assert two transforms match, treating q and -q as the same rotation."""
    for a, e in zip(actual[0], expected[0]):
        assert abs(a - e) < tol

    same = all(abs(a - e) < tol for a, e in zip(actual[1], expected[1]))
    flipped = all(abs(a + e) < tol for a, e in zip(actual[1], expected[1]))
    assert same or flipped


def test_transform_inverse_round_trip():
    a = yaw_transform(1.5, -2.5, 0.7)
    assert_transform_close(transform_multiply(a, transform_inverse(a)), IDENTITY)
    assert_transform_close(transform_multiply(transform_inverse(a), a), IDENTITY)


def test_quaternion_multiply_composes_yaw():
    _, q_a = yaw_transform(0.0, 0.0, 0.4)
    _, q_b = yaw_transform(0.0, 0.0, 0.9)
    _, q_expected = yaw_transform(0.0, 0.0, 1.3)
    assert_transform_close(((0.0, 0.0, 0.0), quaternion_multiply(q_a, q_b)),
                           ((0.0, 0.0, 0.0), q_expected))


def test_map_to_odom_is_identity_when_odometry_has_no_drift():
    """With no drift the correction collapses to identity."""
    base_to_sensor = yaw_transform(0.3, 0.1, 0.2)
    odom_to_base = yaw_transform(2.0, 1.0, 0.5)
    map_to_sensor = transform_multiply(odom_to_base, base_to_sensor)

    result = compute_map_to_odom(map_to_sensor, base_to_sensor, odom_to_base)
    assert_transform_close(result, IDENTITY)


def test_map_to_odom_closes_the_tf_chain():
    """map->odom * odom->base * base->sensor must reproduce the external pose."""
    map_to_sensor = yaw_transform(5.0, -3.0, 1.1)
    base_to_sensor = yaw_transform(0.25, 0.05, -0.3)
    odom_to_base = yaw_transform(4.0, -2.0, 0.9)

    map_to_odom = compute_map_to_odom(map_to_sensor, base_to_sensor, odom_to_base)

    chained = transform_multiply(
        transform_multiply(map_to_odom, odom_to_base), base_to_sensor
    )
    assert_transform_close(chained, map_to_sensor)


def test_map_to_odom_recovers_a_pure_translation_drift():
    """A 1 m odometry under-run must show up as a 1 m map->odom offset."""
    base_to_sensor = yaw_transform(0.2, 0.0, 0.0)
    odom_to_base = yaw_transform(9.0, 0.0, 0.0)
    map_to_sensor = yaw_transform(10.2, 0.0, 0.0)

    result = compute_map_to_odom(map_to_sensor, base_to_sensor, odom_to_base)
    assert_transform_close(result, yaw_transform(1.0, 0.0, 0.0))
