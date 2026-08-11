"""
ROS2 節點：以 service 方式安全關閉上位機（onboard computer）.

刻意與 chassis_driver 分離 — 本節點不碰序列埠，只負責「上位機電源」這件事，
因此不違反單一序列 node 的原則（見 CLAUDE.md §3.1）。
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time as TimeMsg

from chassis_system.power import (
    DEFAULT_SHUTDOWN_COMMAND,
    ShutdownRejected,
    check_confirm_code,
    resolve_delay,
    run_shutdown,
)
from chassis_msgs.msg import MotorState
from chassis_msgs.srv import Shutdown


def _startup_only() -> ParameterDescriptor:
    """Return a descriptor for a parameter that is only read at construction time."""
    return ParameterDescriptor(
        read_only=True,
        description='Read once at startup: set it through YAML or launch, not ros2 param set.',
    )


class SystemServiceNode(Node):
    def __init__(self):
        super().__init__('system_service')

        self._declare_parameters()
        params = self._read_parameters()

        self._confirm_code = params['confirm_code']
        self._default_delay_sec = params['default_delay_sec']
        self._max_delay_sec = params['max_delay_sec']
        self._shutdown_command = list(params['shutdown_command'])
        self._require_stationary = params['require_stationary']
        self._stationary_rpm_threshold = params['stationary_rpm_threshold']
        self._motor_state_timeout = params['motor_state_timeout']
        self._stop_settle_timeout = params['stop_settle_timeout']
        self._tick_period = params['tick_period']
        self._dry_run = params['dry_run']

        self._deadline = None
        self._force = False
        self._countdown_timer = None
        self._latest_motor_state = None
        self._latest_motor_state_time = None

        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pending_pub = self.create_publisher(
            Bool, 'system/shutdown_pending', latching_qos
        )
        # 倒數期間持續送出零速，避免上位機斷電時 VCU 仍保留最後一筆速度命令
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(
            MotorState, 'chassis/motor_state', self._motor_state_callback, 10
        )
        self.create_service(Shutdown, 'system/shutdown', self._shutdown_callback)
        self.create_service(
            Trigger, 'system/shutdown_cancel', self._shutdown_cancel_callback
        )

        self._publish_pending(False)

        if self._dry_run:
            self.get_logger().warn('dry_run is enabled: shutdown requests will NOT power off')

    def _declare_parameters(self):
        self.declare_parameter('confirm_code', 'SHUTDOWN', _startup_only())
        self.declare_parameter('default_delay_sec', 5.0, _startup_only())
        self.declare_parameter('max_delay_sec', 300.0, _startup_only())
        self.declare_parameter('shutdown_command', DEFAULT_SHUTDOWN_COMMAND, _startup_only())
        self.declare_parameter('require_stationary', True, _startup_only())
        self.declare_parameter('stationary_rpm_threshold', 0, _startup_only())
        self.declare_parameter('motor_state_timeout', 1.0, _startup_only())
        self.declare_parameter('stop_settle_timeout', 3.0, _startup_only())
        self.declare_parameter('tick_period', 0.1, _startup_only())
        self.declare_parameter('dry_run', False, _startup_only())

    def _read_parameters(self) -> dict:
        return {
            'confirm_code': self.get_parameter('confirm_code').value,
            'default_delay_sec': self.get_parameter('default_delay_sec').value,
            'max_delay_sec': self.get_parameter('max_delay_sec').value,
            'shutdown_command': self.get_parameter('shutdown_command').value,
            'require_stationary': self.get_parameter('require_stationary').value,
            'stationary_rpm_threshold': self.get_parameter('stationary_rpm_threshold').value,
            'motor_state_timeout': self.get_parameter('motor_state_timeout').value,
            'stop_settle_timeout': self.get_parameter('stop_settle_timeout').value,
            'tick_period': self.get_parameter('tick_period').value,
            'dry_run': self.get_parameter('dry_run').value,
        }

    def _motor_state_callback(self, msg: MotorState):
        self._latest_motor_state = msg
        self._latest_motor_state_time = self.get_clock().now()

    def _blocking_reason(self) -> str | None:
        """回傳阻擋關機的理由；None 表示底盤已靜止、可安全斷電."""
        if not self._require_stationary:
            return None

        if self._latest_motor_state is None:
            return 'no message on /chassis/motor_state yet, chassis state unknown'

        age = (self.get_clock().now() - self._latest_motor_state_time).nanoseconds / 1e9
        if age > self._motor_state_timeout:
            return f'/chassis/motor_state is stale ({age:.1f}s), chassis state unknown'

        left = abs(self._latest_motor_state.left_rpm)
        right = abs(self._latest_motor_state.right_rpm)
        if max(left, right) > self._stationary_rpm_threshold:
            return f'chassis is still moving (left {left} rpm, right {right} rpm)'

        return None

    def _publish_pending(self, pending: bool):
        self._pending_pub.publish(Bool(data=pending))

    def _clear_schedule(self):
        if self._countdown_timer is not None:
            self._countdown_timer.cancel()
            self._countdown_timer = None
        self._deadline = None
        self._force = False
        self._publish_pending(False)

    def _shutdown_callback(self, request, response):
        response.success = False
        response.scheduled_time = TimeMsg()

        if self._deadline is not None:
            remaining = (self._deadline - self.get_clock().now()).nanoseconds / 1e9
            response.message = (
                f'a shutdown is already scheduled ({remaining:.1f}s left); '
                'call system/shutdown_cancel first'
            )
            return response

        try:
            check_confirm_code(request.confirm, self._confirm_code)
            delay = resolve_delay(
                request.delay_sec, self._default_delay_sec, self._max_delay_sec
            )
        except ShutdownRejected as exc:
            response.message = str(exc)
            self.get_logger().warn(f'Shutdown request rejected: {exc}')
            return response

        if not request.force:
            blocker = self._blocking_reason()
            if blocker is not None:
                response.message = f'shutdown refused: {blocker}'
                self.get_logger().warn(f'Shutdown request rejected: {blocker}')
                return response

        self._force = request.force
        self._deadline = self.get_clock().now() + Duration(seconds=delay)
        self._countdown_timer = self.create_timer(self._tick_period, self._countdown_tick)
        self._publish_pending(True)

        reason = request.reason or '(not given)'
        self.get_logger().warn(
            f'Shutdown scheduled in {delay:.1f}s '
            f'(force={request.force}, reason={reason})'
        )

        response.success = True
        response.scheduled_time = self._deadline.to_msg()
        response.message = f'shutdown scheduled in {delay:.1f}s'
        return response

    def _shutdown_cancel_callback(self, request, response):
        if self._deadline is None:
            response.success = False
            response.message = 'no shutdown is currently scheduled'
            return response

        self._clear_schedule()
        self.get_logger().warn('Scheduled shutdown cancelled')
        response.success = True
        response.message = 'scheduled shutdown cancelled'
        return response

    def _countdown_tick(self):
        # 倒數期間每個 tick 都補一筆零速，順便讓 driver 的 cmd_vel watchdog 保持在停止狀態
        self._cmd_vel_pub.publish(Twist())

        remaining = (self._deadline - self.get_clock().now()).nanoseconds / 1e9
        if remaining > 0.0:
            self.get_logger().warn(
                f'Powering off in {remaining:.0f}s', throttle_duration_sec=1.0
            )
            return

        blocker = None if self._force else self._blocking_reason()
        if blocker is None:
            self._execute_shutdown()
            return

        # 已到時間但底盤尚未靜止：再等 stop_settle_timeout，逾時才放棄
        if -remaining > self._stop_settle_timeout:
            self._clear_schedule()
            self.get_logger().error(f'Shutdown aborted, chassis did not stop: {blocker}')
            return

        self.get_logger().warn(
            f'Waiting for chassis to stop before power off: {blocker}',
            throttle_duration_sec=1.0,
        )

    def _execute_shutdown(self):
        if self._countdown_timer is not None:
            self._countdown_timer.cancel()
            self._countdown_timer = None

        self.get_logger().warn('Powering off the onboard computer now')
        try:
            message = run_shutdown(self._shutdown_command, dry_run=self._dry_run)
        except RuntimeError as exc:
            # 指令失敗代表機器不會斷電，必須回到待命狀態讓操作者能重試
            self._clear_schedule()
            self.get_logger().error(f'Shutdown failed: {exc}')
            return

        self.get_logger().warn(message)
        if self._dry_run:
            self._clear_schedule()


def main(args=None):
    rclpy.init(args=args)
    node = SystemServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
