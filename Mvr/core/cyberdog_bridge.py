"""Cyberdog hardware bridge.

This module centralizes all direct communication with robot-side middleware.
It exposes a small, thread-safe Python API to the upper layers so the rest of
the project does not need to touch LCM, ROS 2, camera messages, or heartbeat
details directly.
"""

from __future__ import annotations

import copy
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence

try:
    import lcm
except ImportError:  # pragma: no cover - depends on robot environment
    lcm = None

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
except ImportError:  # pragma: no cover - depends on robot environment
    rclpy = None
    SingleThreadedExecutor = None
    Node = object

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - depends on robot environment
    CvBridge = None

try:
    from sensor_msgs.msg import Image
except ImportError:  # pragma: no cover - depends on robot environment
    Image = None

try:
    from utils.localization_lcmt import localization_lcmt
    from utils.robot_control_cmd_lcmt import robot_control_cmd_lcmt
    from utils.robot_control_response_lcmt import robot_control_response_lcmt
except ImportError as exc:  # pragma: no cover - project integrity issue
    raise ImportError("Required LCM message definitions are missing from utils/") from exc


LOGGER = logging.getLogger(__name__)

LCM_CMD_URL = "udpm://239.255.76.67:7671?ttl=255"
LCM_STATE_URL = "udpm://239.255.76.67:7670?ttl=255"
LCM_ODOM_URL = "udpm://239.255.76.67:7667?ttl=255"

LCM_CMD_CHANNEL = "robot_control_cmd"
LCM_STATE_CHANNEL = "robot_control_response"
LCM_ODOM_CHANNEL = "global_to_robot"

DEFAULT_CAMERA_TOPIC = "/image"
DEFAULT_CMD_HEARTBEAT_HZ = 20.0
DEFAULT_STATE_TIMEOUT_SEC = 1.0
DEFAULT_CAMERA_TIMEOUT_SEC = 1.0

CONTROL_MODE_IDLE = "idle"
CONTROL_MODE_HIGH_LEVEL = "high_level"
CONTROL_MODE_MOTOR_LEVEL = "motor_level"

GAIT_MAP = {
    "slow": 27,
    "medium": 3,
    "fast": 26,
}


@dataclass
class MotionState:
    mode: int = 0
    gait_id: int = 0
    contact: int = 0
    order_process_bar: int = 0
    switch_status: int = 0
    ori_error: int = 0
    footpos_error: int = 0
    motor_error: List[int] = field(default_factory=lambda: [0] * 12)
    stamp: float = 0.0


@dataclass
class OdometryState:
    xyz: List[float] = field(default_factory=lambda: [0.0] * 3)
    vxyz: List[float] = field(default_factory=lambda: [0.0] * 3)
    rpy: List[float] = field(default_factory=lambda: [0.0] * 3)
    omega_body: List[float] = field(default_factory=lambda: [0.0] * 3)
    v_body: List[float] = field(default_factory=lambda: [0.0] * 3)
    timestamp: int = 0
    stamp: float = 0.0


@dataclass
class ImuState:
    quat: List[float] = field(default_factory=lambda: [0.0] * 4)
    rpy: List[float] = field(default_factory=lambda: [0.0] * 3)
    acc: List[float] = field(default_factory=lambda: [0.0] * 3)
    omega: List[float] = field(default_factory=lambda: [0.0] * 3)
    timestamp: int = 0
    stamp: float = 0.0


@dataclass
class JointState:
    q: List[float] = field(default_factory=lambda: [0.0] * 12)
    qd: List[float] = field(default_factory=lambda: [0.0] * 12)
    tau: List[float] = field(default_factory=lambda: [0.0] * 12)
    motor_flags: List[int] = field(default_factory=lambda: [0] * 12)
    err_flag: int = 0
    ctrl_topic_interval: float = 0.0
    timestamp: int = 0
    stamp: float = 0.0


@dataclass
class CameraFrame:
    image: Any = None
    timestamp: float = 0.0
    frame_id: str = ""
    source: str = ""


@dataclass
class MotorCommand:
    q_des: List[float] = field(default_factory=lambda: [0.0] * 12)
    qd_des: List[float] = field(default_factory=lambda: [0.0] * 12)
    kp_des: List[float] = field(default_factory=lambda: [0.0] * 12)
    kd_des: List[float] = field(default_factory=lambda: [0.0] * 12)
    tau_des: List[float] = field(default_factory=lambda: [0.0] * 12)


class CyberdogBridge:
    """Unified bridge for Cyberdog motion, sensor and camera access."""

    def __init__(
        self,
        *,
        cmd_url: str = LCM_CMD_URL,
        state_url: str = LCM_STATE_URL,
        odom_url: str = LCM_ODOM_URL,
        cmd_channel: str = LCM_CMD_CHANNEL,
        state_channel: str = LCM_STATE_CHANNEL,
        odom_channel: str = LCM_ODOM_CHANNEL,
        camera_topic: str = DEFAULT_CAMERA_TOPIC,
        heartbeat_hz: float = DEFAULT_CMD_HEARTBEAT_HZ,
        state_timeout_sec: float = DEFAULT_STATE_TIMEOUT_SEC,
        camera_timeout_sec: float = DEFAULT_CAMERA_TIMEOUT_SEC,
        motor_command_sender: Optional[Callable[[MotorCommand], None]] = None,
        motor_state_receiver: Optional[Callable[[], Optional[Dict[str, Any]]]] = None,
        ros_node_name: str = "cyberdog_bridge",
    ) -> None:
        self._cmd_url = cmd_url
        self._state_url = state_url
        self._odom_url = odom_url
        self._cmd_channel = cmd_channel
        self._state_channel = state_channel
        self._odom_channel = odom_channel
        self._camera_topic = camera_topic
        self._heartbeat_period = 1.0 / heartbeat_hz if heartbeat_hz > 0 else 0.05
        self._state_timeout_sec = state_timeout_sec
        self._camera_timeout_sec = camera_timeout_sec
        self._motor_command_sender = motor_command_sender
        self._motor_state_receiver = motor_state_receiver
        self._ros_node_name = ros_node_name

        self._running = False
        self._lcm_available = lcm is not None
        self._ros_available = all(item is not None for item in (rclpy, SingleThreadedExecutor, Image))
        self._cv_bridge = CvBridge() if CvBridge is not None else None

        self._control_mode = CONTROL_MODE_IDLE
        self._life_count = 0

        self._motion_lock = threading.Lock()
        self._odom_lock = threading.Lock()
        self._imu_lock = threading.Lock()
        self._joint_lock = threading.Lock()
        self._image_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._mode_lock = threading.Lock()

        self._latest_motion_state = MotionState()
        self._latest_odometry = OdometryState()
        self._latest_imu_state = ImuState()
        self._latest_joint_state = JointState()
        self._latest_camera_frame = CameraFrame(source=camera_topic)

        self._last_motion_recv_time = 0.0
        self._last_odom_recv_time = 0.0
        self._last_motor_recv_time = 0.0
        self._last_camera_recv_time = 0.0
        self._last_cmd_send_time = 0.0

        self._last_high_level_cmd = self._build_high_level_cmd()
        self._last_motor_cmd = MotorCommand()

        self._lc_cmd = None
        self._lc_state = None
        self._lc_odom = None

        self._ros_context_initialized = False
        self._ros_node = None
        self._ros_executor = None
        self._ros_thread: Optional[threading.Thread] = None

        self._state_thread: Optional[threading.Thread] = None
        self._odom_thread: Optional[threading.Thread] = None
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._motor_state_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Start communication threads and subscriptions."""
        if self._running:
            return

        self._init_lcm_interfaces()
        self._init_ros_interfaces()

        self._running = True
        self._state_thread = threading.Thread(target=self._state_recv_loop, daemon=True)
        self._odom_thread = threading.Thread(target=self._odom_recv_loop, daemon=True)
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)

        self._state_thread.start()
        self._odom_thread.start()
        self._heartbeat_thread.start()

        if self._motor_state_receiver is not None:
            self._motor_state_thread = threading.Thread(target=self._motor_state_loop, daemon=True)
            self._motor_state_thread.start()

        if self._ros_executor is not None:
            self._ros_thread = threading.Thread(target=self._ros_spin_loop, daemon=True)
            self._ros_thread.start()

    def stop(self) -> None:
        """Stop background activity and release middleware resources."""
        if not self._running:
            return

        self._running = False
        threads = [
            self._state_thread,
            self._odom_thread,
            self._heartbeat_thread,
            self._motor_state_thread,
            self._ros_thread,
        ]
        for thread in threads:
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)

        self._shutdown_interfaces()

        self._state_thread = None
        self._odom_thread = None
        self._heartbeat_thread = None
        self._motor_state_thread = None
        self._ros_thread = None

    def get_motion_state(self) -> MotionState:
        with self._motion_lock:
            return copy.deepcopy(self._latest_motion_state)

    def get_odometry(self) -> OdometryState:
        with self._odom_lock:
            return copy.deepcopy(self._latest_odometry)

    def get_imu_data(self) -> ImuState:
        with self._imu_lock:
            return copy.deepcopy(self._latest_imu_state)

    def get_joint_state(self) -> JointState:
        with self._joint_lock:
            return copy.deepcopy(self._latest_joint_state)

    def get_camera_frame(self) -> CameraFrame:
        with self._image_lock:
            return copy.deepcopy(self._latest_camera_frame)

    def get_current_pose(self) -> Dict[str, float]:
        odom = self.get_odometry()
        return {
            "x": odom.xyz[0],
            "y": odom.xyz[1],
            "z": odom.xyz[2],
            "roll": odom.rpy[0],
            "pitch": odom.rpy[1],
            "yaw": odom.rpy[2],
        }

    def get_robot_state_snapshot(self) -> Dict[str, Any]:
        return {
            "control_mode": self.get_control_mode(),
            "motion": self.get_motion_state(),
            "odometry": self.get_odometry(),
            "imu": self.get_imu_data(),
            "joint": self.get_joint_state(),
            "camera": self.get_camera_frame(),
        }

    def send_velocity(
        self,
        vx: float,
        vy: float,
        yaw_rate: float,
        *,
        gait: str | None = None,
        gait_id: Optional[int] = None,
        step_height: float = 0.05,
        body_height: Optional[float] = None,
        value: int = 0,
    ) -> None:
        self._require_high_level_mode()
        resolved_gait_id = gait_id if gait_id is not None else GAIT_MAP.get(gait or "fast", 26)
        cmd = self._build_high_level_cmd(
            mode=11,
            gait_id=resolved_gait_id,
            vel_des=[vx, vy, yaw_rate],
            pos_des=[0.0, 0.0, body_height if body_height is not None else 0.0],
            step_height=[step_height, step_height],
            duration=0,
            value=value,
        )
        self._publish_high_level_cmd(cmd)

    def send_pose(
        self,
        *,
        rpy: Optional[Sequence[float]] = None,
        pos: Optional[Sequence[float]] = None,
        duration_ms: int = 0,
        gait_id: int = 0,
        mode: int = 21,
        contact: int = 0x0F,
    ) -> None:
        self._require_high_level_mode()
        cmd = self._build_high_level_cmd(
            mode=mode,
            gait_id=gait_id,
            rpy_des=list(rpy) if rpy is not None else [0.0, 0.0, 0.0],
            pos_des=list(pos) if pos is not None else [0.0, 0.0, 0.0],
            contact=contact,
            duration=duration_ms,
        )
        self._publish_high_level_cmd(cmd)

    def send_action(
        self,
        mode: int,
        gait_id: int,
        *,
        duration_ms: int = 0,
        contact: int = 0,
        value: int = 0,
    ) -> None:
        self._require_high_level_mode()
        cmd = self._build_high_level_cmd(
            mode=mode,
            gait_id=gait_id,
            duration=duration_ms,
            contact=contact,
            value=value,
        )
        self._publish_high_level_cmd(cmd)

    def recovery_stand(self) -> None:
        self.send_action(12, 0)

    def pure_damper(self) -> None:
        self.send_action(7, 0)

    def wait_for_motion(self, mode: int, gait_id: int, timeout_s: float = 5.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            motion = self.get_motion_state()
            if motion.mode == mode and motion.gait_id == gait_id:
                return True
            time.sleep(0.01)
        return False

    def enter_motor_control_mode(self, zero_frames: int = 5) -> None:
        if self._motor_command_sender is None:
            raise RuntimeError("Motor command sender is not configured.")

        with self._mode_lock:
            if self._control_mode == CONTROL_MODE_HIGH_LEVEL:
                raise RuntimeError("Cannot enter motor control while high-level control is active.")
            self._control_mode = CONTROL_MODE_MOTOR_LEVEL

        for _ in range(max(1, zero_frames)):
            self.send_zero_motor_command()
            time.sleep(0.01)

    def exit_motor_control_mode(self) -> None:
        with self._mode_lock:
            if self._control_mode == CONTROL_MODE_MOTOR_LEVEL:
                self._control_mode = CONTROL_MODE_IDLE

    def send_motor_command(
        self,
        q_des: Sequence[float],
        qd_des: Sequence[float],
        kp_des: Sequence[float],
        kd_des: Sequence[float],
        tau_des: Sequence[float],
    ) -> None:
        self._require_motor_mode()
        cmd = self._build_motor_cmd(q_des, qd_des, kp_des, kd_des, tau_des)
        self._publish_motor_cmd(cmd)

    def send_zero_motor_command(self) -> None:
        cmd = MotorCommand()
        self._publish_motor_cmd(cmd)

    def is_connected(self) -> bool:
        now = time.time()
        return bool(
            self._last_motion_recv_time and now - self._last_motion_recv_time <= self._state_timeout_sec
        )

    def is_motion_safe(self) -> bool:
        motion = self.get_motion_state()
        return motion.switch_status not in (2, 3) and motion.ori_error == 0

    def is_motor_safe(self) -> bool:
        joint = self.get_joint_state()
        return joint.err_flag == 0

    def has_fresh_camera_frame(self, max_age_s: Optional[float] = None) -> bool:
        age_limit = max_age_s if max_age_s is not None else self._camera_timeout_sec
        return bool(self._last_camera_recv_time and time.time() - self._last_camera_recv_time <= age_limit)

    def has_fresh_odometry(self, max_age_s: Optional[float] = None) -> bool:
        age_limit = max_age_s if max_age_s is not None else self._state_timeout_sec
        return bool(self._last_odom_recv_time and time.time() - self._last_odom_recv_time <= age_limit)

    def get_control_mode(self) -> str:
        with self._mode_lock:
            return self._control_mode

    def update_motor_state(self, state: Dict[str, Any]) -> None:
        """Inject bottom-layer motor and IMU state from an external receiver."""
        now = time.time()
        joint_state = JointState(
            q=list(state.get("q", [0.0] * 12)),
            qd=list(state.get("qd", [0.0] * 12)),
            tau=list(state.get("tau", [0.0] * 12)),
            motor_flags=list(state.get("motor_flags", [0] * 12)),
            err_flag=int(state.get("err_flag", 0)),
            ctrl_topic_interval=float(state.get("ctrl_topic_interval", 0.0)),
            timestamp=int(state.get("timestamp", 0)),
            stamp=now,
        )
        imu_state = ImuState(
            quat=list(state.get("quat", [0.0] * 4)),
            rpy=list(state.get("rpy", [0.0] * 3)),
            acc=list(state.get("acc", [0.0] * 3)),
            omega=list(state.get("omega", [0.0] * 3)),
            timestamp=int(state.get("timestamp", 0)),
            stamp=now,
        )

        with self._joint_lock:
            self._latest_joint_state = joint_state
            self._last_motor_recv_time = now
        with self._imu_lock:
            self._latest_imu_state = imu_state

    def _init_lcm_interfaces(self) -> None:
        if not self._lcm_available:
            LOGGER.warning("LCM is not available; motion LCM interfaces were not initialized.")
            return

        self._lc_cmd = lcm.LCM(self._cmd_url)
        self._lc_state = lcm.LCM(self._state_url)
        self._lc_odom = lcm.LCM(self._odom_url)

        self._lc_state.subscribe(self._state_channel, self._handle_motion_response)
        self._lc_odom.subscribe(self._odom_channel, self._handle_odometry)

    def _init_ros_interfaces(self) -> None:
        if not self._ros_available:
            LOGGER.warning("ROS 2 image interface is unavailable in the current environment.")
            return

        if not rclpy.ok():
            rclpy.init(args=None)
            self._ros_context_initialized = True

        self._ros_node = Node(self._ros_node_name)
        self._ros_node.create_subscription(Image, self._camera_topic, self._handle_camera_image, 10)
        self._ros_executor = SingleThreadedExecutor()
        self._ros_executor.add_node(self._ros_node)

    def _shutdown_interfaces(self) -> None:
        if self._ros_executor is not None:
            try:
                self._ros_executor.shutdown()
            except Exception:  # pragma: no cover - defensive cleanup
                LOGGER.exception("Failed to shut down ROS executor cleanly.")
        if self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception:  # pragma: no cover - defensive cleanup
                LOGGER.exception("Failed to destroy ROS node cleanly.")
        if self._ros_context_initialized and rclpy is not None and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:  # pragma: no cover - defensive cleanup
                LOGGER.exception("Failed to shut down ROS 2 cleanly.")

        self._ros_executor = None
        self._ros_node = None
        self._lc_cmd = None
        self._lc_state = None
        self._lc_odom = None

    def _state_recv_loop(self) -> None:
        while self._running:
            if self._lc_state is None:
                time.sleep(0.05)
                continue
            try:
                self._lc_state.handle_timeout(100)
            except AttributeError:
                self._lc_state.handle()
            except Exception:  # pragma: no cover - middleware runtime issue
                LOGGER.exception("State receive loop failed.")
                time.sleep(0.05)

    def _odom_recv_loop(self) -> None:
        while self._running:
            if self._lc_odom is None:
                time.sleep(0.05)
                continue
            try:
                self._lc_odom.handle_timeout(100)
            except AttributeError:
                self._lc_odom.handle()
            except Exception:  # pragma: no cover - middleware runtime issue
                LOGGER.exception("Odometry receive loop failed.")
                time.sleep(0.05)

    def _heartbeat_loop(self) -> None:
        while self._running:
            if self._control_mode == CONTROL_MODE_HIGH_LEVEL and self._lc_cmd is not None:
                with self._command_lock:
                    try:
                        self._lc_cmd.publish(self._cmd_channel, self._last_high_level_cmd.encode())
                        self._last_cmd_send_time = time.time()
                    except Exception:  # pragma: no cover - middleware runtime issue
                        LOGGER.exception("Failed to publish high-level heartbeat command.")
            time.sleep(self._heartbeat_period)

    def _motor_state_loop(self) -> None:
        while self._running and self._motor_state_receiver is not None:
            try:
                state = self._motor_state_receiver()
                if state:
                    self.update_motor_state(state)
            except Exception:  # pragma: no cover - hardware-specific runtime issue
                LOGGER.exception("Failed to fetch motor state from receiver.")
            time.sleep(0.002)

    def _ros_spin_loop(self) -> None:
        while self._running and self._ros_executor is not None:
            try:
                self._ros_executor.spin_once(timeout_sec=0.1)
            except Exception:  # pragma: no cover - middleware runtime issue
                LOGGER.exception("ROS spin loop failed.")
                time.sleep(0.05)

    def _handle_motion_response(self, channel: str, data: bytes) -> None:
        del channel
        msg = robot_control_response_lcmt.decode(data)
        now = time.time()
        state = MotionState(
            mode=int(msg.mode),
            gait_id=int(msg.gait_id),
            contact=int(msg.contact),
            order_process_bar=int(msg.order_process_bar),
            switch_status=int(msg.switch_status),
            ori_error=int(msg.ori_error),
            footpos_error=int(msg.footpos_error),
            motor_error=list(msg.motor_error),
            stamp=now,
        )
        with self._motion_lock:
            self._latest_motion_state = state
            self._last_motion_recv_time = now

    def _handle_odometry(self, channel: str, data: bytes) -> None:
        del channel
        msg = localization_lcmt.decode(data)
        now = time.time()
        odom = OdometryState(
            xyz=list(msg.xyz),
            vxyz=list(msg.vxyz),
            rpy=list(msg.rpy),
            omega_body=list(msg.omegaBody),
            v_body=list(msg.vBody),
            timestamp=int(msg.timestamp),
            stamp=now,
        )
        with self._odom_lock:
            self._latest_odometry = odom
            self._last_odom_recv_time = now

    def _handle_camera_image(self, msg: Any) -> None:
        image = msg
        if self._cv_bridge is not None:
            try:
                image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                LOGGER.exception("Failed to convert ROS image to OpenCV frame; storing raw message.")

        stamp = time.time()
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        frame = CameraFrame(
            image=image,
            timestamp=stamp,
            frame_id=getattr(getattr(msg, "header", None), "frame_id", ""),
            source=self._camera_topic,
        )
        with self._image_lock:
            self._latest_camera_frame = frame
            self._last_camera_recv_time = time.time()

    def _build_high_level_cmd(
        self,
        *,
        mode: int = 0,
        gait_id: int = 0,
        contact: int = 0,
        vel_des: Optional[Sequence[float]] = None,
        rpy_des: Optional[Sequence[float]] = None,
        pos_des: Optional[Sequence[float]] = None,
        acc_des: Optional[Sequence[float]] = None,
        ctrl_point: Optional[Sequence[float]] = None,
        foot_pose: Optional[Sequence[float]] = None,
        step_height: Optional[Sequence[float]] = None,
        value: int = 0,
        duration: int = 0,
    ) -> robot_control_cmd_lcmt:
        cmd = robot_control_cmd_lcmt()
        cmd.mode = int(mode)
        cmd.gait_id = int(gait_id)
        cmd.contact = int(contact)
        cmd.life_count = self._next_life_count()
        cmd.vel_des = self._normalize_length(vel_des, 3)
        cmd.rpy_des = self._normalize_length(rpy_des, 3)
        cmd.pos_des = self._normalize_length(pos_des, 3)
        cmd.acc_des = self._normalize_length(acc_des, 6)
        cmd.ctrl_point = self._normalize_length(ctrl_point, 3)
        cmd.foot_pose = self._normalize_length(foot_pose, 6)
        cmd.step_height = self._normalize_length(step_height, 2)
        cmd.value = int(value)
        cmd.duration = int(duration)
        return cmd

    def _build_motor_cmd(
        self,
        q_des: Sequence[float],
        qd_des: Sequence[float],
        kp_des: Sequence[float],
        kd_des: Sequence[float],
        tau_des: Sequence[float],
    ) -> MotorCommand:
        return MotorCommand(
            q_des=self._normalize_length(q_des, 12),
            qd_des=self._normalize_length(qd_des, 12),
            kp_des=self._normalize_length(kp_des, 12),
            kd_des=self._normalize_length(kd_des, 12),
            tau_des=self._normalize_length(tau_des, 12),
        )

    def _publish_high_level_cmd(self, cmd: robot_control_cmd_lcmt) -> None:
        if self._lc_cmd is None:
            raise RuntimeError("High-level LCM command interface is not initialized.")

        with self._mode_lock:
            if self._control_mode == CONTROL_MODE_MOTOR_LEVEL:
                raise RuntimeError("Cannot publish high-level command while in motor control mode.")
            self._control_mode = CONTROL_MODE_HIGH_LEVEL

        with self._command_lock:
            self._last_high_level_cmd = cmd
            self._lc_cmd.publish(self._cmd_channel, cmd.encode())
            self._last_cmd_send_time = time.time()

    def _publish_motor_cmd(self, cmd: MotorCommand) -> None:
        if self._motor_command_sender is None:
            raise RuntimeError("Motor command sender is not configured.")

        with self._command_lock:
            self._last_motor_cmd = copy.deepcopy(cmd)
            self._motor_command_sender(cmd)
            self._last_cmd_send_time = time.time()

    def _require_high_level_mode(self) -> None:
        with self._mode_lock:
            if self._control_mode == CONTROL_MODE_MOTOR_LEVEL:
                raise RuntimeError("High-level control is blocked while motor control mode is active.")

    def _require_motor_mode(self) -> None:
        with self._mode_lock:
            if self._control_mode != CONTROL_MODE_MOTOR_LEVEL:
                raise RuntimeError("Motor control commands require motor control mode.")

    def _next_life_count(self) -> int:
        self._life_count = (self._life_count + 1) % 256
        if self._life_count > 127:
            self._life_count -= 256
        return self._life_count

    @staticmethod
    def _normalize_length(values: Optional[Sequence[float]], size: int) -> List[float]:
        base = [0.0] * size
        if values is None:
            return base
        clipped = list(values[:size])
        if len(clipped) < size:
            clipped.extend([0.0] * (size - len(clipped)))
        return [float(item) for item in clipped]
