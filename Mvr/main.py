from __future__ import annotations

import argparse
import logging
import sys
import time
from typing import Optional, Sequence

from core.cyberdog_bridge import CyberdogBridge


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Cyberdog bridge online test entrypoint")
    parser.add_argument("--camera-topic", default="/image", help="ROS 2 camera topic to subscribe to")
    parser.add_argument(
        "--action",
        choices=("none", "stand", "damper", "forward", "pose", "custom"),
        default="none",
        help="Optional test command to send after bridge startup",
    )
    parser.add_argument("--vx", type=float, default=0.15, help="Forward speed used by velocity tests")
    parser.add_argument("--vy", type=float, default=0.0, help="Lateral speed used by velocity tests")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw speed used by velocity tests")
    parser.add_argument("--gait", choices=("slow", "medium", "fast"), default="fast", help="Gait name for velocity tests")
    parser.add_argument("--gait-id", type=int, default=0, help="Explicit gait id for pose/custom action tests")
    parser.add_argument("--step-height", type=float, default=0.05, help="Step height used by send_velocity")
    parser.add_argument("--body-height", type=float, default=0.0, help="Body height offset used by send_velocity")
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="How long to maintain a forward command before sending zero velocity",
    )
    parser.add_argument("--duration-ms", type=int, default=0, help="Duration field for pose/custom action messages")
    parser.add_argument("--contact", type=int, default=0, help="Contact mask for pose/custom action tests")
    parser.add_argument("--value", type=int, default=0, help="Auxiliary value field for velocity/custom action tests")
    parser.add_argument("--mode", type=int, default=12, help="Custom action mode or pose mode")
    parser.add_argument("--pose-roll", type=float, default=0.0, help="Roll target used by pose test")
    parser.add_argument("--pose-pitch", type=float, default=0.0, help="Pitch target used by pose test")
    parser.add_argument("--pose-yaw", type=float, default=0.0, help="Yaw target used by pose test")
    parser.add_argument("--pose-x", type=float, default=0.0, help="X target used by pose test")
    parser.add_argument("--pose-y", type=float, default=0.0, help="Y target used by pose test")
    parser.add_argument("--pose-z", type=float, default=0.0, help="Z target used by pose test")
    parser.add_argument(
        "--wait-motion",
        action="store_true",
        help="After sending the selected command, wait for a target motion state acknowledgement",
    )
    parser.add_argument("--wait-mode", type=int, default=None, help="Expected motion mode for wait_for_motion")
    parser.add_argument("--wait-gait-id", type=int, default=None, help="Expected gait id for wait_for_motion")
    parser.add_argument("--wait-timeout", type=float, default=5.0, help="Timeout in seconds for wait_for_motion")
    parser.add_argument(
        "--report-interval",
        type=float,
        default=1.0,
        help="Seconds between state printouts",
    )
    parser.add_argument(
        "--runtime",
        type=float,
        default=0.0,
        help="Exit automatically after this many seconds; 0 means run until Ctrl+C",
    )
    parser.add_argument(
        "--report-once",
        action="store_true",
        help="Print one report and exit after optional action and wait steps",
    )
    parser.add_argument(
        "--print-snapshot",
        action="store_true",
        help="Print a full snapshot dictionary on each report for debugging",
    )
    return parser


def format_float(value: Optional[float]) -> str:
    if value is None:
        return "None"
    return f"{value:.3f}"


def infer_wait_target(args: argparse.Namespace) -> tuple[Optional[int], Optional[int]]:
    if args.wait_mode is not None or args.wait_gait_id is not None:
        return args.wait_mode, args.wait_gait_id

    if args.action == "stand":
        return 12, 0
    if args.action == "damper":
        return 7, 0
    if args.action == "forward":
        return 11, 26
    if args.action == "pose":
        return args.mode, args.gait_id
    if args.action == "custom":
        return args.mode, args.gait_id
    return None, None


def maybe_wait_for_motion(bridge: CyberdogBridge, args: argparse.Namespace) -> None:
    if not args.wait_motion:
        return

    mode, gait_id = infer_wait_target(args)
    if mode is None or gait_id is None:
        logging.warning("wait_for_motion target is incomplete; skipping wait step")
        return

    logging.info(
        "Waiting for motion acknowledgement mode=%s gait_id=%s timeout=%.2fs",
        mode,
        gait_id,
        args.wait_timeout,
    )
    ok = bridge.wait_for_motion(mode, gait_id, timeout_s=args.wait_timeout)
    logging.info("wait_for_motion result: %s", ok)



def maybe_run_action(bridge: CyberdogBridge, args: argparse.Namespace) -> None:
    if args.action == "none":
        return
    if args.action == "stand":
        logging.info("Sending recovery_stand() test command")
        bridge.recovery_stand()
        maybe_wait_for_motion(bridge, args)
        return
    if args.action == "damper":
        logging.info("Sending pure_damper() test command")
        bridge.pure_damper()
        maybe_wait_for_motion(bridge, args)
        return
    if args.action == "forward":
        logging.info(
            "Sending forward velocity test command vx=%.3f vy=%.3f yaw=%.3f gait=%s step=%.3f body=%.3f for %.2fs",
            args.vx,
            args.vy,
            args.yaw,
            args.gait,
            args.step_height,
            args.body_height,
            args.duration,
        )
        bridge.send_velocity(
            args.vx,
            args.vy,
            args.yaw,
            gait=args.gait,
            step_height=args.step_height,
            body_height=args.body_height,
            value=args.value,
        )
        maybe_wait_for_motion(bridge, args)
        time.sleep(max(0.0, args.duration))
        bridge.send_velocity(0.0, 0.0, 0.0, gait=args.gait, step_height=args.step_height, body_height=args.body_height)
        return
    if args.action == "pose":
        rpy = [args.pose_roll, args.pose_pitch, args.pose_yaw]
        pos = [args.pose_x, args.pose_y, args.pose_z]
        logging.info(
            "Sending pose test command mode=%s gait_id=%s rpy=%s pos=%s duration_ms=%s contact=%s",
            args.mode,
            args.gait_id,
            rpy,
            pos,
            args.duration_ms,
            args.contact,
        )
        bridge.send_pose(
            rpy=rpy,
            pos=pos,
            duration_ms=args.duration_ms,
            gait_id=args.gait_id,
            mode=args.mode,
            contact=args.contact,
        )
        maybe_wait_for_motion(bridge, args)
        return
    if args.action == "custom":
        logging.info(
            "Sending custom action mode=%s gait_id=%s duration_ms=%s contact=%s value=%s",
            args.mode,
            args.gait_id,
            args.duration_ms,
            args.contact,
            args.value,
        )
        bridge.send_action(
            args.mode,
            args.gait_id,
            duration_ms=args.duration_ms,
            contact=args.contact,
            value=args.value,
        )
        maybe_wait_for_motion(bridge, args)



def log_bridge_report(bridge: CyberdogBridge, print_snapshot: bool) -> None:
    motion = bridge.get_motion_state()
    odom = bridge.get_odometry()
    joint = bridge.get_joint_state()
    camera = bridge.get_camera_frame()
    pose = bridge.get_current_pose()

    logging.info(
        "ctrl=%s connected=%s motion_safe=%s motor_safe=%s fresh_odom=%s fresh_cam=%s mode=%s gait=%s switch=%s ori_err=%s foot_err=%s pose=(%s,%s,%s yaw=%s) v_body=(%s,%s,%s) joint_err=%s cam=%s cam_ts=%s",
        bridge.get_control_mode(),
        bridge.is_connected(),
        bridge.is_motion_safe(),
        bridge.is_motor_safe(),
        bridge.has_fresh_odometry(),
        bridge.has_fresh_camera_frame(),
        motion.mode,
        motion.gait_id,
        motion.switch_status,
        motion.ori_error,
        motion.footpos_error,
        format_float(pose["x"]),
        format_float(pose["y"]),
        format_float(pose["z"]),
        format_float(pose["yaw"]),
        format_float(odom.v_body[0]),
        format_float(odom.v_body[1]),
        format_float(odom.v_body[2]),
        joint.err_flag,
        camera.image is not None,
        format_float(camera.timestamp),
    )
    if print_snapshot:
        logging.info("snapshot=%s", bridge.get_robot_state_snapshot())



def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )

    bridge = CyberdogBridge(camera_topic=args.camera_topic)
    logging.info("Starting CyberdogBridge online test entrypoint")
    bridge.start()
    start_time = time.time()

    try:
        time.sleep(1.0)
        maybe_run_action(bridge, args)
        log_bridge_report(bridge, args.print_snapshot)

        if args.report_once:
            return 0

        while True:
            if args.runtime > 0 and time.time() - start_time >= args.runtime:
                logging.info("Runtime limit reached, stopping bridge")
                break
            time.sleep(max(0.1, args.report_interval))
            log_bridge_report(bridge, args.print_snapshot)
    except KeyboardInterrupt:
        logging.info("Interrupted by user, stopping bridge")
    finally:
        try:
            bridge.send_velocity(0.0, 0.0, 0.0, gait=args.gait, step_height=args.step_height, body_height=args.body_height)
        except Exception:
            pass
        bridge.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
