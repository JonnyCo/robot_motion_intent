#!/usr/bin/env python3
import sys
import json
import time
import math

from bosdyn.client import create_standard_sdk
from bosdyn.client.robot_command import (
    RobotCommandClient,
    RobotCommandBuilder,
    blocking_stand,
    blocking_sit,
    ExpiredError,
)
from bosdyn.client.frame_helpers import BODY_FRAME_NAME
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive


USERNAME = "user"
PASSWORD = "83arjd0xu5di"


# ----------------------------------------------------------
# USE **YOUR EXACT** VERSION OF send_with_deadline
# ----------------------------------------------------------
def send_with_deadline(robot, cmd_client, cmd, duration_s):
    """
    Send a command with an absolute end_time in the ROBOT'S clock.
    Retries once on ExpiredError after a time-sync refresh.
    """
    def _send_once():
        # Convert (local now + duration) --> robot time
        robot_end_time = robot.time_sync.robot_timestamp_from_local_secs(
            time.time() + duration_s
        )
        # IMPORTANT: Your version intentionally uses LOCAL TIME instead of robot_end_time
        return cmd_client.robot_command(cmd, end_time_secs=time.time() + duration_s)

    try:
        return _send_once()
    except ExpiredError:
        # Small clock skew? Re-sync and try once more.
        robot.time_sync.wait_for_sync()
        return _send_once()
# ----------------------------------------------------------


def load_jsonl(path):
    return [json.loads(line) for line in open(path, "r")]


def quat_to_yaw(q):
    """Extract yaw from quaternion dict."""
    w, x, y, z = q["w"], q["x"], q["y"], q["z"]
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def clip(v, lim):
    return max(min(v, lim), -lim)


def main():
    log_path = sys.argv[1] if len(sys.argv) > 1 else "spot_state_log.jsonl"
    robot_ip = sys.argv[2] if len(sys.argv) > 2 else "192.168.80.3"

    print(f"Loading {log_path}…")
    entries = load_jsonl(log_path)
    print(f"Loaded {len(entries)} states.")

    sdk = create_standard_sdk("SpotPlaybackBaseOnly")
    robot = sdk.create_robot(robot_ip)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client = robot.ensure_client(RobotCommandClient.default_service_name)

    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):

        print("Powering on…")
        robot.power_on(timeout_sec=20)

        print("Standing…")
        blocking_stand(cmd_client, timeout_sec=15)

        print("🎬 Starting BASE-ONLY playback…")

        prev = entries[0]
        t_prev = time.time()

        for cur in entries[1:]:
            now = time.time()
            dt = now - t_prev
            t_prev = now

            if dt <= 0:
                dt = 0.001

            # ---- Compute velocities from logged poses ----
            p1 = prev["body_pose_vision_frame"]["position"]
            p2 = cur["body_pose_vision_frame"]["position"]

            dx = p2["x"] - p1["x"]
            dy = p2["y"] - p1["y"]

            vx = clip(dx / dt, 0.8)
            vy = clip(dy / dt, 0.8)

            yaw1 = quat_to_yaw(prev["body_pose_vision_frame"]["rotation"])
            yaw2 = quat_to_yaw(cur["body_pose_vision_frame"]["rotation"])

            dyaw = yaw2 - yaw1
            v_rot = clip(dyaw / dt, 1.5)

            # ---- Build base velocity command ----
            base_cmd = RobotCommandBuilder.synchro_velocity_command(
                v_x=vx,
                v_y=vy,
                v_rot=v_rot,
                frame_name=BODY_FRAME_NAME,
            )

            # ---- Use *your* send_with_deadline() ----
            send_with_deadline(robot, cmd_client, base_cmd, duration_s=0.2)

            time.sleep(0.1)
            prev = cur

        print("Stopping…")
        stop_cmd = RobotCommandBuilder.synchro_stop_command()
        send_with_deadline(robot, cmd_client, stop_cmd, duration_s=0.5)

        print("Sitting…")
        try:
            blocking_sit(cmd_client, timeout_sec=10)
        except:
            pass

        print("Powering off…")
        robot.power_off(timeout_sec=20)


if __name__ == "__main__":
    main()
