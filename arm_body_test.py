#!/usr/bin/env python3
import sys
import time
import math
from contextlib import ExitStack

from bosdyn.client import create_standard_sdk
from bosdyn.client.frame_helpers import BODY_FRAME_NAME
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import (
    RobotCommandClient,
    RobotCommandBuilder,
    blocking_stand,
    blocking_sit,
    ExpiredError,   # NOTE: import from robot_command in your SDK
)
from bosdyn.client.robot_state import RobotStateClient


# --- Credentials (edit these or use env vars) ---
USERNAME = "user"
PASSWORD = "83arjd0xu5di"


def send_with_deadline(robot, cmd_client, cmd, duration_s):
    """
    Send a command with an absolute end_time in the ROBOT'S clock.
    Retries once on ExpiredError after a time-sync refresh.
    """
    def _send_once():
        # Convert (local now + duration) --> robot time
        robot_end_time = robot.time_sync.robot_timestamp_from_local_secs(time.time() + duration_s)
        return cmd_client.robot_command(cmd, end_time_secs=time.time()+duration_s)

    try:
        return _send_once()
    except ExpiredError:
        # Small clock skew? Re-sync and try once more.
        robot.time_sync.wait_for_sync()
        return _send_once()


def send_no_deadline(cmd_client, cmd):
    """Send a command without an end_time so it persists until superseded or stopped."""
    return cmd_client.robot_command(cmd)


def main():
    # Allow passing IP on the command line: python Spot_Sit_walk_Sit.py 192.168.80.3
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.80.3"

    sdk = create_standard_sdk("SpotExpressiveIntentClient")
    robot = sdk.create_robot(robot_ip)

    # Authenticate & time sync
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()

    # Clients
    lease_client: LeaseClient = robot.ensure_client(LeaseClient.default_service_name)
    command_client: RobotCommandClient = robot.ensure_client(RobotCommandClient.default_service_name)
    _ = robot.ensure_client(RobotStateClient.default_service_name)  # optional but handy

    with ExitStack() as stack:
        # Acquire and keep the body lease BEFORE powering on
        lease_keepalive = stack.enter_context(
            LeaseKeepAlive(
                lease_client,
                must_acquire=True,
                return_at_exit=True,
            )
        )

        try:
            # Power on while holding the lease (separate E-Stop GUI must be allowing)
            robot.power_on(timeout_sec=20.0)

            print("Standing up…")
            blocking_stand(command_client, timeout_sec=15.0)

            # --- Walk 1 (~2 m) with arm motion ---
            print("Walking forward ~2 meters while moving arm…")

            init_base_cmd = RobotCommandBuilder.synchro_velocity_command(
                v_x=0.5, v_y=0.0, v_rot=0.0, body_height= -0.25, frame_name=BODY_FRAME_NAME
            )
            second_base_cmd = RobotCommandBuilder.synchro_velocity_command(
                v_x=0.5, v_y=0.0, v_rot=0.0, body_height= 0.25, frame_name=BODY_FRAME_NAME
            )

            # Arm points LEFT (yaw=+90°)
            yaw = 1.57
            half = yaw / 2.0
            qw = math.cos(half)
            qx = 0.0
            qy = 0.0
            qz = math.sin(half)

            init_arm_cmd = RobotCommandBuilder.arm_pose_command(
                x=0.5, y=0.5, z=0.215,
                qw=qw, qx=qx, qy=qy, qz=qz,
                frame_name=BODY_FRAME_NAME,
            )
            
            init_synchro_cmd = RobotCommandBuilder.build_synchro_command(init_base_cmd, init_arm_cmd)

            # Send without a deadline so the velocity holds until we stop it.
            #send_with_deadline(robot, command_client, init_synchro_cmd, duration_s=4.0)
            send_with_deadline(robot, command_client, init_base_cmd, duration_s=4.0)
            time.sleep(4.0)
            send_with_deadline(robot, command_client, second_base_cmd, duration_s=4.0)
            time.sleep(4.0)
            # send_with_deadline(robot, command_client, init_base_cmd, duration_s=4.0)


            print("command2")

            # # Spot rotates left 90° over ~2 seconds
            # base_cmd = RobotCommandBuilder.synchro_velocity_command(
            #     v_x=0.0, v_y=0.0, v_rot=0.79, frame_name=BODY_FRAME_NAME
            # )

            # # Arm straight forward
            # arm_cmd = RobotCommandBuilder.arm_cartesian_command(
            #     x=1.5, y=0.0, z=1.0,
            #     qw=1.0, qx=0.0, qy=0.0, qz=0.0,
            #     frame_name=BODY_FRAME_NAME,
            #     max_linear_velocity=2.0,          # m/s  (default ~0.5)
            #     max_angular_velocity=4.0          # rad/s (default ~1.0)
            # )


            # synchro_cmd = RobotCommandBuilder.build_synchro_command(base_cmd, arm_cmd)

            # # Send without a deadline so the velocity holds until we stop it.
            # send_with_deadline(robot, command_client, synchro_cmd, duration_s=2.0)
            # time.sleep(4.0)

            # print("Stopping…")
            # stop_cmd = RobotCommandBuilder.synchro_stop_command()
            # send_with_deadline(robot, command_client, stop_cmd, duration_s=2.0)
            # time.sleep(0.5)


        finally:
            # Best-effort stop while we still have the lease
            try:
                send_with_deadline(robot, command_client, RobotCommandBuilder.synchro_sit_command(), duration_s=2.0)
            except Exception:
                pass
            print("Powering off…")
            robot.power_off(cut_immediately=False, timeout_sec=25.0)


if __name__ == "__main__":
    main()
