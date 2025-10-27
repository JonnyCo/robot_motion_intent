#!/usr/bin/env python3
import sys
import time
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

            # --- Walk 1 (~2 m) ---
            print("Walking forward ~2 meters…")
            walk_cmd = RobotCommandBuilder.synchro_velocity_command(
                v_x=0.4, v_y=0.0, v_rot=0.0, frame_name=BODY_FRAME_NAME
            )
            send_with_deadline(robot, command_client, walk_cmd, duration_s=5.0)
            time.sleep(4.0)

            print("Stopping…")
            stop_cmd = RobotCommandBuilder.synchro_sit_command()
            send_with_deadline(robot, command_client, stop_cmd, duration_s=2.0)
            time.sleep(0.5)

            # --- Sit & wait ---
            print("Sitting and waiting 10 s…")
            blocking_sit(command_client, timeout_sec=15.0)
            time.sleep(5.0)

            # --- Walk 2 (~2 m) ---
            print("Standing again…")
            blocking_stand(command_client, timeout_sec=15.0)

            print("Walking forward again…")
            walk_cmd2 = RobotCommandBuilder.synchro_velocity_command(
                v_x=0.4, v_y=0.0, v_rot=0.0, frame_name=BODY_FRAME_NAME
            )
            send_with_deadline(robot, command_client, walk_cmd2, duration_s=5.0)
            time.sleep(4.0)

            print("Stopping…")
            send_with_deadline(robot, command_client, RobotCommandBuilder.synchro_sit_command(), duration_s=2.0)
            time.sleep(0.5)

            print("Done. Sitting down.")
            blocking_sit(command_client, timeout_sec=15.0)

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
