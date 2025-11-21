#!/usr/bin/env python3
import sys
import time
import json
from datetime import datetime

from bosdyn.client import create_standard_sdk
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import get_a_tform_b, BODY_FRAME_NAME, VISION_FRAME_NAME


USERNAME = "user"
PASSWORD = "83arjd0xu5di"


def get_pose_dict(se3_pose):
    """Convert SE3Pose to Python dict for JSON serialization."""
    return {
        "position": {
            "x": se3_pose.position.x,
            "y": se3_pose.position.y,
            "z": se3_pose.position.z,
        },
        "rotation": {
            "w": se3_pose.rotation.w,
            "x": se3_pose.rotation.x,
            "y": se3_pose.rotation.y,
            "z": se3_pose.rotation.z,
        },
    }


def get_joint_states(robot_state):
    """Extract arm joint states if present."""
    joints = robot_state.kinematic_state.joint_states
    joint_data = {}

    for j in joints:
        joint_data[j.name] = {
            "position": j.position.value,
            "velocity": j.velocity.value,
            "torque": j.load.value,
        }

    return joint_data


def main():
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.80.3"

    sdk = create_standard_sdk("SpotObserver")
    robot = sdk.create_robot(robot_ip)

    # Authentication only — NO LEASE
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()

    state_client = robot.ensure_client(RobotStateClient.default_service_name)

    print("📡 Observing Spot robot state… (Ctrl+C to stop)")

    output_file = "spot_state_log.jsonl"   # JSON lines file

    with open(output_file, "w") as f:
        while True:
            try:
                # Fetch robot state
                state = state_client.get_robot_state()

                # 1. Get body pose (Vision frame recommended)
                vision_tform_body = get_a_tform_b(
                    state.kinematic_state.transforms_snapshot,
                    VISION_FRAME_NAME,
                    BODY_FRAME_NAME,
                )

                # 2. Joint states (arm + legs)
                joint_data = get_joint_states(state)

                # 3. Velocities
                vel = state.kinematic_state.velocity_of_body_in_vision

                # Package into dict
                record = {
                    "timestamp": datetime.utcnow().isoformat(),
                    "body_pose_vision_frame": get_pose_dict(vision_tform_body),
                    "body_velocity": {
                        "linear": {
                            "x": vel.linear.x,
                            "y": vel.linear.y,
                            "z": vel.linear.z,
                        },
                        "angular": {
                            "x": vel.angular.x,
                            "y": vel.angular.y,
                            "z": vel.angular.z,
                        },
                    },
                    "joint_states": joint_data,
                }

                # Write one JSON line
                f.write(json.dumps(record) + "\n")
                f.flush()

                # Frequency of logging
                time.sleep(0.1)  # 10 Hz

            except KeyboardInterrupt:
                print("\n🛑 Stopped logging.")
                break
            except Exception as e:
                print(f"⚠️ Error: {e}")
                time.sleep(0.5)


if __name__ == "__main__":
    main()
