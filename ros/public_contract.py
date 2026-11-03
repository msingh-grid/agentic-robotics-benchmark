"""Public names shared by readiness and the recording allowlist."""

# TODO(pawel): confirm topic, action, robot, camera and optical frame names.
CAMERAS = tuple(f"cam{i}" for i in range(4))
TOPICS = (
    "/clock", "/joint_states", "/tf", "/tf_static", "/flow/state", "/diagnostics",
    *(f"/cameras/{camera}/{suffix}" for camera in CAMERAS
      for suffix in ("color/image_raw", "depth/points")),
)
ACTION_TOPICS = ("/pick_place/_action/feedback", "/pick_place/_action/status")
ROBOT_FRAMES = {
    "robot_base", "robot_shoulder_link", "robot_upper_arm_link", "robot_forearm_link",
    "robot_wrist_1_link", "robot_wrist_2_link", "robot_wrist_3_link",
    "robot_gripper_base_mount", "robot_gripper_base",
    *(f"robot_gripper_{side}_{part}" for side in ("left", "right") for part in (
        "driver", "coupler", "spring_link", "follower", "pad", "silicone_pad",
    )),
}
FRAMES = ROBOT_FRAMES | set(CAMERAS) | {f"{camera}_optical" for camera in CAMERAS}
