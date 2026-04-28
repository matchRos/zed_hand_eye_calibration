#!/usr/bin/env python3
import os
import json
import glob
import cv2
import numpy as np
from charuco_board import BOARD_MODEL_ID


def rotation_matrix_from_quat(quat):
    x, y, z, w = quat
    norm = np.linalg.norm(quat)
    if norm == 0.0:
        raise ValueError("Quaternion has zero length")

    x, y, z, w = quat / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def quat_from_rotation_matrix(matrix):
    m = np.asarray(matrix, dtype=np.float64)
    trace = np.trace(m)

    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s

    quat = np.array([x, y, z, w], dtype=np.float64)
    return quat / np.linalg.norm(quat)


def make_transform(rotation, translation):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def rotation_angle_deg(rotation):
    value = (np.trace(rotation) - 1.0) / 2.0
    value = float(np.clip(value, -1.0, 1.0))
    return float(np.degrees(np.arccos(value)))


def summarize(values):
    values = np.asarray(values, dtype=np.float64)
    return {
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "max": float(np.max(values)),
    }


def main():
    samples_dir = os.path.expanduser("~/handeye_samples")
    json_files = sorted(glob.glob(os.path.join(samples_dir, "sample_*.json")))

    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []
    T_gripper2base = []
    T_target2cam = []
    sample_reprojection_errors = []
    used_files = []
    skipped_files = []

    for jf in json_files:
        with open(jf, "r") as f:
            data = json.load(f)

        model_id = data.get("board_model", {}).get("model_id")
        if model_id != BOARD_MODEL_ID:
            skipped_files.append(jf)
            continue

        used_files.append(jf)

        # lookupTransform("base_link", "tool0") gives tool0 expressed in
        # base_link, which is the gripper -> base transform OpenCV expects.
        trans = np.array(data["tool_pose"]["translation"], dtype=np.float64)
        quat = np.array(data["tool_pose"]["quaternion"], dtype=np.float64)
        rotm = rotation_matrix_from_quat(quat)

        R_gripper2base.append(rotm)
        t_gripper2base.append(trans.reshape(3, 1))
        T_gripper2base.append(make_transform(rotm, trans))

        # ChArUco pose is target -> camera, as expected by OpenCV.
        rvec = np.array(data["board_pose"]["rvec"], dtype=np.float64).reshape(3, 1)
        tvec = np.array(data["board_pose"]["tvec"], dtype=np.float64).reshape(3, 1)
        rot_target2cam, _ = cv2.Rodrigues(rvec)

        R_target2cam.append(rot_target2cam)
        t_target2cam.append(tvec)
        T_target2cam.append(make_transform(rot_target2cam, tvec))

        reprojection_error = data["board_pose"].get("reprojection_error_px", {}).get("mean")
        if reprojection_error is not None:
            sample_reprojection_errors.append(float(reprojection_error))

    if len(used_files) < 3:
        raise RuntimeError(
            "Not enough compatible samples found. "
            f"Need at least 3 samples with board_model={BOARD_MODEL_ID}, "
            f"found {len(used_files)}. "
            f"Skipped {len(skipped_files)} old or incompatible samples."
        )

    print(f"Using {len(used_files)} compatible samples from {samples_dir}")
    if skipped_files:
        print(f"Skipped {len(skipped_files)} old or incompatible samples")
    if sample_reprojection_errors:
        stats = summarize(sample_reprojection_errors)
        print(
            "ChArUco reprojection mean [px]: "
            f"mean={stats['mean']:.3f}, median={stats['median']:.3f}, max={stats['max']:.3f}"
        )

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )

    print("R_cam2gripper:")
    print(R_cam2gripper)
    print("\nt_cam2gripper:")
    print(t_cam2gripper)

    quat_cam2gripper = quat_from_rotation_matrix(R_cam2gripper)
    print("\nquat_cam2gripper [x, y, z, w]:")
    print(quat_cam2gripper)

    # This is the ROS static transform for parent=tool0,
    # child=zedm_left_camera_optical_frame.
    T_cam2gripper = np.eye(4)
    T_cam2gripper[:3, :3] = R_cam2gripper
    T_cam2gripper[:3, 3] = t_cam2gripper.flatten()

    print("\nT_tool0_camera:")
    print(T_cam2gripper)

    print("\nstatic_transform_publisher args:")
    print(
        "{:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} "
        "tool0 zedm_left_camera_optical_frame 100".format(
            t_cam2gripper[0, 0],
            t_cam2gripper[1, 0],
            t_cam2gripper[2, 0],
            quat_cam2gripper[0],
            quat_cam2gripper[1],
            quat_cam2gripper[2],
            quat_cam2gripper[3],
        )
    )

    T_gripper2cam = np.linalg.inv(T_cam2gripper)
    quat_gripper2cam = quat_from_rotation_matrix(T_gripper2cam[:3, :3])
    print("\nInverse transform, for debugging only:")
    print("translation_tool0_expressed_in_camera:")
    print(T_gripper2cam[:3, 3])
    print("quat_tool0_expressed_in_camera [x, y, z, w]:")
    print(quat_gripper2cam)

    T_base_target = [
        T_bg @ T_cam2gripper @ T_ct
        for T_bg, T_ct in zip(T_gripper2base, T_target2cam)
    ]
    translations = np.array([transform[:3, 3] for transform in T_base_target])
    center = np.mean(translations, axis=0)
    translation_errors_mm = np.linalg.norm(translations - center, axis=1) * 1000.0

    reference_rotation = T_base_target[0][:3, :3]
    rotation_errors_deg = [
        rotation_angle_deg(reference_rotation.T @ transform[:3, :3])
        for transform in T_base_target
    ]

    trans_stats = summarize(translation_errors_mm)
    rot_stats = summarize(rotation_errors_deg)
    worst_translation_idx = int(np.argmax(translation_errors_mm))
    worst_rotation_idx = int(np.argmax(rotation_errors_deg))
    print("\nBoard consistency in base_link:")
    print("Mean board position [m]:")
    print(center)
    print(
        "Translation residual [mm]: "
        f"mean={trans_stats['mean']:.2f}, "
        f"median={trans_stats['median']:.2f}, "
        f"max={trans_stats['max']:.2f}"
    )
    print(
        "Worst translation sample: "
        f"{os.path.basename(used_files[worst_translation_idx])} "
        f"({translation_errors_mm[worst_translation_idx]:.2f} mm)"
    )
    print(
        "Rotation spread vs first sample [deg]: "
        f"mean={rot_stats['mean']:.3f}, "
        f"median={rot_stats['median']:.3f}, "
        f"max={rot_stats['max']:.3f}"
    )
    print(
        "Worst rotation sample: "
        f"{os.path.basename(used_files[worst_rotation_idx])} "
        f"({rotation_errors_deg[worst_rotation_idx]:.3f} deg)"
    )


if __name__ == "__main__":
    main()
