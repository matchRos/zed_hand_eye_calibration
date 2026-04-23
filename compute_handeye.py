#!/usr/bin/env python3
import os
import json
import glob
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

samples_dir = os.path.expanduser("~/handeye_samples")
json_files = sorted(glob.glob(os.path.join(samples_dir, "sample_*.json")))

if len(json_files) < 3:
    raise RuntimeError("Not enough samples found")

R_gripper2base = []
t_gripper2base = []
R_target2cam = []
t_target2cam = []

for jf in json_files:
    with open(jf, "r") as f:
        data = json.load(f)

    # tool pose: base_link -> tool0
    trans = np.array(data["tool_pose"]["translation"], dtype=np.float64)
    quat = np.array(data["tool_pose"]["quaternion"], dtype=np.float64)  # x,y,z,w
    rotm = R.from_quat(quat).as_matrix()

    # OpenCV expects gripper -> base
    R_gripper2base.append(rotm)
    t_gripper2base.append(trans.reshape(3, 1))

    # board pose: camera -> board from solvePnP
    rvec = np.array(data["board_pose"]["rvec"], dtype=np.float64).reshape(3, 1)
    tvec = np.array(data["board_pose"]["tvec"], dtype=np.float64).reshape(3, 1)
    rot_target2cam, _ = cv2.Rodrigues(rvec)

    R_target2cam.append(rot_target2cam)
    t_target2cam.append(tvec)

R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base,
    t_gripper2base,
    R_target2cam,
    t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI
)

print("R_cam2gripper:")
print(R_cam2gripper)
print("\nt_cam2gripper:")
print(t_cam2gripper)

quat_cam2gripper = R.from_matrix(R_cam2gripper).as_quat()
print("\nquat_cam2gripper [x, y, z, w]:")
print(quat_cam2gripper)

# optional: invert to get tool0 -> camera
T_cam2gripper = np.eye(4)
T_cam2gripper[:3, :3] = R_cam2gripper
T_cam2gripper[:3, 3] = t_cam2gripper.flatten()

T_gripper2cam = np.linalg.inv(T_cam2gripper)

print("\nT_gripper2cam:")
print(T_gripper2cam)

quat_gripper2cam = R.from_matrix(T_gripper2cam[:3, :3]).as_quat()
print("\ntranslation_gripper2cam:")
print(T_gripper2cam[:3, 3])
print("\nquat_gripper2cam [x, y, z, w]:")
print(quat_gripper2cam)
