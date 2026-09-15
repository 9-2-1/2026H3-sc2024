"""参考运动学模型（独立于被测模块实现）。刻意不复用 team_algorithm.py 的任何代码，避免重言式测试"""
from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

# (joint_xyz, joint_rpy, is_revolute)
JOINTS = [
    ("j1", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), True),
    ("j2", (0.0, 0.0, 0.152), (1.5708, 0.0, 0.0), True),
    ("j3", (-0.425, 0.0, 0.0), (0.0, 0.0, 0.0), True),
    ("j4", (-0.39501, 0.0, 0.0), (0.0, 0.0, 0.0), True),
    ("j5", (0.0, 0.0, 0.1021), (1.5708, 0.0, 0.0), True),
    ("j6", (0.0, 0.0, 0.102), (-1.5708, 0.0, 0.0), True),
    ("arm_hand_joint", (0.0, 0.0, 0.12), (0.0, 0.0, 3.14159), False),
]

# URDF 关节限位（弧度）
JOINT_LIMITS = [
    (-3.0543, 3.0543),   # j1
    (-4.6251, 1.4835),   # j2
    (-2.8274, 2.8274),   # j3
    (-4.6251, 1.4835),   # j4
    (-3.0543, 3.0543),   # j5
    (-3.0543, 3.0543),   # j6
]

BASE_RPY = (0.0, 0.0, math.pi)
GRIPPER_OFFSET = np.array([0.0, 0.0, 0.15])

LINK6_INERTIAL_XYZ = np.array([7.7496e-05, 1.7751e-05, 0.076122])

NEUTRAL_DEG = np.array([-49.45849125928217, -57.601209583849, -138.394013961943,
                        -164.0052115563118, -49.45849125928217, 0.0])


def rpy_to_matrix(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def forward_kinematics(joint_angles_rad) -> List[Tuple[np.ndarray, np.ndarray]]:
    P = np.zeros(3)
    R = rpy_to_matrix(BASE_RPY)
    out: List[Tuple[np.ndarray, np.ndarray]] = []
    for idx, (_name, xyz, rpy, is_rev) in enumerate(JOINTS):
        P = P + R @ np.asarray(xyz, dtype=float)
        R = R @ rpy_to_matrix(rpy)
        if is_rev:
            R = R @ rot_z(float(joint_angles_rad[idx]))
        out.append((P.copy(), R.copy()))
    return out


def gripper_centre(joint_angles_deg) -> np.ndarray:
    rad = np.asarray(joint_angles_deg, dtype=float) * math.pi / 180.0
    mats = forward_kinematics(rad)
    link6_frame_pos, link6_frame_rot = mats[5]
    link6_com_pos = link6_frame_pos + link6_frame_rot @ LINK6_INERTIAL_XYZ
    link7_rot = mats[6][1]
    return link6_com_pos + link7_rot @ GRIPPER_OFFSET


def clamp_to_limits(joint_angles_rad) -> np.ndarray:
    ang = np.asarray(joint_angles_rad, dtype=float).copy()
    for i, (lo, hi) in enumerate(JOINT_LIMITS):
        ang[i] = min(max(ang[i], lo), hi)
    return ang


def normalize_angles(deg) -> np.ndarray:
    return ((np.asarray(deg, dtype=float) / 180.0) + 1.0) / 2.0
