# -*- coding: utf-8 -*-
"""参考运动学模型（独立于被测模块实现）。

严格按 URDF 关节链与 env.py 的语义重新实现，作为测试的"真值基准"（test oracle）：
  - 关节链取自 fr5_description/urdf/fr5v6.urdf 的 <joint><origin>
  - 基座姿态取自 env.py:30 的 baseOrientation = RPY(0, 0, pi)
  - 夹爪中心公式取自 env.py:140-147 的 get_dis()

刻意不复用 team_algorithm.py 的任何代码，避免"用被测实现的假设去验证被测实现"。
"""
from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

# ---------------------------------------------------------------- 关节链定义
# (joint_xyz, joint_rpy, is_revolute)  —— 与 fr5v6.urdf 一一对应
JOINTS = [
    ("j1", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), True),
    ("j2", (0.0, 0.0, 0.152), (1.5708, 0.0, 0.0), True),
    ("j3", (-0.425, 0.0, 0.0), (0.0, 0.0, 0.0), True),
    ("j4", (-0.39501, 0.0, 0.0), (0.0, 0.0, 0.0), True),
    ("j5", (0.0, 0.0, 0.1021), (1.5708, 0.0, 0.0), True),
    ("j6", (0.0, 0.0, 0.102), (-1.5708, 0.0, 0.0), True),
    ("arm_hand_joint", (0.0, 0.0, 0.12), (0.0, 0.0, 3.14159), False),
]

# URDF 关节限位（弧度），取自 fr5v6.urdf 的 <limit lower/upper>
JOINT_LIMITS = [
    (-3.0543, 3.0543),   # j1
    (-4.6251, 1.4835),   # j2
    (-2.8274, 2.8274),   # j3
    (-4.6251, 1.4835),   # j4
    (-3.0543, 3.0543),   # j5
    (-3.0543, 3.0543),   # j6
]

BASE_RPY = (0.0, 0.0, math.pi)   # env.py:30 baseOrientation
# env.py:142 中夹爪中心相对 link7 的固定偏移
GRIPPER_OFFSET = np.array([0.0, 0.0, 0.15])

# env.py:61-70 reset() 设置的初始关节角（度）
NEUTRAL_DEG = np.array([-49.45849125928217, -57.601209583849, -138.394013961943,
                        -164.0052115563118, -49.45849125928217, 0.0])


def rpy_to_matrix(rpy) -> np.ndarray:
    """URDF 固定轴 XYZ 外旋约定：R = Rz(y) @ Ry(p) @ Rx(r)。"""
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
    """返回每个关节变换后的 (位置, 旋转矩阵) 列表，共 7 项。

    第 i 项即 PyBullet 中 link (i+1) 的世界位姿——由 team_algorithm.py 末尾的
    footnote（由 env.debugdoor() 真实输出）交叉验证。
    """
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
    """复刻 env.py:140-147 get_dis() 中的夹爪中心世界坐标。

    env 用 getLinkState(fr5, 6)[0] 取位置、getLinkState(fr5, 7)[1] 取姿态，
    即本模块 forward_kinematics 的第 6 项（索引 5）与第 7 项（索引 6）。
    """
    rad = np.asarray(joint_angles_deg, dtype=float) * math.pi / 180.0
    mats = forward_kinematics(rad)
    link6_pos = mats[5][0]
    link7_rot = mats[6][1]
    return link6_pos + link7_rot @ GRIPPER_OFFSET


def clamp_to_limits(joint_angles_rad) -> np.ndarray:
    """按 URDF 限位截断 6 个转动关节（PyBullet 位置控制会强制执行限位）。"""
    ang = np.asarray(joint_angles_rad, dtype=float).copy()
    for i, (lo, hi) in enumerate(JOINT_LIMITS):
        ang[i] = min(max(ang[i], lo), hi)
    return ang


def normalize_angles(deg) -> np.ndarray:
    """复刻 env.py:101-104 的观测归一化：d/180 -> [-1,1] -> [0,1]。"""
    return ((np.asarray(deg, dtype=float) / 180.0) + 1.0) / 2.0
