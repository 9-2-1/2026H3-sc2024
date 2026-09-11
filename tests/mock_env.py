# -*- coding: utf-8 -*-
"""运动学级 Mock 环境，复刻 env.py 的对外契约，替代 pybullet。

mock 依据（逐行对应 env.py）：
  - reset()            -> env.py:56-98   随机化目标/障碍、回到中立位
  - get_observation()  -> env.py:100-114 归一化关节角 + 目标 + 障碍，形状 (1,12)
  - step(action)       -> env.py:116-138 关节角累加 action/180*pi，先判分再执行
  - reward()           -> env.py:149-190 成功/超时/碰撞惩罚判定表
  - get_dis()          -> env.py:140-147 夹爪中心到目标的欧氏距离

与真实环境的差异（已在测试报告中声明为测试替身的已知局限）：
  1. 关节按指令瞬时到位，不模拟 POSITION_CONTROL 的动力学滞后（动作上限 1°/步，
     20 个子步 ≈ 0.083 s，滞后可忽略）；
  2. 关节限位按 URDF <limit> 截断，与 PyBullet 对位置控制的强制限位一致；
  3. 不做碰撞检测，碰撞惩罚不在 mock 中计分（场景用例只断言最终距离）。
"""
from __future__ import annotations

import math

import numpy as np

import kinematics_ref as KR


class _StubBulletClient:
    """pybullet 客户端替身，只提供被测模块实际调用到的最小接口。

    team_algorithm.get_action 在 env 非空时会调用 env.p.getKeyboardEvents()
    做键盘遥操作（team_algorithm.py:347），本替身默认返回"无按键按下"。
    """

    KEY_IS_DOWN = 1

    def __init__(self, pressed: dict | None = None):
        self.pressed = pressed or {}
        self.call_count = 0

    def getKeyboardEvents(self):
        self.call_count += 1
        return self.pressed


class MockEnv:
    """env.py 中 Env 的运动学等价替身。"""

    max_steps = 100

    def __init__(self, is_senior: bool = True, seed: int = 100):
        self.is_senior = is_senior
        self.rng = np.random.default_rng(seed)
        self.p = _StubBulletClient()
        self.reset()

    # ------------------------------------------------------------- 回合重置
    def reset(self):
        self.step_num = 0
        self.success_reward = 0
        self.terminated = False
        self.obstacle_contact = False
        self.angles_deg = KR.NEUTRAL_DEG.copy()

        # env.py:79-81 目标位置分布
        self.goalx = self.rng.uniform(-0.2, 0.2)
        self.goaly = self.rng.uniform(0.8, 0.9)
        self.goalz = self.rng.uniform(0.1, 0.3)
        self.target_position = np.array([self.goalx, self.goaly, self.goalz])

        # env.py:87-91 障碍位置分布
        self.obstacle_position = np.array([
            self.rng.uniform(-0.2, 0.2) + self.goalx,
            0.6,
            self.rng.uniform(0.1, 0.3),
        ])
        return self.get_observation()

    # --------------------------------------------------------------- 观测
    def get_observation(self) -> np.ndarray:
        obs_angles = KR.normalize_angles(self.angles_deg)          # env.py:101-104
        return np.hstack((obs_angles, self.target_position,
                          self.obstacle_position)).reshape(1, -1)

    # --------------------------------------------------------------- 距离
    def get_dis(self) -> float:
        return float(np.linalg.norm(
            KR.gripper_centre(self.angles_deg) - self.target_position))

    # --------------------------------------------------------------- 判分
    def reward(self):
        """复刻 env.py:149-190 的判定表。注意 env 在动作执行前调用本函数。"""
        if self.get_dis() < 0.05 and self.step_num <= self.max_steps:
            self.success_reward = 100
            if self.obstacle_contact:
                self.success_reward = 20 if self.is_senior else 50
            self.terminated = True
        elif self.step_num >= self.max_steps:
            distance = self.get_dis()
            if 0.05 <= distance <= 0.2:
                self.success_reward = 100 * (1 - ((distance - 0.05) / 0.15))
            else:
                self.success_reward = 0
            if self.obstacle_contact:
                self.success_reward *= 0.2 if self.is_senior else 0.5
            self.terminated = True

    # --------------------------------------------------------------- 步进
    def step(self, action):
        self.step_num += 1
        action = np.clip(np.asarray(action, dtype=float), -1, 1)   # env.py:122-123
        # env.py:124 关节角累加，随后按 URDF 限位截断（PyBullet 位置控制语义）
        new_rad = KR.clamp_to_limits(
            self.angles_deg * math.pi / 180.0 + action[:6] / 180.0 * math.pi)
        self.reward()                                              # env.py:127
        self.angles_deg = new_rad * 180.0 / math.pi
        return self.get_observation()

    # ------------------------------------------------ 辅助：障碍接近度（估算）
    def min_link_to_obstacle(self) -> float:
        """各关节原点到障碍球心的最小距离，用于辅助观察是否可能碰撞。

        障碍半径 0.1 m；连杆按半径 0.05 m 的保守圆柱估算，仅作参考指标，
        不作为用例通过判据。
        """
        rad = self.angles_deg * math.pi / 180.0
        mats = KR.forward_kinematics(np.append(rad, 0.0))
        pts = [m[0] for m in mats[:7]]
        return float(min(np.linalg.norm(p - self.obstacle_position) for p in pts))


def run_episode(algorithm, env: MockEnv, max_steps: int = 100):
    """跑一个完整回合，返回 (步数, 最终夹爪-目标距离, 得分, 障碍最小接近距离)。"""
    while not env.terminated and env.step_num < max_steps:
        obs = env.get_observation()
        action = algorithm.get_action(obs, env)
        if action is None:
            return env.step_num, env.get_dis(), -1.0, env.min_link_to_obstacle()
        env.step(action)
    return env.step_num, env.get_dis(), env.success_reward, env.min_link_to_obstacle()
