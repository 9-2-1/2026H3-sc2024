from __future__ import annotations

import math

import numpy as np

import kinematics_ref as KR


class _StubBulletClient:
    KEY_IS_DOWN = 1

    def __init__(self, pressed: dict | None = None):
        self.pressed = pressed or {}
        self.call_count = 0

    def getKeyboardEvents(self):
        self.call_count += 1
        return self.pressed


class MockEnv:
    max_steps = 100

    def __init__(self, is_senior: bool = True, seed: int = 100):
        self.is_senior = is_senior
        self.rng = np.random.default_rng(seed)
        self.p = _StubBulletClient()
        self.reset()

    def reset(self):
        self.step_num = 0
        self.success_reward = 0
        self.terminated = False
        self.obstacle_contact = False
        self.angles_deg = KR.NEUTRAL_DEG.copy()

        self.goalx = self.rng.uniform(-0.2, 0.2)
        self.goaly = self.rng.uniform(0.8, 0.9)
        self.goalz = self.rng.uniform(0.1, 0.3)
        self.target_position = np.array([self.goalx, self.goaly, self.goalz])

        self.obstacle_position = np.array([
            self.rng.uniform(-0.2, 0.2) + self.goalx,
            0.6,
            self.rng.uniform(0.1, 0.3),
        ])
        return self.get_observation()

    def get_observation(self) -> np.ndarray:
        obs_angles = KR.normalize_angles(self.angles_deg)
        return np.hstack((obs_angles, self.target_position,
                          self.obstacle_position)).reshape(1, -1)

    def get_dis(self) -> float:
        return float(np.linalg.norm(
            KR.gripper_centre(self.angles_deg) - self.target_position))

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

    def step(self, action):
        self.step_num += 1
        action = np.clip(np.asarray(action, dtype=float), -1, 1)
        new_rad = KR.clamp_to_limits(
            self.angles_deg * math.pi / 180.0 + action[:6] / 180.0 * math.pi)
        self.reward()
        self.angles_deg = new_rad * 180.0 / math.pi
        return self.get_observation()

    def min_link_to_obstacle(self) -> float:
        rad = self.angles_deg * math.pi / 180.0
        mats = KR.forward_kinematics(np.append(rad, 0.0))
        pts = [m[0] for m in mats[:7]]
        return float(min(np.linalg.norm(p - self.obstacle_position) for p in pts))


def run_episode(algorithm, env: MockEnv, max_steps: int = 100):
    while not env.terminated and env.step_num < max_steps:
        obs = env.get_observation()
        action = algorithm.get_action(obs, env)
        if action is None:
            return env.step_num, env.get_dis(), -1.0, env.min_link_to_obstacle()
        env.step(action)
    return env.step_num, env.get_dis(), env.success_reward, env.min_link_to_obstacle()
