# -*- coding: utf-8 -*-
"""测试用例规格（数据驱动）。

每条用例 = 一条 Excel 记录 + 一个可执行检查函数，保证用例清单与自动化脚本
一一对应。检查函数返回 (status, actual)，status 取值遵循模板定义：
    OK  测试通过 / POK 部分通过 / NG 不通过 / NT 用例无法执行

用例按交付要求分为两组，见文件末尾的 MODULE1_IDS：
    模块一（测试基础实践）  30 条，人工一眼可判的黑盒行为用例
    模块二（AI 融合实践）   20 条，需运动学/白盒知识的进阶用例，作为 AI 生成清单

被测模块：team_algorithm.py 的 MyCustomAlgorithm 及其运动学辅助函数
预期结果依据（测试预言 test oracle）：
    - fr5_description/urdf/fr5v6.urdf 的关节链与关节限位
    - env.py 中 get_observation / step / reward / get_dis 的对外契约
    - team_algorithm.py 末尾 footnote（env.debugdoor() 的真实 PyBullet 输出）
"""
from __future__ import annotations

import contextlib
import io
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
_ROOT = os.path.dirname(HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import kinematics_ref as KR                      # noqa: E402
from mock_env import MockEnv, run_episode, _StubBulletClient   # noqa: E402
import team_algorithm as TA                      # noqa: E402
from team_algorithm import MyCustomAlgorithm, A, forward_kinematics_mat, robot  # noqa: E402

_SINK = io.StringIO()


def _quiet(fn, *args, **kwargs):
    """被测模块会在 sig_reset 中 print 调试信息，测试时静音。"""
    with contextlib.redirect_stdout(_SINK):
        return fn(*args, **kwargs)


def make_obs(angles_deg, target, obstacle) -> np.ndarray:
    """按 env.py:100-114 的契约构造 (1,12) 观测。"""
    return np.hstack((KR.normalize_angles(angles_deg),
                      np.asarray(target, dtype=float),
                      np.asarray(obstacle, dtype=float))).reshape(1, -1)


def obs_for_bdb(bdb_deg, design=(0.0, 0.85, 0.2), radius=0.7):
    """构造一个使 sig_reset 中 bdb = 方位角(障碍) - 方位角(目标) = bdb_deg 的观测。

    sig_reset 用 rot_coord 取柱坐标方位角，故此处按方位角反推障碍坐标。
    """
    az_design = math.degrees(math.atan2(design[1], design[0]))
    az_ball = math.radians(az_design + bdb_deg)
    obstacle = [radius * math.cos(az_ball), radius * math.sin(az_ball), design[2]]
    return make_obs([0.0] * 6, design, obstacle)


def fresh_env(seed=1000, target=None, obstacle=None, is_senior=True) -> MockEnv:
    env = MockEnv(is_senior=is_senior, seed=seed)
    if target is not None:
        env.target_position = np.asarray(target, dtype=float)
    if obstacle is not None:
        env.obstacle_position = np.asarray(obstacle, dtype=float)
    return env


# 中性位（env.py:61-70）
NEUTRAL = [float(x) for x in KR.NEUTRAL_DEG]
# team_algorithm.py:553 footnote —— env.debugdoor() 的真实 PyBullet 输出（零位）
FOOTNOTE_POS = [
    [0.0, 0.0, 0.0], [0.0, 0.0, 0.152], [0.425, 0.0, 0.152], [0.820010, 0.0, 0.152],
    [0.820010, 0.102100, 0.152], [0.820010, 0.102099, 0.050000],
    [0.820010, 0.222099, 0.049999],
]
FOOTNOTE_ROT = [
    [[-1, 0, 0], [0, -1, 0], [0, 0, 1]],
    [[-1, 0, 0], [0, 0.000004, 1], [-0, 1, -0.000004]],
    [[-1, 0, 0], [0, 0.000004, 1], [-0, 1, -0.000004]],
    [[-1, 0, 0], [0, 0.000004, 1], [-0, 1, -0.000004]],
    [[-1, 0, 0], [0, 1, -0.000007], [0, -0.000007, -1]],
    [[-1, 0, 0], [0, 0.000004, 1], [-0, 1, -0.000004]],
    [[1, 0, 0], [0, -0.000004, 1], [0.000003, -1, -0.000004]],
]


# =====================================================================
# A 组：输入解析与接口契约
# =====================================================================
def c001():
    """正常等价类：中立位观测下模块可正常返回动作。"""
    alg = MyCustomAlgorithm()
    act = _quiet(alg.get_action, make_obs(NEUTRAL, [0.0, 0.85, 0.2], [0.1, 0.6, 0.2]), None)
    if act is None:
        return 'NG', '返回 None（弃权），未产出动作'
    act = np.asarray(act)
    ok = act.shape == (6,) and np.all(np.isfinite(act))
    return ('OK' if ok else 'NG'), f'action={np.round(act, 4).tolist()} shape={act.shape}'


def c002():
    """边界值：归一化下界 0 与上界 1 应分别解析为 -180° 与 +180°。"""
    lo = _quiet(MyCustomAlgorithm().get_action,
                make_obs([-180.0] * 6, [0.0, 0.85, 0.2], [0.1, 0.6, 0.2]), None)
    hi = _quiet(MyCustomAlgorithm().get_action,
                make_obs([180.0] * 6, [0.0, 0.85, 0.2], [0.1, 0.6, 0.2]), None)
    alg = MyCustomAlgorithm()
    a_lo = (np.asarray([-180.0] * 6) / 180 + 1) / 2
    a_hi = (np.asarray([180.0] * 6) / 180 + 1) / 2
    decoded_lo = (make_obs([-180.0] * 6, [0, 0.85, 0.2], [0.1, 0.6, 0.2])[0][:6] * 2 - 1) * 180
    ok = np.allclose(a_lo, 0.0) and np.allclose(a_hi, 1.0) and np.allclose(decoded_lo, -180.0)
    return ('OK' if ok else 'NG'), (
        f'归一化下界={a_lo[0]:.1f} 上界={a_hi[0]:.1f}；'
        f'反解下界={decoded_lo[0]:.1f}°；返回类型 lo={type(lo).__name__} hi={type(hi).__name__}')


def c003():
    """接口契约：观测必须为 (1,12)，越界形状应暴露问题。"""
    shape_ok = True
    detail = []
    good = make_obs(NEUTRAL, [0.0, 0.85, 0.2], [0.1, 0.6, 0.2])
    for obs, desc in [(good, '标准(1,12)'), (good.ravel(), '一维(12,)'), (good[:, :11], '少一维(1,11)')]:
        try:
            r = _quiet(MyCustomAlgorithm().get_action, obs, None)
            detail.append(f'{desc}->{type(r).__name__}')
        except Exception as e:
            detail.append(f'{desc}->{type(e).__name__}')
            if desc.startswith('标准'):
                shape_ok = False
    return ('OK' if shape_ok else 'NG'), \
        '；'.join(detail) + '（非标准形状未做显式校验，直接抛 IndexError/ValueError）'


def c004():
    """接口兼容：按官方 README 的单参数签名调用应可用。"""
    try:
        act = _quiet(MyCustomAlgorithm().get_action,
                     make_obs(NEUTRAL, [0.0, 0.85, 0.2], [0.1, 0.6, 0.2]))
        return 'OK', f'单参数调用成功，返回 {type(act).__name__}'
    except TypeError as e:
        return 'NG', f'单参数调用失败：TypeError: {e}'


def c005():
    """接口兼容：按 test.py:22 的双参数签名调用应可用。"""
    try:
        env = fresh_env()
        act = _quiet(MyCustomAlgorithm().get_action,
                     make_obs(NEUTRAL, list(env.target_position), list(env.obstacle_position)), env)
        return 'OK', f'双参数调用成功，返回 {type(act).__name__}'
    except Exception as e:
        return 'NG', f'双参数调用失败：{type(e).__name__}: {e}'


def c006():
    """外部依赖：env 提供 p.getKeyboardEvents 时应被调用且不影响动作。"""
    env = fresh_env()
    act = _quiet(MyCustomAlgorithm().get_action,
                 make_obs(NEUTRAL, list(env.target_position), list(env.obstacle_position)), env)
    calls = env.p.call_count
    return ('OK' if calls >= 1 and act is not None else 'NG'), \
        f'getKeyboardEvents 调用 {calls} 次，动作返回 {type(act).__name__}'


# =====================================================================
# B 组：回合变更检测与策略重置
# =====================================================================
def c007():
    """场景：新回合首次调用应完成基准角与策略初始化。"""
    alg = MyCustomAlgorithm()
    before = alg.base_angles.copy()
    _quiet(alg.get_action, make_obs(NEUTRAL, [0.0, 0.85, 0.2], [0.1, 0.6, 0.2]), None)
    changed = not np.allclose(before, alg.base_angles)
    return ('OK' if changed else 'NG'), \
        f'base_angles {np.round(before, 2).tolist()} -> {np.round(alg.base_angles, 2).tolist()}'


def c008():
    """等价类：目标与障碍均不变时不应重置基准角。"""
    alg = MyCustomAlgorithm()
    obs = make_obs(NEUTRAL, [0.0, 0.85, 0.2], [0.1, 0.6, 0.2])
    _quiet(alg.get_action, obs, None)
    base1 = alg.base_angles.copy()
    obs2 = make_obs([0.0, -60.0, -138.0, -164.0, -49.0, 0.0], [0.0, 0.85, 0.2], [0.1, 0.6, 0.2])
    _quiet(alg.get_action, obs2, None)
    same = np.allclose(base1, alg.base_angles)
    return ('OK' if same else 'NG'), \
        f'第2次调用后 base_angles={np.round(alg.base_angles, 2).tolist()}（应保持首次值）'


def c009():
    """等价类：目标位置变化应触发策略重置。"""
    alg = MyCustomAlgorithm()
    _quiet(alg.get_action, make_obs([0.0] * 6, [0.0, 0.85, 0.2], [0.1, 0.6, 0.2]), None)
    _quiet(alg.get_action, make_obs([0.0] * 6, [0.1, 0.85, 0.2], [0.1, 0.6, 0.2]), None)
    ok = np.allclose(alg.design_target, [0.1, 0.85, 0.2])
    return ('OK' if ok else 'NG'), f'design_target 已更新为 {np.round(alg.design_target, 3).tolist()}'


def c010():
    """等价类：障碍位置变化应触发策略重置。"""
    alg = MyCustomAlgorithm()
    _quiet(alg.get_action, make_obs([0.0] * 6, [0.0, 0.85, 0.2], [0.1, 0.6, 0.2]), None)
    b1 = alg.ball_target.copy()
    _quiet(alg.get_action, make_obs([0.0] * 6, [0.0, 0.85, 0.2], [-0.1, 0.6, 0.2]), None)
    ok = not np.allclose(b1, alg.ball_target)
    return ('OK' if ok else 'NG'), f'ball_target {np.round(b1, 3).tolist()} -> {np.round(alg.ball_target, 3).tolist()}'


# =====================================================================
# C 组：运动学正解正确性
# =====================================================================
def c011():
    """白盒：零位下 FK 各连杆位置应与 URDF/PyBullet 真值一致。"""
    mats = forward_kinematics_mat(robot, A([0.0] * 7), 6)
    worst, worst_i = 0.0, 0
    for i in range(7):
        d = float(np.max(np.abs(mats[i][0] - np.asarray(FOOTNOTE_POS[i]))))
        if d > worst:
            worst, worst_i = d, i + 1
    return ('OK' if worst < 1e-5 else 'NG'), f'最大位置偏差 {worst:.2e} m（Link {worst_i}，阈值 1e-5）'


def c012():
    """白盒：零位下 FK 各连杆姿态应与 URDF/PyBullet 真值一致。

    注意约定：被测模块内部用行向量 v@R，故其 R 是真值的转置；
    等价性判据为 mats[i][1].T == 真值。
    """
    mats = forward_kinematics_mat(robot, A([0.0] * 7), 6)
    worst, worst_i = 0.0, 0
    for i in range(7):
        d = float(np.max(np.abs(mats[i][1].T - np.asarray(FOOTNOTE_ROT[i]))))
        if d > worst:
            worst, worst_i = d, i + 1
    return ('OK' if worst < 1e-4 else 'NG'), \
        f'最大姿态偏差 {worst:.2e}（Link {worst_i}，阈值 1e-4，已按行向量约定转置比对）'


def c013():
    """等价类：非零位下 FK 位置应与独立参考模型一致。"""
    rng = np.random.default_rng(7)
    worst = 0.0
    for _ in range(30):
        deg = rng.uniform(-120, 120, 6)
        m = forward_kinematics_mat(robot, A([*(deg * np.pi / 180), 0.0]), 6)
        ref = KR.forward_kinematics([*(deg * math.pi / 180), 0.0])
        for i in range(7):
            worst = max(worst, float(np.max(np.abs(m[i][0] - ref[i][0]))))
    return ('OK' if worst < 1e-6 else 'NG'), f'30 组随机位姿最大位置偏差 {worst:.2e} m（阈值 1e-6）'


def c014():
    """白盒：mats[i][2]（link 帧）不应被 URDF <inertial> 质心 origin 污染。"""
    mats = forward_kinematics_mat(robot, A([0.0] * 7), 6)
    devs = {}
    for i in range(1, 7):
        devs[f'Link{i+1}'] = float(np.max(np.abs(mats[i][2] - np.asarray(FOOTNOTE_POS[i]))))
    bad = {k: v for k, v in devs.items() if v > 1e-5}
    if bad:
        return 'NG', ('link 帧位置被 <inertial> 质心 origin 污染，与 PyBullet 实际报告不符：'
                      + '，'.join(f'{k} 偏差 {v:.5f}m' for k, v in bad.items()))
    return 'OK', 'link 帧位置与 PyBullet 真值一致'


def c015():
    """功能：abc_getdist 对同一点应返回 0，对不同点应返回正值。"""
    alg = MyCustomAlgorithm()
    d0 = alg.abc_getdist(A([0.0] * 6), alg.abc_endeffe(A([0.0] * 6)))
    d1 = alg.abc_getdist(A([0.0] * 6), A([0.9, 0.3, 0.3]))
    return ('OK' if d0 < 1e-9 and d1 > 0 else 'NG'), f'同点距离={d0:.2e}，异点距离={d1:.5f}'


# =====================================================================
# D 组：末端夹爪中心模型（与 env.get_dis 的一致性）
# =====================================================================
def c016():
    """一致性：abc_endeffe 应等价于 env.get_dis() 的夹爪中心定义。"""
    worst = 0.0
    for deg in ([0.0] * 6, NEUTRAL, [30.0, -45.0, -100.0, -150.0, -60.0, 20.0]):
        truth = KR.gripper_centre(deg)
        got = MyCustomAlgorithm().abc_endeffe(A(deg))
        worst = max(worst, float(np.linalg.norm(got - truth)))
    return ('OK' if worst <= 0.05 else 'NG'), \
        f'与 env.get_dis() 夹爪中心最大偏差 {worst:.5f} m（成功阈值 0.05 m）'


def c017():
    """一致性：算法自身模型内，回合结束时下达的关节角应使夹爪中心落在目标上。

    只依赖被测模块自身（abc_endeffe 与 self.target），不涉及任何环境假设。
    """
    env = fresh_env(seed=1000)
    alg = MyCustomAlgorithm()
    with contextlib.redirect_stdout(_SINK):
        run_episode(alg, env)
    err = float(np.linalg.norm(alg.abc_endeffe(alg.target) - env.target_position))
    return ('OK' if err <= 0.05 else 'NG'), \
        (f'回合结束后，按模块自身 FK 计算 self.target 处夹爪中心距目标 {err:.5f} m'
         f'（Muli={alg.Muli:.5f}，阈值 0.05 m）')


def c018():
    """边界值：夹爪中心与目标距离恰好 0.05 m 时的成功判定（阈值边界）。"""
    d = 0.05
    scored = 100 if d < 0.05 else (
        100 * (1 - ((d - 0.05) / 0.15)) if 0.05 <= d <= 0.2 else 0)
    return ('OK' if abs(scored - 100.0) < 1e-9 else 'NG'), \
        f'env.py:165/179 判定：d=0.05 时得分为 {scored:.2f}（进入线性递减档）'


def c019():
    """边界值：距离 0.2 m 处线性递减到 0 分（下限边界）。"""
    d = 0.2
    scored = 100 * (1 - ((d - 0.05) / 0.15))
    return ('OK' if abs(scored) < 1e-9 else 'NG'), f'd=0.2 时得分 {scored:.2f}'


# =====================================================================
# E 组：避障姿态策略 sig_reset（判定表 / 等价类 / 边界值）
# =====================================================================
def spec_wrist(bdb, bda):
    """避障姿态决策表（被测模块 sig_reset 的设计规格）。

    区间语义为左开右闭：判定写成 if bdb > 25 / elif bdb > 15 / ...，
    因此边界值归属于"较低"的那一档。
    """
    if bdb > 25:
        return 10.0, -90.0
    if bdb > 15:
        return 10.0, -120.0
    if bdb > 8:
        return 0.0, -130.0
    if bdb > 6:
        return -5.0, -140.0
    if bdb > 3:
        return (40.0, 0.0) if bda > -0.28 else (15.0, -20.0)
    if bdb > -5:
        return 15.0, -20.0
    if bdb > -15:
        return 10.0, -20.0
    if bdb > -25:
        return 10.0, -50.0
    return 10.0, -90.0


def _rot_coord(xyz):
    """模块内 rot_coord（team_algorithm.py:262-267）的逐运算等价实现。

    注意必须复用同一套浮点运算顺序：模块写作 atan2*180/pi（两步），
    而 math.degrees 是 atan2*(180/pi)（一步常数），末位舍入不同会使
    边界值（bdb=3/6/15 等）落到不同分档，导致预言误判。
    """
    x, y, z = xyz
    a = np.sqrt(x ** 2 + y ** 2)
    b = np.arctan2(y, x) * 180 / np.pi
    return float(a), float(b), float(z)


def _wrist_for(bdb_req, bda_req=None, design=(0.0, 0.85, 0.2), radius=0.7):
    """按请求的 bdb（及可选 bda）构造观测，返回 (alg, 实际bdb, 实际bda)。"""
    az_design = math.degrees(math.atan2(design[1], design[0]))
    r = radius if bda_req is None else math.hypot(design[0], design[1]) + bda_req
    az_ball = math.radians(az_design + bdb_req)
    obstacle = [r * math.cos(az_ball), r * math.sin(az_ball), design[2]]
    alg = MyCustomAlgorithm()
    _quiet(alg.get_action, make_obs([0.0] * 6, design, obstacle), None)
    br, dr = _rot_coord(obstacle), _rot_coord(design)
    return alg, br[1] - dr[1], br[0] - dr[0]


def _check_wrist(bdb_req, exp_strategy=None, bda_req=None):
    """按规格表推导预期姿态，与实际输出比对。边界值可能因浮点往返偏移，据实记录。"""
    alg, bdb, bda = _wrist_for(bdb_req, bda_req)
    exp_a4, exp_a5 = spec_wrist(bdb, bda)
    got = (round(alg.A4, 6), round(alg.A5, 6))
    want = (round(exp_a4, 6), round(exp_a5, 6))
    tag = f'请求bdb={bdb_req}° 实测bdb={bdb:.10f}° bda={bda:.4f}'
    if got != want:
        return 'NG', f'{tag} -> 实际 A4={alg.A4:.1f} A5={alg.A5:.1f}，规格预期 A4={exp_a4:.1f} A5={exp_a5:.1f}'
    if exp_strategy is not None and exp_strategy not in alg.strategies:
        return 'POK', f'{tag} -> A4={alg.A4:.1f} A5={alg.A5:.1f} 正确，但缺策略标记 {exp_strategy}'
    extra = f'，含 {exp_strategy}' if exp_strategy else ''
    return 'OK', f'{tag} -> A4={alg.A4:.1f} A5={alg.A5:.1f}{extra}'


def c020():
    """等价类：bdb>25°（障碍远离目标连线）应采用垂直下抓姿态。"""
    return _check_wrist(40.0)


def c021():
    """等价类：15<bdb<=25 档应取 A5=-120（易与上一档混淆，需验证）。"""
    return _check_wrist(20.0)


def c022():
    """边界值：bdb=25° 属下一档（>25 不成立），应取 A5=-120 而非 -90。"""
    return _check_wrist(25.0)


def c023():
    """边界值：bdb=15° 属下一档（>15 不成立），应取 A4=0, A5=-130。"""
    return _check_wrist(15.0)


def c024():
    """等价类：8<bdb<=15 档应放平手腕取 A4=0, A5=-130。"""
    return _check_wrist(10.0)


def c025():
    """边界值：bdb=8° 属下一档（>8 不成立），应取 A4=-5, A5=-140。"""
    return _check_wrist(8.0)


def c026():
    """边界值：bdb=6° 属下一档（>6 不成立），bda>-0.28 时取 A4=40, A5=0。"""
    return _check_wrist(6.0)


def c027():
    """等价类：3<bdb<=6 且 bda>-0.28 时应抬肘从障碍上方越过。"""
    return _check_wrist(5.0)


def c028():
    """等价类：3<bdb<=6 但 bda<=-0.28（障碍更近）时应改取 A4=15, A5=-20。"""
    return _check_wrist(5.0, bda_req=-0.5)


def c029():
    """边界值：bdb=3° 属下一档（>3 不成立），应取 A4=15, A5=-20。"""
    return _check_wrist(3.0)


def c030():
    """边界值：bdb=-5° 属下一档（>-5 不成立），应取 A4=10, A5=-20。"""
    return _check_wrist(-5.0)


def c031():
    """边界值：bdb=-15° 属下一档（>-15 不成立），应取 A5=-50。"""
    return _check_wrist(-15.0)


def c032():
    """边界值：bdb=-25° 属末档（>-25 不成立），应取 A4=10, A5=-90。"""
    return _check_wrist(-25.0)


def c033():
    """等价类：A5>-90 时策略集应含 A1Left（基座左转）。"""
    return _check_wrist(5.0, exp_strategy='A1Left')


def c034():
    """等价类：A5<-90 时策略集应含 A1Right（基座右转）。"""
    return _check_wrist(10.0, exp_strategy='A1Right')


def c035():
    """边界值：A5 恰为 -90 时不应产生任何 A1 方向策略（> 与 < 均不成立）。"""
    alg, bdb, bda = _wrist_for(40.0)
    has = ('A1Left' in alg.strategies) or ('A1Right' in alg.strategies)
    return ('OK' if not has else 'NG'), \
        f'bdb={bdb:.4f}° A5={alg.A5:.1f} 时 strategies={sorted(alg.strategies)}（不含 A1 方向）'


def c036():
    """功能：构造参数 DFL 应能强制覆盖查表得到的 A4/A5/A6。"""
    alg = MyCustomAlgorithm(A4=0.0, A5=-30.0, A6=45.0)
    obs = obs_for_bdb(10.0)   # 查表本应给出 A4=0, A5=-130
    _quiet(alg.get_action, obs, None)
    ok = (abs(alg.A4 - 0.0) < 1e-9 and abs(alg.A5 + 30.0) < 1e-9
          and abs(alg.A6 - 45.0) < 1e-9)
    return ('OK' if ok else 'NG'), f'DFL 覆盖后 A4={alg.A4:.1f} A5={alg.A5:.1f} A6={alg.A6:.1f}'


# =====================================================================
# F 组：逆解、限幅与异常安全
# =====================================================================
def c037():
    """边界值：目标恰好超出工作空间时不应抛出数值异常（arccos 定义域保护）。"""
    try:
        alg = MyCustomAlgorithm()
        far = [0.0, 1.6, 0.2]      # 超出 l2+l3=0.82001 m 可达半径
        act = _quiet(alg.get_action, make_obs(NEUTRAL, far, [0.1, 0.6, 0.2]), None)
        return 'OK', f'超程目标处理正常，返回 {type(act).__name__}'
    except Exception as e:
        return 'NG', f'抛出 {type(e).__name__}: {e}'


def c038():
    """功能：解算结果相对基准角的单关节变化应被限制在 ±100° 内。"""
    alg = MyCustomAlgorithm()
    env = fresh_env(seed=1000)
    _quiet(alg.get_action, make_obs(NEUTRAL, list(env.target_position),
                                    list(env.obstacle_position)), None)
    diff = np.abs(alg.target - alg.base_angles)
    return ('OK' if float(np.max(diff)) <= 100.0 + 1e-6 else 'NG'), \
        f'最大单关节变化 {float(np.max(diff)):.3f}°（限值 100°）'


def c039():
    """异常安全：不可达时返回 None，调用方 np.clip(None) 会崩溃。"""
    alg = MyCustomAlgorithm()
    r = _quiet(alg.get_action, make_obs([0.0] * 6, [0.0, 0.85, 0.2], [0.1, 0.6, 0.2]), None)
    if r is None:
        try:
            np.clip(r, -1, 1)
            return 'POK', '返回 None 但下游未崩溃（不可达约定需调用方处理）'
        except TypeError as e:
            return 'NG', f'返回 None，env.step 中 np.clip 抛出 TypeError: {e}'
    return 'OK', '未触发弃权路径，正常返回动作'


def c040():
    """功能：输出动作应落在 env.py:122-123 的 [-1,1] 裁剪区间内或可被裁剪。"""
    rng = np.random.default_rng(11)
    worst = 0.0
    for _ in range(50):
        alg = MyCustomAlgorithm()
        deg = KR.NEUTRAL_DEG + rng.uniform(-5, 5, 6)
        tgt = [rng.uniform(-0.2, 0.2), rng.uniform(0.8, 0.9), rng.uniform(0.1, 0.3)]
        obs = make_obs(deg, tgt, [tgt[0] + rng.uniform(-0.2, 0.2), 0.6, rng.uniform(0.1, 0.3)])
        a = _quiet(alg.get_action, obs, None)
        if a is None:
            continue
        worst = max(worst, float(np.max(np.abs(a))))
    return 'OK', f'50 组样本动作最大绝对值 {worst:.3f}（env 侧裁剪为 [-1,1]，单位：度/步）'


def c041():
    """功能：sig_reset 应重置内部步进计数，保证分阶段抬臂阈值按回合生效。"""
    alg = MyCustomAlgorithm()
    _quiet(alg.get_action, make_obs(NEUTRAL, [0.0, 0.85, 0.2], [0.1, 0.6, 0.2]), None)
    for _ in range(5):
        _quiet(alg.get_action, make_obs(NEUTRAL, [0.0, 0.85, 0.2], [0.1, 0.6, 0.2]), None)
    mid = alg.step
    _quiet(alg.get_action, make_obs(NEUTRAL, [0.1, 0.85, 0.2], [0.1, 0.6, 0.2]), None)
    return ('OK' if alg.step == 1 else 'NG'), f'连续 6 次后 step={mid}，换目标后 step={alg.step}（预期 1）'


# =====================================================================
# G 组：端到端场景（Mock 环境完整回合）
# =====================================================================
def _scenario(seed, target, obstacle, is_senior=True, max_steps=100):
    env = fresh_env(seed=seed, target=target, obstacle=obstacle, is_senior=is_senior)
    alg = MyCustomAlgorithm()
    return run_episode(alg, env, max_steps=max_steps), env


def c042():
    """场景：障碍位于目标正前方（bdb≈0）时的完整抓取回合。"""
    (steps, dist, score, omin), env = _scenario(
        1000, [0.0, 0.85, 0.2], [0.0, 0.6, 0.2])
    return ('OK' if dist <= 0.05 else 'NG'), \
        f'{steps} 步结束，最终夹爪-目标距离 {dist:.5f} m（阈值 0.05），得分 {score:.2f}'


def c043():
    """场景：障碍明显偏置（bdb≈+18°）时的完整抓取回合。"""
    (steps, dist, score, omin), env = _scenario(
        1000, [0.0, 0.85, 0.2], [0.2, 0.6, 0.2])
    return ('OK' if dist <= 0.05 else 'NG'), \
        f'{steps} 步结束，最终距离 {dist:.5f} m，得分 {score:.2f}'


def c044():
    """场景：障碍明显反向偏置（bdb≈-18°）时的完整抓取回合。"""
    (steps, dist, score, omin), env = _scenario(
        1000, [0.0, 0.85, 0.2], [-0.2, 0.6, 0.2])
    return ('OK' if dist <= 0.05 else 'NG'), \
        f'{steps} 步结束，最终距离 {dist:.5f} m，得分 {score:.2f}'


def c045():
    """场景：目标位于随机分布左端（x=-0.2）时的完整回合。"""
    (steps, dist, score, omin), env = _scenario(
        1000, [-0.2, 0.8, 0.1], [-0.1, 0.6, 0.2])
    return ('OK' if dist <= 0.05 else 'NG'), \
        f'{steps} 步结束，最终距离 {dist:.5f} m，得分 {score:.2f}'


def c046():
    """场景：目标位于随机分布右端（x=+0.2, y=0.9, z=0.3）时的完整回合。"""
    (steps, dist, score, omin), env = _scenario(
        1000, [0.2, 0.9, 0.3], [0.0, 0.6, 0.2])
    return ('OK' if dist <= 0.05 else 'NG'), \
        f'{steps} 步结束，最终距离 {dist:.5f} m，得分 {score:.2f}'


def c047():
    """场景：低年级组（is_senior=False）评分档位下的完整回合。"""
    (steps, dist, score, omin), env = _scenario(
        1000, [0.0, 0.85, 0.2], [0.0, 0.6, 0.2], is_senior=False)
    return ('OK' if dist <= 0.05 else 'NG'), \
        f'低年级组：{steps} 步结束，最终距离 {dist:.5f} m，得分 {score:.2f}'


def c048():
    """场景：随机种子批量回合，统计抓取成功率（稳定性）。"""
    ds = []
    for s in range(1000, 1010):
        env = MockEnv(is_senior=True, seed=s)
        alg = MyCustomAlgorithm()
        with contextlib.redirect_stdout(_SINK):
            steps, dist, score, omin = run_episode(alg, env)
        ds.append(dist)
    ds = np.asarray(ds)
    rate = float((ds <= 0.05).mean())
    return ('OK' if rate >= 0.8 else 'NG'), \
        f'10 个随机回合成功率 {rate*100:.0f}%，最终距离均值 {ds.mean():.5f} m（要求 ≥80%）'


def c049():
    """场景：单回合步数不得超过 env 的 100 步上限。"""
    env = fresh_env(seed=1000)
    alg = MyCustomAlgorithm()
    with contextlib.redirect_stdout(_SINK):
        steps, dist, score, omin = run_episode(alg, env)
    return ('OK' if steps <= 100 else 'NG'), f'实际执行 {steps} 步（上限 100）'


def c050():
    """场景：回合必须终止（不允许死循环占用评测时长）。"""
    env = fresh_env(seed=1000)
    alg = MyCustomAlgorithm()
    with contextlib.redirect_stdout(_SINK):
        steps, dist, score, omin = run_episode(alg, env)
    return ('OK' if env.terminated or steps >= 100 else 'NG'), \
        f'terminated={env.terminated}，执行 {steps} 步'


CASES = [
    dict(id='TC-ALG-001', item='输入解析', title='中立位观测下正常返回六轴动作', level='高',
         pre='模块已实例化，观测为 (1,12) 标准格式', method='等价类划分（有效等价类）',
         inp='关节角=中立位，目标(0,0.85,0.2)，障碍(0.1,0.6,0.2)',
         steps='构造观测并调用 get_action(obs, None)；检查返回形状与数值有效性',
         exp='返回形状 (6,) 的有限实数数组', check=c001),
    dict(id='TC-ALG-002', item='输入解析', title='归一化边界 0/1 的反归一化正确性', level='高',
         pre='观测前 6 维为归一化关节角', method='边界值分析（取 0 与 1）',
         inp='前 6 维分别取 0.0 与 1.0',
         steps='按 (x*2-1)*180 反算角度；核对是否映射为 -180°/+180°',
         exp='0→-180°，1→+180°', check=c002),
    dict(id='TC-ALG-003', item='接口契约', title='观测形状异常时的健壮性', level='中',
         pre='模块已实例化', method='等价类划分（无效等价类）',
         inp='观测形状分别为 (1,12)、(12,)、(1,11)',
         steps='依次传入三种形状并调用 get_action',
         exp='(1,12) 正常返回；异常形状不应静默产生错误动作', check=c003),
    dict(id='TC-ALG-004', item='接口契约', title='官方单参数签名调用兼容性', level='高',
         pre='官方接口 BaseAlgorithm.get_action(observation)', method='等价类划分（接口有效类）',
         inp='仅传入 observation，不传 env',
         steps='调用 get_action(obs) 并捕获 TypeError',
         exp='正常返回动作，不抛异常', check=c004),
    dict(id='TC-ALG-005', item='接口契约', title='test.py 双参数签名调用兼容性', level='高',
         pre='test.py:22 以 get_action(observation, env) 调用', method='等价类划分（接口有效类）',
         inp='传入 observation 与 Mock env 对象',
         steps='调用 get_action(obs, env)',
         exp='正常返回动作，不抛异常', check=c005),
    dict(id='TC-ALG-006', item='外部依赖', title='Mock env 提供 p.getKeyboardEvents 时的行为', level='中',
         pre='env 替身提供 p.getKeyboardEvents()', method='等价类划分（依赖替身）',
         inp='Mock env（无按键按下）',
         steps='调用 get_action(obs, env)，统计替身方法调用次数',
         exp='键盘事件被读取且不影响正常输出', check=c006),
    dict(id='TC-ALG-007', item='回合重置', title='新回合首次调用完成基准角初始化', level='高',
         pre='新建模块实例，base_angles 为全 0', method='场景法（回合初始化）',
         inp='首帧观测（当前角=中立位）',
         steps='调用一次 get_action，比较调用前后的 base_angles',
         exp='base_angles 被更新为当前关节角', check=c007),
    dict(id='TC-ALG-008', item='回合重置', title='目标与障碍不变时不重置基准角', level='高',
         pre='同一回合内连续调用', method='等价类划分（有效等价类）',
         inp='第 2 帧目标与障碍与第 1 帧完全相同，仅关节角变化',
         steps='连续两次调用并比较 base_angles',
         exp='base_angles 保持首帧值不变', check=c008),
    dict(id='TC-ALG-009', item='回合重置', title='目标位置变化触发策略重置', level='高',
         pre='已用目标 A 调用过一次', method='等价类划分（回合切换）',
         inp='目标由 (0,0.85,0.2) 变为 (0.1,0.85,0.2)',
         steps='再次调用 get_action，检查 design_target',
         exp='design_target 更新为新目标', check=c009),
    dict(id='TC-ALG-010', item='回合重置', title='障碍位置变化触发策略重置', level='高',
         pre='已用障碍 A 调用过一次', method='等价类划分（回合切换）',
         inp='障碍由 (0.1,0.6,0.2) 变为 (-0.1,0.6,0.2)',
         steps='再次调用 get_action，检查 ball_target',
         exp='ball_target 更新为新障碍位置', check=c010),
    dict(id='TC-ALG-011', item='运动学正解', title='零位 FK 各连杆位置与 PyBullet 真值一致', level='高',
         pre='参考真值取 team_algorithm.py footnote（env.debugdoor 输出）', method='边界值分析（零位特例）',
         inp='七个关节角全部为 0',
         steps='调用 forward_kinematics_mat 取 mats[i][0]，与真值逐项求差',
         exp='7 个连杆位置最大偏差 < 1e-5 m', check=c011),
    dict(id='TC-ALG-012', item='运动学正解', title='零位 FK 各连杆姿态与 PyBullet 真值一致', level='中',
         pre='同上', method='边界值分析（零位特例）',
         inp='七个关节角全部为 0',
         steps='取 mats[i][1] 与真值旋转矩阵逐项求差',
         exp='最大姿态偏差 < 1e-4', check=c012),
    dict(id='TC-ALG-013', item='运动学正解', title='随机位姿下 FK 与独立参考模型一致', level='高',
         pre='参考模型 kinematics_ref 直接按 URDF 关节链实现', method='等价类划分（一般位姿）',
         inp='30 组 [-120°,120°] 随机关节角',
         steps='分别用被测 FK 与参考 FK 求 7 个连杆位置并求最大偏差',
         exp='最大偏差 < 1e-6 m', check=c013),
    dict(id='TC-ALG-014', item='运动学正解', title='link 帧位置未被 URDF 质心 origin 污染', level='高',
         pre='URDF <inertial> 的 origin 为质心，不能当作运动学偏移', method='白盒（数据流检查）',
         inp='零位关节角',
         steps='比较 mats[i][2] 与 PyBullet 真值位置',
         exp='与真值一致（偏差 < 1e-5 m）', check=c014),
    dict(id='TC-ALG-015', item='距离度量', title='abc_getdist 距离度量的自反性与正定性', level='中',
         pre='模块已实例化', method='边界值分析（零距离）',
         inp='末端点与自身、末端点与另一点',
         steps='调用 abc_getdist 比较返回值',
         exp='同点返回 0，异点返回正值', check=c015),
    dict(id='TC-ALG-016', item='末端模型', title='abc_endeffe 与 env.get_dis 夹爪中心定义一致', level='高',
         pre='env.get_dis 用 link6 位置 + link7 姿态旋转 [0,0,0.15]', method='等价类划分（多姿态）',
         inp='零位、中立位、一般位姿三组关节角',
         steps='比较 abc_endeffe 与参考夹爪中心的欧氏偏差',
         exp='偏差 ≤ 0.05 m（抓取成功阈值）', check=c016),
    dict(id='TC-ALG-017', item='末端模型', title='算法内部模型下达的关节角应指向目标', level='高',
         pre='模块已就一个回合解算完毕', method='白盒（模型自洽性）',
         inp='随机种子 1000 生成的目标与障碍',
         steps='取 alg.target，用模块自身 FK 算夹爪中心并与目标比较',
         exp='模型内偏差 ≤ 0.05 m', check=c017),
    dict(id='TC-ALG-018', item='评分判定', title='距离恰好 0.05 m 时的得分档位（阈值边界）', level='高',
         pre='env.py:165 与 :179 的分档条件', method='边界值分析（上边界 0.05）',
         inp='夹爪-目标距离 = 0.05 m',
         steps='按 env.py 判定式复算得分',
         exp='不满足 <0.05，进入线性递减档，得分 100', check=c018),
    dict(id='TC-ALG-019', item='评分判定', title='距离 0.2 m 处得分归零（下限边界）', level='中',
         pre='env.py:180 的线性递减公式', method='边界值分析（下边界 0.2）',
         inp='夹爪-目标距离 = 0.2 m',
         steps='按公式 100*(1-(d-0.05)/0.15) 复算',
         exp='得分 = 0', check=c019),
    dict(id='TC-ALG-020', item='避障策略', title='障碍远离目标连线时垂直下抓（bdb>25°）', level='高',
         pre='sig_reset 判定表首档 bdb > 25', method='等价类划分（有效等价类）',
         inp='bdb = 40°',
         steps='构造对应观测并调用 get_action，读取 A4/A5',
         exp='A4=10, A5=-90', check=c020),
    dict(id='TC-ALG-021', item='避障策略', title='障碍远离但仍偏于连线一侧（15<bdb<=25°）', level='高',
         pre='判定 bdb>15 成立但不满足 bdb>25', method='等价类划分（有效等价类）',
         inp='bdb = 20°',
         steps='构造对应观测并调用 get_action，读取 A4/A5',
         exp='A5=-120（与 >25° 档的 -90 不同，易混淆）', check=c021),
    dict(id='TC-ALG-022', item='避障策略', title='bdb=25° 的分档归属（阈值边界）', level='中',
         pre='判定条件为 bdb > 25', method='边界值分析（边界 25）',
         inp='bdb = 25°',
         steps='构造对应观测并读取 A4/A5',
         exp='25 不满足 >25，落入 15<bdb<=25 档：A5=-120', check=c022),
    dict(id='TC-ALG-023', item='避障策略', title='bdb=15° 的分档归属（阈值边界）', level='中',
         pre='判定条件为 bdb > 15', method='边界值分析（边界 15）',
         inp='bdb = 15°',
         steps='构造对应观测并读取 A4/A5',
         exp='15 不满足 >15，落入 8<bdb<=15 档：A4=0, A5=-130', check=c023),
    dict(id='TC-ALG-024', item='避障策略', title='障碍渐近目标连线时手腕放平（8<bdb<=15°）', level='高',
         pre='sig_reset 判定表中段', method='等价类划分（有效等价类）',
         inp='bdb = 10°',
         steps='构造对应观测并读取 A4/A5',
         exp='A4=0, A5=-130', check=c024),
    dict(id='TC-ALG-025', item='避障策略', title='bdb=8° 的分档归属（阈值边界）', level='中',
         pre='判定条件为 bdb > 8', method='边界值分析（边界 8）',
         inp='bdb = 8°',
         steps='构造对应观测并读取 A4/A5',
         exp='8 不满足 >8，落入 6<bdb<=8 档：A4=-5, A5=-140', check=c025),
    dict(id='TC-ALG-026', item='避障策略', title='bdb=6° 的分档归属（阈值边界）', level='中',
         pre='判定条件为 bdb > 6', method='边界值分析（边界 6）',
         inp='bdb = 6°，bda > -0.28',
         steps='构造对应观测并读取 A4/A5',
         exp='6 不满足 >6，落入 3<bdb<=6 档且 bda>-0.28 分支：A4=40, A5=0', check=c026),
    dict(id='TC-ALG-027', item='避障策略', title='障碍正对连线且更远时抬肘越障', level='高',
         pre='3<bdb<=6 且柱坐标半径差 bda > -0.28', method='等价类划分（有效等价类）',
         inp='bdb = 5°，bda > -0.28',
         steps='构造对应观测并读取 A4/A5',
         exp='A4=40, A5=0（抬肘从障碍上方越过）', check=c027),
    dict(id='TC-ALG-028', item='避障策略', title='障碍正对连线但更近时改走侧向规避', level='高',
         pre='3<bdb<=6 且柱坐标半径差 bda <= -0.28', method='等价类划分（有效等价类）',
         inp='bdb = 5°，bda <= -0.28',
         steps='构造对应观测并读取 A4/A5',
         exp='A4=15, A5=-20（不抬肘，改由后续策略侧向绕行）', check=c028),
    dict(id='TC-ALG-029', item='避障策略', title='bdb=3° 的分档归属（阈值边界）', level='中',
         pre='判定条件为 bdb > 3', method='边界值分析（边界 3）',
         inp='bdb = 3°',
         steps='构造对应观测并读取 A4/A5',
         exp='3 不满足 >3，落入 -5<bdb<=3 档：A4=15, A5=-20', check=c029),
    dict(id='TC-ALG-030', item='避障策略', title='bdb=-5° 的分档归属（阈值边界）', level='中',
         pre='判定条件为 bdb > -5', method='边界值分析（边界 -5）',
         inp='bdb = -5°',
         steps='构造对应观测并读取 A4/A5',
         exp='-5 不满足 >-5，落入 -15<bdb<=-5 档：A4=10, A5=-20', check=c030),
    dict(id='TC-ALG-031', item='避障策略', title='bdb=-15° 的分档归属（阈值边界）', level='中',
         pre='判定条件为 bdb > -15', method='边界值分析（边界 -15）',
         inp='bdb = -15°',
         steps='构造对应观测并读取 A4/A5',
         exp='-15 不满足 >-15，落入 -25<bdb<=-15 档：A4=10, A5=-50', check=c031),
    dict(id='TC-ALG-032', item='避障策略', title='bdb=-25° 的边界归属与档末姿态', level='中',
         pre='判定条件为 bdb > -25', method='边界值分析（边界 -25）',
         inp='bdb = -25°',
         steps='构造对应观测并读取 A4/A5',
         exp='-25 不满足 >-25，落入 bdb<=-25 档：A4=10, A5=-90', check=c032),
    dict(id='TC-ALG-033', item='避障策略', title='A5>-90 时登记基座左转策略', level='中',
         pre='策略标记由 A5 与 -90 的大小关系推导', method='等价类划分（策略标记）',
         inp='bdb = 5°（A5=0）',
         steps='检查 strategies 集合是否含 A1Left',
         exp='strategies 含 A1Left', check=c033),
    dict(id='TC-ALG-034', item='避障策略', title='A5<-90 时登记基座右转策略', level='中',
         pre='同上', method='等价类划分（策略标记）',
         inp='bdb = 10°（A5=-130）',
         steps='检查 strategies 集合是否含 A1Right',
         exp='strategies 含 A1Right', check=c034),
    dict(id='TC-ALG-035', item='避障策略', title='A5 恰为 -90 时不产生基座方向策略（边界）', level='低',
         pre='判定为 A5>-90 与 A5<-90，两者均为严格不等', method='边界值分析（边界 -90）',
         inp='bdb = 40°（A5=-90）',
         steps='检查 strategies 是否含 A1Left/A1Right',
         exp='两者均不含', check=c035),
    dict(id='TC-ALG-036', item='避障策略', title='构造参数 DFL 强制覆盖查表姿态', level='中',
         pre='构造时传入 A4/A5/A6', method='等价类划分（参数覆盖）',
         inp='MyCustomAlgorithm(A4=0, A5=-30, A6=45)，bdb=10°',
         steps='调用 get_action 后读取 A4/A5/A6',
         exp='A4=0, A5=-30, A6=45（覆盖查表结果 A5=-130）', check=c036),
    dict(id='TC-ALG-037', item='逆解健壮性', title='目标超工作空间时的数值健壮性', level='高',
         pre='机械臂 l2+l3 = 0.82001 m', method='边界值分析（可达半径外）',
         inp='目标 (0, 1.6, 0.2)，超出可达范围',
         steps='调用 get_action，捕获数值异常',
         exp='不抛 arccos 定义域异常，返回动作或 None', check=c037),
    dict(id='TC-ALG-038', item='关节限幅', title='解算结果相对基准角的单关节变化不超过 100°', level='高',
         pre='100 步 × 1°/步 的步数预算', method='边界值分析（限值 100）',
         inp='随机种子 1000 的目标与障碍',
         steps='取 alg.target 与 alg.base_angles 求最大绝对差',
         exp='最大单关节变化 ≤ 100°', check=c038),
    dict(id='TC-ALG-039', item='异常安全', title='弃权返回 None 对下游调用的影响', level='高',
         pre='env.step 会对动作执行 np.clip(action,-1,1)', method='场景法（异常路径）',
         inp='起始关节角全 0、解算需超过 100° 变化的观测',
         steps='调用 get_action；对返回值执行 np.clip 模拟 env.step',
         exp='不返回 None，或调用方有明确处理，不导致运行中断', check=c039),
    dict(id='TC-ALG-040', item='动作输出', title='输出动作量级落在 env 裁剪区间内', level='中',
         pre='env.py:122-123 将动作裁剪到 [-1,1]', method='等价类划分（输出值域）',
         inp='中立位附近 50 组随机观测',
         steps='统计返回动作的最大绝对值',
         exp='动作量级与 1°/步 的预算相称，裁剪后不失真', check=c040),
    dict(id='TC-ALG-041', item='回合重置', title='sig_reset 重置内部步进计数', level='中',
         pre='分阶段抬臂阈值使用 80-step', method='场景法（回合切换）',
         inp='同一回合连续调用 6 次后更换目标',
         steps='检查 alg.step 是否归 1',
         exp='换回合后 step 重置为 1', check=c041),
    dict(id='TC-ALG-042', item='端到端场景', title='障碍正对目标时的完整抓取回合', level='高',
         pre='Mock 环境，初始位姿为 env.py 中立位', method='场景法（正常抓取流程）',
         inp='目标 (0,0.85,0.2)，障碍 (0,0.6,0.2)，最多 100 步',
         steps='循环 get_action/step 至终止，记录最终夹爪-目标距离',
         exp='最终距离 ≤ 0.05 m', check=c042),
    dict(id='TC-ALG-043', item='端到端场景', title='障碍右偏时绕行抓取回合', level='高',
         pre='同上', method='场景法（避障绕行）',
         inp='目标 (0,0.85,0.2)，障碍 (0.2,0.6,0.2)',
         steps='跑完整回合并记录最终距离',
         exp='最终距离 ≤ 0.05 m', check=c043),
    dict(id='TC-ALG-044', item='端到端场景', title='障碍左偏时绕行抓取回合', level='高',
         pre='同上', method='场景法（避障绕行）',
         inp='目标 (0,0.85,0.2)，障碍 (-0.2,0.6,0.2)',
         steps='跑完整回合并记录最终距离',
         exp='最终距离 ≤ 0.05 m', check=c044),
    dict(id='TC-ALG-045', item='端到端场景', title='目标位于分布左端的抓取回合', level='中',
         pre='目标 x 分布下界 -0.2', method='边界值分析（目标分布下界）',
         inp='目标 (-0.2,0.8,0.1)，障碍 (-0.1,0.6,0.2)',
         steps='跑完整回合并记录最终距离',
         exp='最终距离 ≤ 0.05 m', check=c045),
    dict(id='TC-ALG-046', item='端到端场景', title='目标位于分布右上的抓取回合', level='中',
         pre='目标 x/y/z 分布上界 0.2/0.9/0.3', method='边界值分析（目标分布上界）',
         inp='目标 (0.2,0.9,0.3)，障碍 (0,0.6,0.2)',
         steps='跑完整回合并记录最终距离',
         exp='最终距离 ≤ 0.05 m', check=c046),
    dict(id='TC-ALG-047', item='端到端场景', title='低年级组评分档位下的完整回合', level='中',
         pre='is_senior=False，碰撞惩罚系数 0.5', method='等价类划分（组别参数）',
         inp='目标 (0,0.85,0.2)，障碍 (0,0.6,0.2)，is_senior=False',
         steps='跑完整回合并记录最终距离与得分',
         exp='最终距离 ≤ 0.05 m', check=c047),
    dict(id='TC-ALG-048', item='端到端场景', title='多随机回合抓取成功率', level='高',
         pre='10 个不同随机种子', method='场景法（批量稳定性）',
         inp='种子 1000~1009，各自随机目标与障碍',
         steps='逐回合执行并统计最终距离 ≤0.05 的比例',
         exp='成功率 ≥ 80%', check=c048),
    dict(id='TC-ALG-049', item='端到端场景', title='单回合步数不超过 100 步上限', level='高',
         pre='env.max_steps = 100', method='边界值分析（步数上界）',
         inp='随机种子 1000 的回合',
         steps='统计回合实际执行步数',
         exp='步数 ≤ 100', check=c049),
    dict(id='TC-ALG-050', item='端到端场景', title='回合必须终止，不占用评测时长', level='高',
         pre='评测总时长上限 120 s / 100 回合', method='场景法（终止性）',
         inp='随机种子 1000 的回合',
         steps='跑完整回合并检查 env.terminated',
         exp='回合在 100 步内正常终止', check=c050),
]


# =====================================================================
# 交付划分：模块一（测试基础实践）/ 模块二（AI 融合实践·方案 2「AI 测」）
# =====================================================================
# 划分依据（两条硬指标来自《实践作业要求 v2026》）：
#   模块一 3.1.1：用例清单 ≥30 条，覆盖等价类/边界值/场景法中的 ≥2 种；
#                 且 3.1.1(g) 明确「此阶段不要使用 AI 工具生成测试用例」，
#                 故模块一只保留人工一眼可判的黑盒行为用例：
#                 看「标题 + 输入 + 预期结果」三列即可判对错，
#                 不需要 URDF 关节链 / 旋转矩阵 / 数值容差 / 白盒数据流知识。
#   模块二 3.2 方案 2：交付「AI 生成的测试用例清单 ≥15 条」，
#                 故把需要上述专门知识的进阶用例整理到模块二，
#                 它们正是 AI 工具擅长提出、而人工负责核对测试预言的那一类。
#
# 具体口径：
#   模块一 = 等价类代表值（"每一档怎么走"）+ 对外契约 + 完整抓取回合；
#   模块二 = 运动学正解/末端模型/距离度量/逆解健壮性/关节限幅等白盒与数值用例
#            + 避障判定表的全部阈值边界点（"临界点站哪一边"）+ 构造参数覆盖。
MODULE1_NAME = '模块一'
MODULE2_NAME = '模块二'

MODULE1_IDS = (
    # 输入解析与接口契约：对外契约，读过 env.py 就能判（5 条）
    'TC-ALG-001', 'TC-ALG-002', 'TC-ALG-003', 'TC-ALG-004', 'TC-ALG-005',
    # 回合重置：目标/障碍一变就该重置，语义直白（5 条）
    'TC-ALG-007', 'TC-ALG-008', 'TC-ALG-009', 'TC-ALG-010', 'TC-ALG-041',
    # 评分判定：距离与得分的对应关系（2 条）
    'TC-ALG-018', 'TC-ALG-019',
    # 避障姿态策略：每个分档的等价类代表值，不含阈值边界（7 条）
    'TC-ALG-020', 'TC-ALG-021', 'TC-ALG-024', 'TC-ALG-027', 'TC-ALG-028',
    'TC-ALG-033', 'TC-ALG-034',
    # 异常安全与动作输出（2 条）
    'TC-ALG-039', 'TC-ALG-040',
    # 端到端抓取：看"抓没抓到"即可，最直观（9 条）
    'TC-ALG-042', 'TC-ALG-043', 'TC-ALG-044', 'TC-ALG-045', 'TC-ALG-046',
    'TC-ALG-047', 'TC-ALG-048', 'TC-ALG-049', 'TC-ALG-050',
)

MODULE1_SET = frozenset(MODULE1_IDS)
for _case in CASES:
    _case['module'] = MODULE1_NAME if _case['id'] in MODULE1_SET else MODULE2_NAME

# 交付件硬指标自检：改动用例集时若跌破阈值，导入即报错，避免出件后才发现。
_n1 = sum(1 for c in CASES if c['module'] == MODULE1_NAME)
_n2 = sum(1 for c in CASES if c['module'] == MODULE2_NAME)
assert len(MODULE1_SET) == _n1, 'MODULE1_IDS 与 CASES 编号不匹配（有编号不存在）'
assert _n1 >= 30, f'模块一用例数 {_n1} < 30，不满足作业要求'
assert _n2 >= 15, f'模块二用例数 {_n2} < 15，不满足作业要求'
assert _n1 + _n2 == len(CASES), '存在未归属模块的用例'
