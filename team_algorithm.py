import numpy as np
import numpy
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Literal, Tuple, Dict, Any, Optional, Set
from numpy.typing import NDArray

DESIRE = 101

# from pprint import pprint

numpy.set_printoptions(precision=6, suppress=True, sign="+", floatmode="fixed")

A = numpy.array
Pos = NDArray[np.float64]


@dataclass
class JointOrigin:
    xyz: Pos
    rpy: Pos


@dataclass
class Joint:
    name: str
    origin: JointOrigin


@dataclass
class LinkOrigin:
    xyz: Pos
    rpy: Pos


@dataclass
class Link:
    name: str
    origin: LinkOrigin


@dataclass
class Robot:
    links: List[Link]
    joints: List[Joint]


robot = Robot(
    links=[
        # Link(name="world", origin=LinkOrigin(xyz=A([0, 0, 0]), rpy=A([0, 0, 0]))),
        # Link(
        #     name="base_link",
        #     origin=LinkOrigin(
        #         xyz=A([-0.00031896, -0.00029673, 0.042463]), rpy=A([0, 0, 0])
        #     ),
        # ),
        Link(name="j1_Link", origin=LinkOrigin(xyz=A([0, 0, 0]), rpy=A([0, 0, 0]))),
        Link(
            name="j2_Link",
            origin=LinkOrigin(xyz=A([-0.2125, -5.7643e-09, 0.1346]), rpy=A([0, 0, 0])),
        ),
        Link(
            name="j3_Link",
            origin=LinkOrigin(
                xyz=A([-0.18793, -8.4503e-07, 0.0066357]), rpy=A([0, 0, 0])
            ),
        ),
        Link(
            name="j4_Link",
            origin=LinkOrigin(xyz=A([4.98e-07, -0.003754, 0.097155]), rpy=A([0, 0, 0])),
        ),
        Link(
            name="j5_Link",
            origin=LinkOrigin(
                xyz=A([-4.5588e-07, 0.0038617, 0.098257]), rpy=A([0, 0, 0])
            ),
        ),
        Link(
            name="j6_Link",
            origin=LinkOrigin(
                xyz=A([7.7496e-05, 1.7751e-05, 0.076122]), rpy=A([0, 0, 0])
            ),
        ),
        Link(
            name="hand_base_link",
            origin=LinkOrigin(xyz=A([1.045, -0.01, -0.1]), rpy=A([0, 0, 0])),
        ),
        # Link(
        #     name="finger_link1",
        #     origin=LinkOrigin(
        #         xyz=A([-0.0765920818887161, 0.0785524897759977, 0.00485564932597477]),
        #         rpy=A([0, 0, 0]),
        #     ),
        # ),
        # Link(
        #     name="finger_link2",
        #     origin=LinkOrigin(
        #         xyz=A([-0.0765920818887161, 0.0800524897759977, 0]), rpy=A([0, 0, 0])
        #     ),
        # ),
    ],
    joints=[
        # Joint(name="world", origin=JointOrigin(xyz=A([0, 0, 0]), rpy=A([0, 0, 0]))),
        Joint(name="j1", origin=JointOrigin(xyz=A([0, 0, 0]), rpy=A([0, 0, 0]))),
        Joint(
            name="j2", origin=JointOrigin(xyz=A([0, 0, 0.152]), rpy=A([1.5708, 0, 0]))
        ),
        Joint(name="j3", origin=JointOrigin(xyz=A([-0.425, 0, 0]), rpy=A([0, 0, 0]))),
        Joint(name="j4", origin=JointOrigin(xyz=A([-0.39501, 0, 0]), rpy=A([0, 0, 0]))),
        Joint(
            name="j5", origin=JointOrigin(xyz=A([0, 0, 0.1021]), rpy=A([1.5708, 0, 0]))
        ),
        Joint(
            name="j6", origin=JointOrigin(xyz=A([0, 0, 0.102]), rpy=A([-1.5708, 0, 0]))
        ),
        Joint(
            name="arm_hand_joint",
            origin=JointOrigin(xyz=A([0, 0, 0.12]), rpy=A([0, 0, 3.14159])),
        ),
        # Joint(
        #     name="fj1", origin=JointOrigin(xyz=A([0.0445, 0, 0]), rpy=A([1.5708, 0, 0]))
        # ),
        # Joint(
        #     name="fj2",
        #     origin=JointOrigin(
        #         xyz=A([-0.0425, 0.0087157, -0.0015]), rpy=A([1.5708, 0, 3.1416])
        #     ),
        # ),
    ],
)


# from urdfpy import URDF

sin = np.sin
cos = np.cos

PMat = NDArray[np.float64]
RMat = NDArray[np.float64]


def turnX(Rx: float) -> RMat:
    return np.array(
        [
            [1, 0, 0],
            [0, cos(Rx), sin(Rx)],
            [0, -sin(Rx), cos(Rx)],
        ]
    )


def turnY(Ry: float) -> RMat:
    return np.array(
        [
            [cos(Ry), 0, -sin(Ry)],
            [0, 1, 0],
            [sin(Ry), 0, cos(Ry)],
        ]
    )


def turnZ(Rz: float) -> RMat:
    return np.array(
        [
            [cos(Rz), sin(Rz), 0],
            [-sin(Rz), cos(Rz), 0],
            [0, 0, 1],
        ]
    )


# def forward_kinematics(
#     robot: Robot, joint_angles: NDArray[np.float64], i: int, link: bool = True
# ) -> Tuple[PMat, RMat]:
#     """
#     根据URDF文件和关节角度计算关节和连接的位置
#     """
#     P: PMat = np.zeros(3)
#     R: RMat = np.eye(3)
#     link = robot.links[i].origin
#     if link:
#         P = P @ turnX(link.rpy[0]) @ turnY(link.rpy[1]) @ turnZ(link.rpy[2])
#         R = R @ turnX(link.rpy[0]) @ turnY(link.rpy[1]) @ turnZ(link.rpy[2])
#         P = P + link.xyz
#     for j in range(i, -1, -1):
#         joint = robot.joints[j].origin
#         ang = joint_angles[j]
#         P = P @ turnZ(ang)
#         R = R @ turnZ(ang)
#         P = P @ turnX(joint.rpy[0]) @ turnY(joint.rpy[1]) @ turnZ(joint.rpy[2])
#         R = R @ turnX(joint.rpy[0]) @ turnY(joint.rpy[1]) @ turnZ(joint.rpy[2])
#         P = P + joint.xyz
#     P = P @ turnZ(np.pi)
#     R = R @ turnZ(np.pi)
#     # print(f"Test {i+1}")
#     # print(P)
#     # print(R)
#     return (P, R)


def forward_kinematics_mat(
    robot: Robot, joint_angles: NDArray[np.float64], i: int
) -> List[Tuple[PMat, RMat, PMat, RMat]]:
    """
    根据URDF文件和关节角度计算关节和连接的位置
    """
    ret: List[Tuple[PMat, RMat, PMat, RMat]] = []
    P: PMat = np.zeros(3)
    R: RMat = np.eye(3)
    R = turnZ(np.pi) @ R
    for j in range(i + 1):
        joint = robot.joints[j].origin
        ang = joint_angles[j]
        P = P + joint.xyz @ R
        R = turnX(joint.rpy[0]) @ turnY(joint.rpy[1]) @ turnZ(joint.rpy[2]) @ R
        R = turnZ(ang) @ R

        link = robot.links[j].origin
        P0 = P + link.xyz @ R
        R0 = turnX(link.rpy[0]) @ turnY(link.rpy[1]) @ turnZ(link.rpy[2]) @ R
        # print(f"Test {j+1}")
        # print(P)
        # print(R)
        # print(P0)
        # print(R0)
        ret.append((P, R, P0, R0))
    return ret


class BaseAlgorithm(ABC):
    @abstractmethod
    def get_action(self, observation: NDArray[np.float64]) -> NDArray[np.float64]:
        """
        输入观测值，返回动作
        Args:
            observation: numpy array of shape (1, 12) 包含:
                - 6个关节角度 (归一化到[0,1])
                - 3个目标位置坐标
                - 3个障碍物位置坐标
        Returns:
            action: numpy array of shape (6,) 范围在[-1,1]之间
        """
        pass


class MyCustomAlgorithm(BaseAlgorithm):
    def __init__(self) -> None:
        # 自定义初始化
        self.target: Optional[NDArray[np.float64]] = None
        self.xyz_target: Optional[NDArray[np.float64]] = None
        self.A4 = 0
        self.A5 = 0
        self.A6 = 0
        self.Muli = 0
        self.strategies: Set[str] = set()
        self.design_target = np.zeros(3)
        self.ball_target = np.zeros(3)
        self.base_angles = np.zeros(6)
        self.step = 0

    def rot_coord(self, xyz: NDArray[np.float64]) -> NDArray[np.float64]:
        x, y, z = xyz
        a = np.sqrt(x**2 + y**2)
        b = np.arctan2(y, x) * 180 / np.pi
        c = z
        return A([a, b, c])

    def sig_reset(self) -> None:
        self.A4 = +10
        self.A5 = -90
        self.A6 = 0
        self.step = 0
        # return
        self.Muli = 0.2 - (DESIRE / 100 * 0.15)
        br = self.rot_coord(self.ball_target)
        dr = self.rot_coord(self.design_target)
        bdiff = br - dr
        bda, bdb, bdc = bdiff
        self.strategies.clear()
        if bdb > 25:
            self.A5 = -90
            self.strategies.add("A1Right")
        elif bdb > 15:
            self.A5 = -120
            self.strategies.add("A1Right")
        elif bdb > 8:
            self.A4 = 0
            self.A5 = -130
            self.strategies.add("A1Right")
        elif bdb > 6:
            self.A4 = -5
            self.A5 = -140
            self.strategies.add("A1Right")
        elif bdb > 3:
            if bda > -0.28:
                self.A4 = +40
                self.A5 = 0  # !
                self.strategies.add("A1Left")
            else:
                self.A4 = +15
                self.A5 = -20
                self.strategies.add("A1Left")
        elif bdb > -5:
            self.A4 = +15
            self.A5 = -20
            self.strategies.add("A1Left")
        elif bdb > -15:
            self.A5 = -20
            self.strategies.add("A1Left")
        elif bdb > -25:
            self.A5 = -50
            self.strategies.add("A1Left")
        else:
            self.A5 = -90
            self.strategies.add("A1Left")
        self.strategies.add("A2Up")
        self.strategies.add("A3Up")
        self.strategies.add("A4Up")
        print(A([bda, bdb, bdc]), A([self.A4, self.A5, self.A6]), self.strategies)

    def get_action(
        self, observation: NDArray[np.float64], env: Any = None
    ) -> NDArray[np.float64]:
        # 输入观测值，返回动作
        args = observation[0][:6]
        angles = (args * 2 - 1) * 180
        design_target = observation[0][6:9]
        ball_target = observation[0][9:12]
        if (design_target != self.design_target).any() or (
            ball_target != self.ball_target
        ).any():
            self.base_angles = angles
            self.design_target = design_target
            self.ball_target = ball_target
            self.sig_reset()

        # 暂时不动先。
        if self.target is None:
            self.target = angles
        if self.xyz_target is None or True:
            self.xyz_target = design_target

        if True:
            if env is not None:
                keys = env.p.getKeyboardEvents()
                v = [self.A4, self.A5, self.A6]
                for j, k in zip(range(3), "123"):
                    if ord(k) in keys and keys[ord(k)] & env.p.KEY_IS_DOWN:
                        if ord("9") in keys and keys[ord("9")] & env.p.KEY_IS_DOWN:
                            self.xyz_target[j] += -0.005
                        if ord("0") in keys and keys[ord("0")] & env.p.KEY_IS_DOWN:
                            self.xyz_target[j] += 0.005
                for j, k in zip(range(3), "456"):
                    if ord(k) in keys and keys[ord(k)] & env.p.KEY_IS_DOWN:
                        if ord("9") in keys and keys[ord("9")] & env.p.KEY_IS_DOWN:
                            v[j] += -1
                        if ord("0") in keys and keys[ord("0")] & env.p.KEY_IS_DOWN:
                            v[j] += 1
                if ord("8") in keys and keys[ord("8")] & env.p.KEY_IS_DOWN:
                    print(A(v))
                self.A4, self.A5, self.A6 = v

            A4 = self.A4
            A5 = self.A5
            A6 = self.A6
            vbase_mats = forward_kinematics_mat(
                robot, A([0.0, 0.0, 0.0, A4, A5, A6, 0.0]) * np.pi / 180, 6
            )
            relative_position = A([0, 0, 0.15 + self.Muli])
            vbase_endpos = vbase_mats[5][2] + relative_position @ vbase_mats[6][3]
            # print(vbase_endpos)
            # print(vbase_mats[0][0])
            # print(vbase_mats[1][0])
            # print(vbase_mats[2][0])
            # print(vbase_mats[3][0])
            # print(vbase_mats[4][0])
            vp4 = vbase_mats[3][0]
            p4eoffs = vbase_endpos - vp4
            pex, pey, pez = self.xyz_target
            # p4eoffs = numpy.zeros(3)
            p4ex, p4ey, p4ez = p4eoffs
            # print("p4of", p4eoffs)
            #! p4 = self.xyz_target - p4eoffs

            p4z = pez - p4ez

            pec = np.sqrt(pex**2 + pey**2)
            # 腕心横向偏置 p4ey 由当前姿态决定。目标水平投影距离 pec 与之相当
            # （即目标贴近基座竖直轴线）时，平面上该构型无解，开方项为负，
            # NaN 会沿 a1~a4 一路传播出去。此处退化为保持姿态：返回零动作。
            if pec**2 - p4ey**2 < 0.1 * p4ey**2:
                self.step += 1
                return np.zeros(6)
            p4l = np.sqrt(pec**2 - p4ey**2) + p4ex
            p4a = (
                180
                + (np.arctan2(pey, pex) - np.arctan2(-p4ey, -p4ex + p4l)) * 180 / np.pi
            )

            # print("p4l", p4l)
            # print("p4a", p4a)

            a1 = p4a

            p23x = p4l
            p23y = p4z - robot.joints[1].origin.xyz[2]

            # a1 = 180
            # p23x = 0.5
            # p23y = 0.0

            l2 = -robot.joints[2].origin.xyz[0]
            l3 = -robot.joints[3].origin.xyz[0]
            if p23x**2 + p23y**2 > (l2 + l3) ** 2 - 0.0001:
                k = np.sqrt((p23x**2 + p23y**2) / ((l2 + l3) ** 2))
                p23x = p23x / k - 0.0001
                p23y = p23y / k - 0.0001
            a2p = p23x**2 + p23y**2 + l2**2 - l3**2
            a2q = 2 * l2 * np.sqrt(p23x**2 + p23y**2)
            A2 = (np.arctan2(p23y, p23x) + np.arccos(a2p / a2q)) * 180 / np.pi
            A3 = (
                np.arctan2(
                    p23y - l2 * np.sin(A2 * np.pi / 180),
                    p23x - l2 * np.cos(A2 * np.pi / 180),
                )
                * 180
                / np.pi
            )
            a2 = 180 + A2
            a3 = (180 + A3) - a2
            # print("yx", p23y, p23x)
            # print("l23", l2, l3)
            # print("a23", A2, A3)

            a4 = A4 - a2 - a3
            a5 = A5
            a6 = A6

            a1 = (a1 + 180) % 360 - 180
            a2 = (a2 + 270) % 360 - 270
            a3 = (a3 + 180) % 360 - 180
            a4 = (a4 + 270) % 360 - 270
            a5 = (a5 + 180) % 360 - 180
            a6 = (a6 + 180) % 360 - 180

            if "A1Left" in self.strategies:
                aa1 = angles[0]
                Aa1 = aa1 - a1
                # print("A1left", aa1, a1, Aa1, self.step)
                if abs(Aa1) < 90 - self.step:
                    # print("A1left yes")
                    a1 = +180
                    pass
            if "A1Right" in self.strategies:
                aa1 = angles[0]
                Aa1 = aa1 - a1
                if abs(Aa1) < 90 - self.step:
                    a1 = -180
                    pass
            if "A2Up" in self.strategies:
                aa2 = angles[1]
                Aa2 = aa2 - a2
                if abs(Aa2) < 85 - self.step:
                    a2 = 90
                    pass
            if "A3Up" in self.strategies:
                aa3 = angles[2]
                Aa3 = aa3 - a3
                if abs(Aa3) < 85 - self.step:
                    a3 = 180
                    pass
            if "A4Up" in self.strategies:
                aa4 = angles[3]
                Aa4 = aa4 - a4
                if abs(Aa4) < 85 - self.step:
                    a4 = 180
                    pass
            # a1=0.0
            # a2=0.0
            # a3=0.0
            # a4=A4
            self.target = A([a1, a2, a3, a4, a5, a6])
            # print(self.target)
        else:
            if env is not None:
                keys = env.p.getKeyboardEvents()
                for j, k in zip(range(6), "123456"):
                    if ord(k) in keys and keys[ord(k)] & env.p.KEY_IS_DOWN:
                        if ord("9") in keys and keys[ord("9")] & env.p.KEY_IS_DOWN:
                            self.target[j] += -1
                        if ord("0") in keys and keys[ord("0")] & env.p.KEY_IS_DOWN:
                            self.target[j] += 1
        self.target = A([a1, a2, a3, a4, a5, a6])

        diff = self.target - self.base_angles
        # print("## ", A([A4, A5, A6]))
        # print(A([self.target, angles, diff]))
        # print(self.target)
        # print(self.base_angles)
        diff = np.clip(diff, -100.0, 100.0)
        self.target = self.base_angles + diff

        vbase_mats = forward_kinematics_mat(
            robot, A([*(self.target * np.pi / 180), 0.0]), 6
        )

        # for i in range(0, 7):
        #     P,R = forward_kinematics(robot, tuple(angles * np.pi / 180) + (0.0,), i)
        #     print(f"Test {i+1}")
        #     print(P)
        #     print(R)

        # print("angles", self.target)

        mats = forward_kinematics_mat(robot, A([*(self.target * np.pi / 180), 0.0]), 6)
        # for i, (P, R, P0, R0) in enumerate(mats):
        #     print(f"Fast {i+1}")
        #     print(P)
        #     print(R)
        #     print(P0)
        #     print(R0)

        # print("gripper_centre_po!", self.abc_endeffe(self.target))
        # print("xyz_target", self.xyz_target)

        action = self.target - angles
        # 契约兜底：env.py:122 的 np.clip 不拦截 NaN，而 NaN 动作会让 PyBullet
        # 的四元数归一化抛 ValueError 直接中断整场评测，任何情况下都不许下发。
        action = np.nan_to_num(action, nan=0.0, posinf=0.0, neginf=0.0)
        actionmax = np.max(np.abs(action))
        # print(self.Muli, actionmax, action)
        if actionmax > 1:
            # action /= actionmax
            pass

        self.step += 1
        return action

    def abc_endeffe(self, angles: NDArray[np.float64]) -> NDArray[np.float64]:
        mats = forward_kinematics_mat(robot, A([*(angles * np.pi / 180), 0.0]), 6)
        relative_position = A([0, 0, 0.15])
        end_effector_position = mats[5][2] + relative_position @ mats[6][3]
        return end_effector_position

    def abc_getdist(
        self, angles: NDArray[np.float64], target: NDArray[np.float64]
    ) -> float:
        diff = self.abc_endeffe(angles) - target
        return sum(diff * diff)


footnote = """\
Link 1
[+0.000000 +0.000000 +0.000000]
[[-1.000000 -0.000000 +0.000000]
 [+0.000000 -1.000000 +0.000000]
 [+0.000000 +0.000000 +1.000000]]
Link 2
[+0.000000 +0.000000 +0.152000]
[[-1.000000 +0.000000 +0.000000]
 [+0.000000 +0.000004 +1.000000]
 [-0.000000 +1.000000 -0.000004]]
Link 3
[+0.425000 +0.000000 +0.152000]
[[-1.000000 +0.000000 +0.000000]
 [+0.000000 +0.000004 +1.000000]
 [-0.000000 +1.000000 -0.000004]]
Link 4
[+0.820010 +0.000000 +0.152000]
[[-1.000000 +0.000000 +0.000000]
 [+0.000000 +0.000004 +1.000000]
 [-0.000000 +1.000000 -0.000004]]
Link 5
[+0.820010 +0.102100 +0.152000]
[[-1.000000 +0.000000 -0.000000]
 [+0.000000 +1.000000 -0.000007]
 [+0.000000 -0.000007 -1.000000]]
Link 6
[+0.820010 +0.102099 +0.050000]
[[-1.000000 +0.000000 +0.000000]
 [+0.000000 +0.000004 +1.000000]
 [-0.000000 +1.000000 -0.000004]]
Link 7
[+0.820010 +0.222099 +0.049999]
[[+1.000000 +0.000003 -0.000000]
 [+0.000000 -0.000004 +1.000000]
 [+0.000003 -1.000000 -0.000004]]\
"""

if __name__ == "__main__":
    obs = A([[0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.0, 0.7, 0.2, 0.0, 0.4, 0.5]])
    MyCustomAlgorithm().get_action(obs, None)
    # print(footnote)
