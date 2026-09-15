# 种子杯 2024 机械臂避障抓取 —— 算法核心模块自动化测试

《软件测试与质量保证实践》小组作业（模块一：测试基础实践；模块二：AI 融合实践·方案 2「AI 测」）。
被测对象为 2024 种子杯「机械臂避障挑战」参赛工程的核心决策模块，本仓库同时是赛题工程与测试工程。

## 被测对象

- 赛题：在 PyBullet 仿真中控制 FR5 六轴机械臂避开球形障碍抓取圆柱目标，100 步内夹爪中心到目标中心 ≤ 0.05 m 判成功。
- 被测模块：`team_algorithm.py`（约 560 行）中的 `MyCustomAlgorithm` 类及其运动学辅助函数
  （`forward_kinematics_mat` 正运动学 / `abc_endeffe` 夹爪中心 / `abc_getdist` 距离度量 / `sig_reset` 避障姿态 / `get_action` 主入口），
  职责是「由观测解算六轴动作」。
- 运行环境：`env.py`（赛题仿真环境类）、`fr5_description/`（FR5 机械臂 URDF 与网格）。

## 目录结构

```
.
├── team_algorithm.py        # 被测模块（算法核心）
├── env.py                   # 赛题仿真环境（PyBullet）
├── test.py                  # 赛题官方端到端仿真（100 轮，需图形界面）
├── fr5_description/         # FR5 机械臂 URDF / 网格
├── tests/                   # 测试工程
│   ├── cases_spec.py        # 54 条用例的规格与检查函数（数据驱动，单一来源）
│   ├── test_cases.py        # pytest 入口：把用例参数化成 54 个测试项
│   ├── conftest.py          # OK/POK/NG/NT 判定映射，会话结束写出 results.json
│   ├── run_cases.py         # 一键执行器：调 pytest 后按交付模块汇总
│   ├── kinematics_ref.py    # 独立参考运动学（测试预言的真值来源）
│   ├── mock_env.py          # 复现 env.py 语义的 Mock 环境（免 PyBullet）
│   ├── make_excel.py        # 把 results.json 回填《附录1》模板，生成交付件
│   └── results.json         # 测试结果中间产物（由 pytest 写出，非源文件）
├── requirements.txt
├── pytest.ini
├── 附录1：测试用例清单（模块一）.xlsx   # 交付件：34 条人工设计黑盒用例
├── 附录1：测试用例清单（模块二）.xlsx   # 交付件：20 条 AI 扩展进阶用例
└── 附录3：测试报告（模块一）.docx
```

## 环境配置

需要 Python 3.13。用 uv 创建虚拟环境并安装依赖：

```bash
uv venv
uv pip install -r requirements.txt
```

依赖：`numpy < 2`、`scipy`、`pybullet`、`stable_baselines3`、`pytest`。
没有编译环境的成员可先在其机器上运行 `python export_wheels.py <目录>` 导出 wheel，再离线安装。

## 测试运行

一键跑全部 54 条用例（推荐）：

```bash
.venv/bin/python tests/run_cases.py
```

也可直接用 pytest（配置见 `pytest.ini`）：

```bash
.venv/bin/python -m pytest                 # 全部 54 条
.venv/bin/python -m pytest -m module1      # 只跑模块一 34 条
.venv/bin/python -m pytest -m module2      # 只跑模块二 20 条
.venv/bin/python -m pytest -k TC-ALG-042   # 只跑某一条
```

执行后逐条判定（OK / POK / NG / NT）写入 `tests/results.json`。
测试全部基于「纯逻辑 + Mock 替身」，不依赖图形界面，任何机器上均可一键复现。

## 交付件生成

```bash
.venv/bin/python tests/make_excel.py      # 依据 results.json 生成 附录1（模块一/二）.xlsx
```

## 用例划分

- **模块一（测试基础实践）34 条**：人工设计、看「标题 + 输入 + 预期结果」即可判定的黑盒行为用例
  （输入解析与接口契约、回合重置、评分判定、避障档位代表值、异常安全、动作输出、端到端抓取回合），
  覆盖等价类划分、边界值分析、场景法三种方法。
- **模块二（AI 融合实践·方案 2）20 条**：在模块一基础上由 AI 工具扩展生成的进阶用例
  （运动学正解、末端夹爪中心模型、距离度量、避障判定表的全部阈值边界点、逆解数值健壮性与关节限幅），
  判定需 URDF 关节链 / 旋转矩阵 / 数值容差方面的专门知识。归属见 `tests/cases_spec.py` 的 `MODULE1_IDS`。

## 备注

- `results.json` 由 pytest 执行后派生，改动它不影响测试内容。
- 以 `test.py` 运行真实 PyBullet 端到端仿真时需图形界面，与 `tests/` 的 Mock 执行相互独立。
- `team_algorithm_60.py` / `team_algorithm_97.py` 为存档的历史提交版本。
