# -*- coding: utf-8 -*-
"""把自动化测试结果回填进《附录1：测试用例清单》模板，生成正式交付件。

数据来源：
    tests/results.json —— 由 pytest 执行 cases_spec.CASES 后写出（见 conftest.py）
模板来源：
    tests/附录1：测试用例清单模板.xlsx

输出（按交付模块各出一份，归属见 cases_spec.MODULE1_IDS）：
    项目根目录\\附录1：测试用例清单（模块一）.xlsx   30 条，人工设计的黑盒用例
    项目根目录\\附录1：测试用例清单（模块二）.xlsx   20 条，AI 扩展生成的进阶用例

模板 Information 页的"测试用例数"与 OK/POK/NG/NT 项均为公式，按 L 列自动统计，
分模块出件后无需手工改数。
"""
from __future__ import annotations

import copy
import datetime
import io
import json
import os
import sys

import openpyxl

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TEMPLATE = os.path.join(HERE, '附录1：测试用例清单模板.xlsx')
RESULTS = os.path.join(HERE, 'results.json')

SHEET_INFO = 'Information文档信息'
SHEET_CASES = 'Test Cases测试用例'
FIRST_DATA_ROW = 2

# 重要级别：模板要求 High / Medium / Low
LEVEL_MAP = {'高': 'High', '中': 'Medium', '低': 'Low'}


def method_tag(method: str) -> str:
    if method.startswith('等价类'):
        return '等价类划分'
    if method.startswith('边界值'):
        return '边界值分析'
    if method.startswith('场景法'):
        return '场景法'
    if method.startswith('白盒'):
        return '白盒分析'
    return '其他'


# 测试预言与测试替身的说明，两份交付件共用
_ORACLE = (
    '测试预言（test oracle）不依赖被测代码，而由三处独立来源构造：'
    '① fr5_description/urdf/fr5v6.urdf 的关节链偏移与关节限位；'
    '② env.py 对外契约（观测归一化、step 动作语义、reward 判定式、get_dis 夹爪中心定义）；'
    '③ team_algorithm.py 末尾由 env.debugdoor() 打印的真实 PyBullet 输出（footnote）。\n'
    '用例全部以"纯逻辑测试 + Mock 替身"实现，由 pytest 驱动执行：'
    '以严格按 URDF 重建的参考运动学（tests/kinematics_ref.py）作为位置真值，'
    '以复现 env.py 语义的 Mock 环境（tests/mock_env.py）替代 PyBullet 物理引擎，'
    '故不依赖图形界面、任何机器上均可一键复现；'
    '真实 PyBullet（conda-forge pybullet 3.2.5）另用于 test.py 的端到端对照运行。'
)

OVERVIEW_1 = (
    '本清单为【模块一·测试基础实践】交付件，针对 2024 种子杯机械臂避障抓取赛项的'
    '"算法核心模块"——team_algorithm.py 的 MyCustomAlgorithm 类及其运动学辅助函数'
    '（约 560 行，含 forward_kinematics_mat / abc_endeffe / abc_getdist / sig_reset / '
    'get_action 等核心函数），在系统中承担"由观测解算六轴动作"的决策职责。\n'
    '共 30 条用例，全部由小组人工设计并自动化执行（本阶段按要求不使用 AI 工具生成用例），'
    '覆盖等价类划分、边界值分析与场景法三种方法。用例只取材于对外可观测的行为——'
    '输入解析、接口契约、回合重置、评分判定、避障姿态分档代表值、异常安全、动作输出'
    '与端到端抓取回合；每条用例的标题、输入与预期结果均可直接阅读判断，'
    '不需要 URDF 关节链或旋转矩阵方面的推导。\n'
    '需要上述专门知识的进阶用例（运动学正解、末端夹爪中心模型、避障判定表的全部阈值'
    '边界点、逆解数值健壮性与关节限幅等），另见《附录1：测试用例清单（模块二）》。\n'
    + _ORACLE
)

OVERVIEW_2 = (
    '本清单为【模块二·AI 融合实践（方案 2"AI 测"）】交付件，被测对象与模块一相同：'
    '2024 种子杯机械臂避障抓取赛项的"算法核心模块"——team_algorithm.py 的 '
    'MyCustomAlgorithm 类及其运动学辅助函数。\n'
    '共 20 条用例，是在模块一 30 条的基础上借助 AI 工具扩展生成的：'
    '由 AI 提出用例方向、构造输入观测、推演预期值并生成检查函数，'
    '小组逐条核对测试预言，剔除与 URDF / env.py 契约不符的臆测。'
    '用例集中在模块一未覆盖的深水区——运动学正解与连杆姿态、末端夹爪中心模型、'
    '距离度量的代数性质、避障判定表的全部阈值边界点、构造参数的白盒覆盖、'
    '逆解数值健壮性与关节限幅；判定每一条都需要 URDF 关节链、旋转矩阵'
    '或数值容差方面的专门知识。\n'
    'AI 辅助与人工修正的关键差异、以及 AI 输出中的错误与遗漏'
    '（典型如 TC-ALG-036 因被测接口变更而失效），见《附录4：测试报告（模块二）》。\n'
    + _ORACLE
)

# 交付模块 -> 出件配置
MODULES = (
    {
        'name': '模块一',
        'filename': '附录1：测试用例清单（模块一）.xlsx',
        'project_id': 'SEEDCUP-2024-ALG-M1',
        'project_name': '种子杯 2024 机械臂避障抓取——算法核心模块自动化测试（模块一）',
        'overview': OVERVIEW_1,
        'revision_note': 'initial 初稿完成：依据 URDF/env.py/footnote 构造测试预言，'
                         '人工设计 30 条用例并以 pytest 自动化执行',
    },
    {
        'name': '模块二',
        'filename': '附录1：测试用例清单（模块二）.xlsx',
        'project_id': 'SEEDCUP-2024-ALG-M2',
        'project_name': '种子杯 2024 机械臂避障抓取——算法核心模块自动化测试（模块二·AI 融合）',
        'overview': OVERVIEW_2,
        'revision_note': 'initial 初稿完成：在模块一基础上由 AI 扩展生成 20 条进阶用例，'
                         '人工逐条核对测试预言后以 pytest 自动化执行',
    },
)


def load_results() -> list[dict]:
    with open(RESULTS, encoding='utf-8') as f:
        data = json.load(f)
    return data['cases']


def widen_for_readability(ws) -> None:
    """模板列宽按原设计留白很少，填入中文后会被相邻单元格裁掉，这里放宽并换行。"""
    for col, width in (('B', 16), ('C', 13), ('D', 10), ('E', 46),
                       ('G', 24), ('H', 4), ('J', 14)):
        ws.column_dimensions[col].width = width
    for coord in ('E7', 'E8', 'E9', 'E10', 'E11', 'J8', 'J9', 'J10', 'J11'):
        ws[coord].alignment = openpyxl.styles.Alignment(
            horizontal='left', vertical='center', wrap_text=True)
    # 概述段落：模板未合并该行，不合并则只能挤在 B 列里显示
    ws.merge_cells('B16:M16')
    ws['B16'].alignment = openpyxl.styles.Alignment(
        horizontal='left', vertical='top', wrap_text=True)
    ws.row_dimensions[16].height = 132


def fill_information(ws, module: dict, total: int, today: str) -> None:
    ws['E7'] = 'v1.0（2024 种子杯参赛提交版本）'
    ws['E8'] = module['project_name']
    ws['J8'] = module['project_id']
    ws['E9'] = '（待填写）'
    ws['J9'] = today
    ws['E10'] = '（待填写）'
    ws['J10'] = today
    ws['E11'] = '（待填写）'
    ws['J11'] = today
    ws['B16'] = module['overview']
    ws['B20'] = today
    ws['C20'] = '1.00'
    ws['E20'] = module['revision_note']


def write_module(module: dict, cases: list[dict], today: str) -> dict:
    wb = openpyxl.load_workbook(TEMPLATE)
    ws = wb[SHEET_CASES]

    # 以第 2 行作为数据行样式样板（模板已预置边框、自动换行、Times New Roman 11）
    proto = [ws.cell(row=FIRST_DATA_ROW, column=col) for col in range(1, 14)]

    for i, case in enumerate(cases):
        row = FIRST_DATA_ROW + i
        values = [
            case['id'],                                   # A 测试用例编号
            case['item'],                                 # B 测试项
            case['title'],                                # C 测试用例标题
            LEVEL_MAP.get(case['level'], case['level']),  # D 重要级别
            '是',                                          # E 是否自动用例
            '新增',                                        # F 是否新增修改用例
            case['pre'],                                  # G 预置条件
            case['input'],                                # H 输入
            case['steps'],                                # I 操作步骤
            case['expected'],                             # J 预期结果
            case['actual'],                               # K 实际结果
            case['status'],                               # L 是否通过
            case['method'],                               # M 备注（测试方法）
        ]
        for col, value in enumerate(values, start=1):
            src = proto[col - 1]
            dst = ws.cell(row=row, column=col)
            dst.value = value
            dst._style = copy.copy(src._style)
            dst.alignment = copy.copy(src.alignment)

    widen_for_readability(wb[SHEET_INFO])
    fill_information(wb[SHEET_INFO], module, len(cases), today)

    out = os.path.join(ROOT, module['filename'])
    wb.save(out)

    counts = {k: sum(1 for c in cases if c['status'] == k) for k in ('OK', 'POK', 'NG', 'NT')}
    tags: dict[str, int] = {}
    for case in cases:
        key = method_tag(case['method'])
        tags[key] = tags.get(key, 0) + 1

    print(f'已生成 {out}')
    print(f'  {module["name"]} 用例 {len(cases)} 条：'
          f'OK={counts["OK"]} POK={counts["POK"]} NG={counts["NG"]} NT={counts["NT"]}   '
          f'OK 率 {counts["OK"] / len(cases) * 100:.1f}%')
    print('  测试方法覆盖：' + '、'.join(f'{k} {v} 条' for k, v in sorted(tags.items())))
    return counts


def main() -> int:
    rows = load_results()
    today = datetime.date.today().strftime('%Y.%m.%d')

    seen = {r['module'] for r in rows}
    missing = [m['name'] for m in MODULES if m['name'] not in seen]
    if missing:
        names = '、'.join(missing)
        print(f'results.json 中缺少 {names} 的用例，'
              f'请先执行 python tests/run_cases.py 跑完全部用例', file=sys.stderr)
        return 1

    for module in MODULES:
        cases = [r for r in rows if r['module'] == module['name']]
        write_module(module, cases, today)
    return 0


if __name__ == '__main__':
    sys.exit(main())
