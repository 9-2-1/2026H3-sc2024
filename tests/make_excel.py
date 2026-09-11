# -*- coding: utf-8 -*-
"""把自动化测试结果回填进《附录1：测试用例清单》模板，生成正式交付件。

数据来源：
    tests/results.json   —— 由 tests/run_cases.py 执行 cases_spec.CASES 后写出
模板来源：
    E:\\yanjiusheng\\suance course\\实践作业-文档模板2026\\附录1：测试用例清单模板.xlsx

输出：
    项目根目录\\附录1：测试用例清单.xlsx
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
OUTPUT = os.path.join(ROOT, '附录1：测试用例清单.xlsx')
RESULTS = os.path.join(HERE, 'results.json')

SHEET_INFO = 'Information文档信息'
SHEET_CASES = 'Test Cases测试用例'
FIRST_DATA_ROW = 2

# 重要级别：模板要求 High / Medium / Low
LEVEL_MAP = {'高': 'High', '中': 'Medium', '低': 'Low'}

# 测试方法归类（备注列同时保留细分说明）
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


OVERVIEW = (
    '本次测试针对 2024 种子杯机械臂避障抓取赛项中的“算法核心模块”——'
    'team_algorithm.py 的 MyCustomAlgorithm 类及其运动学辅助函数。\n'
    '测试预言（test oracle）不依赖被测代码，而由三处独立来源构造：'
    '① fr5_description/urdf/fr5v6.urdf 的关节链偏移与关节限位；'
    '② env.py 对外契约（观测归一化、step 动作语义、reward 判定式、get_dis 夹爪中心定义）；'
    '③ team_algorithm.py 末尾由 env.debugdoor() 打印的真实 PyBullet 输出（footnote）。\n'
    '受运行环境限制（pybullet 未提供 Python 3.14 轮子），采用纯逻辑测试 + Mock 方案：'
    '以严格按 URDF 重建的参考运动学（tests/kinematics_ref.py）作为位置真值，'
    '以复现 env.py 语义的 Mock 环境（tests/mock_env.py）替代 PyBullet 物理引擎，'
    '从而使全部用例均可在无仿真器环境下自动执行。\n'
    '用例设计综合使用等价类划分、边界值分析与场景法，共 50 条，'
    '覆盖输入解析、回合重置、运动学正解、末端模型、评分判定、避障姿态策略、'
    '逆解与限幅、异常安全、动作输出以及端到端抓取 10 个测试项。'
)


def load_results() -> tuple[list[dict], dict]:
    with open(RESULTS, encoding='utf-8') as f:
        data = json.load(f)
    return data['cases'], data['summary']


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


def fill_information(ws, summary: dict, total: int, today: str) -> None:
    ws['E7'] = 'v1.0（2024 种子杯参赛提交版本）'
    ws['E8'] = '种子杯 2024 机械臂避障抓取——算法核心模块自动化测试'
    ws['J8'] = 'SEEDCUP-2024-ALG'
    ws['E9'] = '（待填写）'
    ws['J9'] = today
    ws['E10'] = '（待填写）'
    ws['J10'] = today
    ws['E11'] = '（待填写）'
    ws['J11'] = today
    ws['B16'] = OVERVIEW
    ws['B20'] = today
    ws['C20'] = '1.00'
    ws['E20'] = ('initial 初稿完成：依据 URDF/env.py/footnote 构造测试预言，'
                 '完成 50 条用例设计与自动化执行')


def main() -> int:
    cases, summary = load_results()
    total = len(cases)
    today = datetime.date.today().strftime('%Y.%m.%d')

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
    fill_information(wb[SHEET_INFO], summary, total, today)
    wb.save(OUTPUT)

    counts = {k: summary[k] for k in ('OK', 'POK', 'NG', 'NT')}
    tags: dict[str, int] = {}
    for case in cases:
        key = method_tag(case['method'])
        tags[key] = tags.get(key, 0) + 1

    print(f'已生成 {OUTPUT}')
    print(f'用例总数 {total}：OK={counts["OK"]} POK={counts["POK"]} '
          f'NG={counts["NG"]} NT={counts["NT"]}')
    print(f'OK 率 {counts["OK"] / total * 100:.1f}%')
    print('测试方法覆盖：' + '、'.join(f'{k} {v} 条' for k, v in sorted(tags.items())))
    return 0


if __name__ == '__main__':
    sys.exit(main())
