# -*- coding: utf-8 -*-
"""一键执行器：调用 pytest 跑完全部用例，再按交付模块打印汇总。

用法（仓库根目录）：
    python tests/run_cases.py              # 跑全部 50 条
    python tests/run_cases.py -m module1   # 只跑模块一（30 条）
    python tests/run_cases.py -m module2   # 只跑模块二（20 条）
    python tests/run_cases.py -k TC-ALG-042  # 只跑某一条
    python tests/run_cases.py -x           # 任意 pytest 参数原样透传

pytest 本身也能直接执行（配置见仓库根目录 pytest.ini）：
    pytest

逐条判定由 conftest.py 写成 tests/results.json，供 make_excel.py 回填交付件。
"""
from __future__ import annotations

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import pytest  # noqa: E402

RESULTS = os.path.join(HERE, 'results.json')
TEST_FILE = os.path.join(HERE, 'test_cases.py')


def _by_module(rows: list[dict]) -> list[tuple[str, list[dict]]]:
    """按交付模块分组，保持用例原有先后顺序。"""
    order: list[str] = []
    groups: dict[str, list[dict]] = {}
    for row in rows:
        name = row['module']
        if name not in groups:
            groups[name] = []
            order.append(name)
        groups[name].append(row)
    return [(name, groups[name]) for name in order]


def _print_summary(label: str, rows: list[dict]) -> None:
    n = len(rows)
    counts = {k: sum(1 for r in rows if r['status'] == k) for k in ('OK', 'POK', 'NG', 'NT')}
    print('=' * 78)
    print(f'{label} {n} 条   OK={counts["OK"]}  POK={counts["POK"]}  '
          f'NG={counts["NG"]}  NT={counts["NT"]}')
    print(f'通过率(OK) = {counts["OK"]/n*100:.1f}%   '
          f'通过率(OK+POK) = {(counts["OK"]+counts["POK"])/n*100:.1f}%')
    print('=' * 78)


def main() -> int:
    pytest.main([TEST_FILE] + sys.argv[1:])

    if not os.path.exists(RESULTS):
        print('未生成 results.json：本次没有执行任何用例，请检查 -k / -m 筛选条件')
        return 1

    with open(RESULTS, encoding='utf-8') as f:
        data = json.load(f)

    rows = data['cases']
    print()
    groups = _by_module(rows)
    for label, module_rows in groups:
        _print_summary(label, module_rows)
    if len(groups) > 1:
        _print_summary('合计', rows)

    failed = [r for r in rows if r['status'] not in ('OK', 'POK')]
    if failed:
        print(f'未通过 {len(failed)} 条（《附录2：缺陷报告》的候选）：')
        for r in failed:
            print(f'  {r["status"]}  {r["id"]}  {r["title"]}')
        print('注：用例未通过属于被测模块的缺陷，不改变本脚本退出码；'
              'pytest 自身的退出码见上方输出。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
