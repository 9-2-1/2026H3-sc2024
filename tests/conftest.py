# -*- coding: utf-8 -*-
"""pytest 插件：把 pytest 的通过/失败映射回《附录1》模板的 OK/POK/NG/NT 判定。

模板（Introduction 使用说明 第 9 条）规定的四种判定：
    OK  测试通过
    POK 部分通过
    NG  不能通过
    NT  测试用例错误，测试无法执行

pytest 只区分"通过/失败"两态，表达不了 POK 与 NT，故映射规则如下：
    test_cases.py 断言通过        -> 采用用例自报的 OK / POK
    断言失败                      -> NG（用例跑通了，但实际结果不符合预期）
    用例自身抛出异常              -> NT（用例已失效/无法执行，如被测接口变更）

会话结束时把逐条判定写成 tests/results.json，供 make_excel.py 回填交付件。
"""
from __future__ import annotations

import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

RESULTS = os.path.join(HERE, 'results.json')

# 会话级结果收集：测试体每执行完一条就 append 一行
_ROWS: list[dict] = []

STATUSES = ('OK', 'POK', 'NG', 'NT')


@pytest.fixture
def record_result():
    """用例把 (case, status, actual) 交给它落账，返回一个记录函数。"""

    def _record(case: dict, status: str, actual: str, seconds: float) -> None:
        assert status in STATUSES, f'未知判定 {status!r}'
        _ROWS.append({
            'id': case['id'],
            'module': case['module'],
            'item': case['item'],
            'title': case['title'],
            'level': case['level'],
            'method': case['method'],
            'pre': case['pre'],
            'input': case['inp'],
            'steps': case['steps'],
            'expected': case['exp'],
            'status': status,
            'actual': actual,
            'seconds': round(seconds, 3),
        })

    return _record


def pytest_sessionfinish(session, exitstatus) -> None:
    """写出 results.json。没跑任何用例（如误跑单个辅助文件）时不覆盖既有结果。"""
    if not _ROWS:
        return
    rows = sorted(_ROWS, key=lambda r: r['id'])
    counts = {k: sum(1 for r in rows if r['status'] == k) for k in STATUSES}
    with open(RESULTS, 'w', encoding='utf-8') as f:
        json.dump({'cases': rows, 'summary': counts, 'total': len(rows)}, f,
                  ensure_ascii=False, indent=2)
