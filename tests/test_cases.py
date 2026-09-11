# -*- coding: utf-8 -*-
"""pytest 用例入口：把 cases_spec.CASES 参数化成 54 个独立测试项。

一条用例 = 一个测试项，可按编号 / 标题 / 交付模块筛选：
    pytest                                  # 跑完全部 54 条
    pytest -k TC-ALG-042                    # 只跑某一条
    pytest -k 端到端                         # 按标题关键字筛选
    pytest -m module1                       # 只跑模块一（34 条）
    pytest -m module2                       # 只跑模块二（20 条）

判定映射见 conftest.py：断言通过 = OK/POK，断言失败 = NG，用例抛异常 = NT。
"""
from __future__ import annotations

import time

import pytest

from cases_spec import CASES, MODULE1_NAME

# 交付模块 -> pytest marker，与 pytest.ini 的 markers 声明对应
MODULE_MARKS = {
    MODULE1_NAME: pytest.mark.module1,
    '模块二': pytest.mark.module2,
}

PARAMS = [
    pytest.param(case, id=case['id'], marks=MODULE_MARKS[case['module']])
    for case in CASES
]


@pytest.mark.parametrize('case', PARAMS)
def test_case(case, record_result):
    """执行单条用例：跑通看实际结果是否符合预期；跑不通记为 NT。"""
    started = time.time()
    try:
        status, actual = case['check']()
    except Exception as exc:                     # 用例自身失效 -> NT，不阻断其余用例
        record_result(case, 'NT', f'{type(exc).__name__}: {exc}', time.time() - started)
        pytest.fail(f'[{case["id"]}] 用例无法执行：{type(exc).__name__}: {exc}')

    record_result(case, status, actual, time.time() - started)
    assert status in ('OK', 'POK'), (
        f'[{case["id"]}] {case["title"]}\n'
        f'  预期：{case["exp"]}\n'
        f'  实际：{actual}'
    )
