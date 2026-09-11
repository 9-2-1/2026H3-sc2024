# -*- coding: utf-8 -*-
"""用例执行器：顺序执行 cases_spec.CASES，输出控制台报告与 results.json。"""
from __future__ import annotations

import json
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from cases_spec import CASES  # noqa: E402


def main() -> int:
    rows = []
    t0 = time.time()
    for case in CASES:
        start = time.time()
        try:
            status, actual = case['check']()
            note = ''
        except Exception as exc:                       # 用例自身无法执行 -> NT
            status = 'NT'
            actual = f'{type(exc).__name__}: {exc}'
            note = traceback.format_exc(limit=2)
        rows.append({
            'id': case['id'],
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
            'seconds': round(time.time() - start, 3),
        })
        mark = {'OK': '  OK ', 'POK': ' POK ', 'NG': '  NG ', 'NT': '  NT '}[status]
        print(f'[{mark}] {case["id"]}  {case["title"]}')
        print(f'          实际结果: {actual}')

    total = len(rows)
    counts = {k: sum(1 for r in rows if r['status'] == k) for k in ('OK', 'POK', 'NG', 'NT')}
    print()
    print('=' * 78)
    print(f'合计 {total} 条   OK={counts["OK"]}  POK={counts["POK"]}  '
          f'NG={counts["NG"]}  NT={counts["NT"]}   耗时 {time.time()-t0:.1f}s')
    print(f'通过率(OK) = {counts["OK"]/total*100:.1f}%   '
          f'通过率(OK+POK) = {(counts["OK"]+counts["POK"])/total*100:.1f}%')
    print('=' * 78)

    out = os.path.join(HERE, 'results.json')
    with open(out, 'w', encoding='utf-8') as f:
        json.dump({'cases': rows, 'summary': counts, 'total': total}, f,
                  ensure_ascii=False, indent=2)
    print(f'结果已写入 {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
