#!/usr/bin/env python3
"""경로 위 **지정한 거리·차로**에 사물을 심는다 -> 시나리오 XML.

사용자 요청 2026-09-12: "장애물을 3개를 한 줄에 넣어놓으면 어카냐. 장애물이 있으면
**차선 변경 하게** 하는 게 목표지".

`add_slalom.py` 는 같은 일을 **정지차**로 하고 자리도 비율로 알아서 잡는다. 여기는
자리를 **미터로 못 박고**(회피 뒤 되돌아올 거리를 내가 재야 한다) **사물**을 심는다.
차로 오프셋은 차로폭 배수다 — `0` 이면 내 차로, `1` 이면 왼쪽 한 차로, `-1` 이면 오른쪽.

    python3 vtd/add_objects_at.py <xodr> LC4_OBS_LEFT --at 25:0 --at 62:1 --at 99:0

⚠️ 같은 이름표(`--name Can`)로 이미 심어 둔 것은 **지우고 다시 넣는다**. 그래야 여러 번
   돌려도 같은 결과가 나온다(안 그러면 옛 자리에 하나 더 얹힌다).
⚠️ 회피는 `drive.py:_nudge_offset` 이 한다. 옆 차로가 통째로 있으면(`room >= 차로폭 +
   차폭절반`) **차로 하나를 온전히 옮긴다** — 그래서 사물 하나로도 진짜 차선변경이 난다.
   같은 차로에 일렬로 세 개를 놓으면 한 번 비킨 뒤 그대로 지나가므로 시험이 안 된다.
"""
import argparse
import json
import math
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from add_vru import add_object                                        # noqa: E402
from build_lane_plan import Map                                       # noqa: E402


def cumdist(rt):
    c = [0.0]
    for a, b in zip(rt[:-1], rt[1:]):
        c.append(c[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return c


def hdg(rt, i):
    a = rt[max(0, i - 3)]
    b = rt[min(len(rt) - 1, i + 3)]
    return math.atan2(b[1] - a[1], b[0] - a[0])


def strip(src, dst, prefix):
    """`prefix` 로 시작하는 이름의 오브젝트를 XML 에서 걷어낸다."""
    s = open(src, encoding="utf-8").read()
    s, n = re.subn(rf'<Object Type="other" Name="{prefix}\d*".*?</Object>'
                   rf'<ObjectActions Object="{prefix}\d*"\s*/>', "", s, flags=re.S)
    open(dst, "w", encoding="utf-8").write(s)
    return n


def main():
    ap = argparse.ArgumentParser(description="경로 위 지정한 자리에 사물을 심는다")
    ap.add_argument("xodr")
    ap.add_argument("course", help="예: LC4_OBS_LEFT")
    ap.add_argument("--at", action="append", required=True,
                    help="<거리m>:<차로오프셋> (왼쪽 +). 여러 번 준다")
    ap.add_argument("--definition", default="Fuelcan01", help="VTD 모델 이름")
    ap.add_argument("--name", default="Can", help="이름표 앞자리")
    ap.add_argument("--z-lift", type=float, help="노면 위로 올릴 높이[m]. 기본은 기름통 기준 0.74 — "
                    "WheelStack01 처럼 원점이 바닥인 모델은 0")
    ap.add_argument("--src", help="원본 XML (기본: scenarios/<course>.xml)")
    ap.add_argument("--out", help="결과 XML (기본: 원본과 같은 파일 = 제자리 갱신)")
    a = ap.parse_args()

    rt = json.load(open(f"{HERE}/../routes/{a.course}.json", encoding="utf-8"))["ego_route"]
    lp = json.load(open(f"{HERE}/../routes/{a.course}_lane.json", encoding="utf-8"))["pts"]
    mp = Map(a.xodr)
    cum = cumdist(rt)
    src = a.src or f"{HERE}/../scenarios/{a.course}.xml"
    out = a.out or src

    tmpd = os.environ.get("TMPDIR", "/tmp")
    stage = f"{tmpd}/_objat_0.xml"
    gone = strip(src, stage, a.name)
    if gone:
        print(f"이미 있던 {a.name}* {gone}개를 지웠다")

    made = []
    for k, spec in enumerate(a.at, 1):
        m_s, _, m_l = spec.partition(":")
        at_m, lane_off = float(m_s), float(m_l or 0.0)
        i = min(range(len(cum)), key=lambda j: abs(cum[j] - at_m))
        p = lp[i] if i < len(lp) else {}
        w = float(p.get("w") or 3.2)
        room = float(p.get("l" if lane_off > 0 else "r") or 0.0)
        if abs(lane_off) * w > room + 1e-6:
            raise SystemExit(f"{spec}: 그쪽 여유가 {room:.1f}m 뿐이다(필요 {abs(lane_off)*w:.1f}m)")
        h = hdg(rt, i)
        x = rt[i][0] - lane_off * w * math.sin(h)
        y = rt[i][1] + lane_off * w * math.cos(h)
        dst = f"{tmpd}/_objat_{k}.xml"
        nm = f"{a.name}{k}"
        z = (add_object(stage, dst, mp, x, y, h, a.definition, nm) if a.z_lift is None
             else add_object(stage, dst, mp, x, y, h, a.definition, nm, z_lift=a.z_lift))
        stage = dst
        made.append(f"{nm} s={cum[i]:.0f}m 차로오프셋{lane_off:+.0f}({lane_off*w:+.1f}m) "
                    f"({x:.1f},{y:.1f}) Z={z:.1f}")

    shutil.copy(stage, out)
    print(f"만듦: {out}")
    for t in made:
        print("  ", t)


if __name__ == "__main__":
    main()
