#!/usr/bin/env python3
"""아무 코스에나 **정지차를 좌우 번갈아** 심는다 -> `scenarios/<코스>_SLA.xml`.

사용자 요청 2026-09-12: "2차선 도로에서 장애물이 2차선 -> 1차선 -> 2차선 이런식으로
배치되어 S자로 움직이게 하는 것". 오프라인 판(`eval/scenarios/slalom_2lane.json`)으로
먼저 만들었고(100점), 이건 그걸 VTD 에서 보기 위한 것이다.

심는 자리: 경로 길이의 `--at` 비율마다 하나씩, **내 차로 -> 옆 차로 -> 내 차로** 순서.
  · 교차로(j) 안이거나 옆 차로가 없는(l < 1.5 차로폭) 자리는 앞으로 밀어 다시 찾는다.
  · 정지차라 속도 0. 트리거는 자차가 다가오면 뜨게 큰 값을 준다(add_oncoming 규약과 같다).

사용: python3 vtd/add_slalom.py <xodr> HL_FMA_NEW_H [--at 0.30,0.42,0.54]
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from add_oncoming import add                                          # noqa: E402
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


def _c(lp, i):
    """그 점에서 경로가 차도 안 어디에 있나((l-r)/2). 모르면 None."""
    p = lp[i] if i < len(lp) else None
    if not p or p.get("l") is None or p.get("r") is None:
        return None
    return (p["l"] - p["r"]) / 2.0


def spot(rt, lp, cum, from_m, want_left):
    """`from_m` 부터 앞으로 훑어 **쓸 만한 자리**를 찾는다. (x, y, heading, 차로폭, s).

    거르는 자리: 교차로 안 · 옆 차로가 없는 곳 · **경로가 차선을 옮기는 중**인 곳
    (앞뒤 30m 에서 (l-r)/2 가 1m 넘게 변하면 그 구간이다. 거기 심으면 S자 시험이 아니라
     차선변경 시험이 된다).
    """
    i0 = min(range(len(cum)), key=lambda k: abs(cum[k] - from_m))
    for i in range(i0, min(len(rt), i0 + 600)):
        p = lp[i] if i < len(lp) else None
        if not p or p.get("j") or p.get("jx"):
            continue                                   # 교차로 안에는 안 심는다
        w = float(p.get("w") or 3.2)
        if want_left and float(p.get("l") or 0.0) < 1.5 * w:
            continue                                   # 옆 차로가 없다
        ja = min(len(rt) - 1, i + 30)
        jb = max(0, i - 30)
        ca, cb, ci = _c(lp, ja), _c(lp, jb), _c(lp, i)
        if None in (ca, cb, ci) or abs(ca - ci) > 1.0 or abs(ci - cb) > 1.0:
            continue                                   # 경로가 옮겨 가는 구간
        h = hdg(rt, i)
        x, y = rt[i][0], rt[i][1]
        if want_left:                                  # 왼쪽 법선 = (-sin, cos)
            x -= w * math.sin(h)
            y += w * math.cos(h)
        return x, y, math.degrees(h) % 360.0, w, cum[i]
    return None


MIN_GAP = 60.0          # 정지차 사이 최소 간격[m] — 붙여 놓으면 S자가 아니라 벽이 된다


def main():
    ap = argparse.ArgumentParser(description="정지차를 좌우 번갈아 심는다")
    ap.add_argument("xodr")
    ap.add_argument("course", help="예: HL_FMA_NEW_H")
    ap.add_argument("--at", default="0.30,0.42,0.54", help="경로 길이 비율(쉼표)")
    ap.add_argument("--start-left", action="store_true", help="옆 차로부터 시작")
    a = ap.parse_args()

    rt = json.load(open(f"{HERE}/../routes/{a.course}.json", encoding="utf-8"))["ego_route"]
    lp = json.load(open(f"{HERE}/../routes/{a.course}_lane.json", encoding="utf-8"))["pts"]
    Map(a.xodr)                                        # 지도는 검증용(자리 계산은 차로계획으로 한다)
    cum = cumdist(rt)
    src = f"{HERE}/../scenarios/{a.course}.xml"
    out = f"{HERE}/../scenarios/{a.course}_SLA.xml"
    if not os.path.exists(src):
        raise SystemExit(f"{src} 가 없다 — 원본 시나리오가 있어야 한다")

    cur, n, made = src, 0, []
    prev_m = -1e9
    for k, f in enumerate(float(v) for v in a.at.split(",")):
        left = (k % 2 == 1) if not a.start_left else (k % 2 == 0)
        want_m = cum[-1] * f if f <= 1.0 else f         # 1 이하면 비율, 크면 미터
        s = spot(rt, lp, cum, max(want_m, prev_m + MIN_GAP), left)
        if s is None:
            print(f"⚠️ 비율 {f} 근처에 쓸 자리가 없다 — 건너뜀", file=sys.stderr)
            continue
        x, y, hd, w, at_m = s
        prev_m = at_m
        n += 1
        dst = f"/tmp/_sla_{n}.xml"
        nm = f"Sla{n}{'L' if left else 'R'}"
        add(cur, dst, x, y, hd, speed=0.0, trig=200.0, name=nm)
        cur = dst
        made.append(f"{nm} {'옆 차로' if left else '내 차로'} s={at_m:.0f}m ({x:.1f},{y:.1f})")

    if not made:
        raise SystemExit("심은 게 없다")
    os.replace(cur, out)
    print(f"만듦: {out}")
    for m in made:
        print("  ", m)
    print(f"\n① 올리기 :  bash vtd/push_scenarios.sh {a.course}_SLA      (이름 꼭 준다 — 안 주면 전부 덮어쓴다)")
    print(f"② VTD    :  OMEN 에서 {a.course}_SLA 시나리오 로드")
    print(f"③ 주행   :  CAM=1 bash drive.sh {a.course}"
          f"      (경로는 원본 {a.course} 그대로 쓴다)")


if __name__ == "__main__":
    main()
