#!/usr/bin/env python3
"""**이 좌표에서 왜 그렇게 굴었나**를 한 번에 보여준다.

    python3 tools/whyhere.py <코스> <x> <y> [반경m] [run.csv]
    python3 tools/whyhere.py A 1266 141
    python3 tools/whyhere.py HL_FMA_NEW_E 1465 898 25 runs_tr/run_E_TR.csv

왜 필요한가(2026-08-26): 사용자가 화면을 보다가 "여기서 왜 깜빡이를 켜"라고 물으면,
매번 손으로 차로계획을 열고 `_upcoming_turn` 을 재현하고 로그를 뒤졌다. 오늘 하루에만
세 번 했고 **두 번은 각도를 눈대중으로 역산하다 틀렸다.** 지도와 로그가 아는 것을
그대로 찍는다 — 추측할 자리를 없앤다.

찍는 것
  · 그 지점 차로계획: 차로번호 · 차로폭 · 좌우 여유 · 교차로 여부 · 제한속도 · `sig`
  · `_upcoming_turn`(교차로 회전 지시등)이 그 자리에서 켜지는지와 그 근거(방위변화)
  · 가까운 신호등 정지선 · 도색 정지선 · 횡단보도 거리
  · 주행 CSV 가 있으면 그 좌표를 지나간 구간의 프레임(속도·판단·지시등·오프셋·주변 물체)
"""
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

from scenario import Scenario                                    # noqa: E402
from drive import DrivingStack                                   # noqa: E402

R = os.path.join(ROOT, "routes")


def route_base(name):
    """`A` 같은 짧은 이름도 받는다. drive.sh 의 짝 규칙과 같게."""
    if os.path.exists(os.path.join(R, name + ".json")):
        return name
    for cand in (f"HL_FMA_NEW_{name}", f"HL_FMA_{name}"):
        if os.path.exists(os.path.join(R, cand + ".json")):
            return cand
    if name in ("v1", "v2", "v3", "v4", "v5", "v6"):
        return "planned_from_xml_v1"
    if name == "v7":
        return "planned_v7"
    if name in ("WP9", "wp9"):
        return "planned_wp9"
    raise SystemExit(f"❌ 경로 파일을 못 찾겠다: {name}")


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        raise SystemExit(2)
    base = route_base(sys.argv[1])
    x, y = float(sys.argv[2]), float(sys.argv[3])
    rad = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0
    csv_path = sys.argv[5] if len(sys.argv) > 5 else None
    if csv_path is None:                       # 스윕이 남긴 CSV 가 있으면 자동으로 쓴다
        for c in (f"runs_tr/run_{sys.argv[1]}_TR.csv", f"runs_local/run_{base}.csv"):
            if os.path.exists(os.path.join(ROOT, c)):
                csv_path = os.path.join(ROOT, c)
                break

    lane = json.load(open(f"{R}/{base}_lane.json", encoding="utf-8"))["pts"]
    st = DrivingStack(scenario=Scenario.load(f"{R}/{base}.json"), base_limit=8.33,
                      lane_plan=lane)
    route, cum = st.route, st.cum
    bi = min(range(len(route)), key=lambda k: (route[k][0] - x) ** 2 + (route[k][1] - y) ** 2)
    off = math.hypot(route[bi][0] - x, route[bi][1] - y)
    print(f"■ 경로 {base} · 가장 가까운 경로점 #{bi} s={cum[bi]:.1f}m "
          f"({route[bi][0]:.1f},{route[bi][1]:.1f}) — 준 좌표에서 {off:.1f}m")
    if off > 30:
        print("  ⚠️ 30m 넘게 떨어졌다. 이 코스의 경로가 아닐 수 있다.")

    p = lane[bi] or {}
    print(f"\n■ 차로계획  차로 {p.get('lane')} · 폭 {p.get('w')}m · "
          f"좌여유 {p.get('l')} · 우여유 {p.get('r')} · 반대차선 xl {p.get('xl')} xr {p.get('xr')}")
    print(f"            교차로 j={p.get('j')} · 제한 {p.get('lim')}m/s"
          f"({(p.get('lim') or 0) * 3.6:.0f}km/h) · 경로차선변경 sig={p.get('sig')}")
    if p.get("sig"):
        j = bi
        while j < len(route) - 1 and cum[j] - cum[bi] < 22.0:
            j += 1
        q = lane[j] or {}
        wp = (p.get("l") or 0) + (p.get("r") or 0)
        wq = (q.get("l") or 0) + (q.get("r") or 0)
        print(f"            └ 22m 뒤: 차로 {q.get('lane')} 좌 {q.get('l')} 우 {q.get('r')} "
              f"· 도로폭 {wp:.2f} -> {wq:.2f} ({'폭 유지 = 진짜 진로변경' if abs(wq - wp) < 1.5 else '폭 변함 = 차로 증감'})")

    t = st._upcoming_turn(bi)

    def hd(k):
        a = route[max(0, k - 3)]
        b = route[min(len(route) - 1, k + 3)]
        return math.atan2(b[1] - a[1], b[0] - a[0])
    j = bi
    while j < len(route) - 1 and cum[j] - cum[bi] < st.TURN_SIGNAL_AHEAD:
        j += 1
    dh = math.degrees((hd(j) - hd(bi) + math.pi) % (2 * math.pi) - math.pi)
    jn = sum(1 for k in range(bi, min(len(lane), j + 1)) if lane[k] and lane[k].get("j"))
    print(f"\n■ 교차로 회전 지시등  _upcoming_turn={t} "
          f"({'좌' if t > 0 else '우' if t < 0 else '꺼짐'})")
    print(f"            앞 {st.TURN_SIGNAL_AHEAD:.0f}m 방위변화 {dh:+.1f}° "
          f"(문턱 {math.degrees(st.TURN_SIGNAL_MIN):.0f}°) · 그 구간 교차로 칸 {jn}칸")

    def nearest(items, get):
        best = None
        for it in items:
            px, py = get(it)
            d = math.hypot(px - x, py - y)
            if best is None or d < best[0]:
                best = (d, it)
        return best
    tl = json.load(open(f"{R}/tl_map_livinglab.json", encoding="utf-8"))
    sl = json.load(open(f"{R}/stoplines.json", encoding="utf-8"))["stoplines"]
    cw = json.load(open(f"{R}/crosswalks.json", encoding="utf-8"))["crosswalks"]
    a = nearest(tl.items(), lambda kv: (kv[1][0], kv[1][1]))
    b = nearest(sl, lambda q: (q[0], q[1]))
    c = nearest(cw, lambda q: (q["x"], q["y"]))
    print(f"\n■ 주변  신호정지선 {a[0]:.1f}m(tl {a[1][0]}) · 도색정지선 {b[0]:.1f}m · "
          f"횡단보도 {c[0]:.1f}m({'신호있음' if c[1].get('signal') else '무신호'}, "
          f"제한 {c[1].get('lim')}m/s)")

    if not csv_path or not os.path.exists(csv_path):
        print("\n(주행 CSV 가 없어 실제 판단은 못 보여준다 — 5번째 인자로 주면 된다)")
        return
    print(f"\n■ 실제 주행 {os.path.relpath(csv_path, ROOT)} — 이 좌표 {rad:.0f}m 안")
    rows = [r for r in csv.DictReader(open(csv_path, encoding="utf-8"))
            if math.hypot(float(r["x"]) - x, float(r["y"]) - y) <= rad]
    if not rows:
        print("   지나간 프레임이 없다.")
        return
    print(f"   {len(rows)}프레임 · t {float(rows[0]['t']):.1f}~{float(rows[-1]['t']):.1f}s")
    last = -99.0
    for r in rows:
        tt = float(r["t"])
        if tt - last < 0.6:
            continue
        last = tt
        sig = {"0": "-", "1": "◀좌", "2": "우▶"}.get(r["sig"], r["sig"])
        print(f"   {tt:7.2f} v={float(r['v']) * 3.6:5.1f}km/h {sig:>3} "
              f"off={float(r['off']):+5.2f} nudge={float(r['nudge']):+5.2f} "
              f"d_ego={float(r['d_ego']):+5.2f} {r['reason'][:18]:18} cap={r['cap_by'][:13]:13} "
              f"clr={r['clr']:>6} tl={r['tl_id']}/{r['tl_state']}")
    # 마지막 프레임의 주변 물체(경로축 폭 rw · 상대방위 dh 까지)
    o = rows[-1].get("objs") or ""
    if o:
        print(f"   └ t={float(rows[-1]['t']):.2f} 의 주변 물체 (fx fy 길이 폭 높이 속도 실여유 경로횡 rw dh)")
        for tok in o.split("|")[:12]:
            f = tok.split(":")
            if len(f) >= 8:
                extra = f"  rw={f[8]} dh={f[9]}°" if len(f) >= 10 else "  (rw·dh 없는 옛 로그)"
                print(f"       fx{float(f[0]):+7.1f} fy{float(f[1]):+6.1f} "
                      f"{f[2]}x{f[3]}x{f[4]} v={f[5]} 여유={f[6]} 경로횡={f[7]}{extra}")


if __name__ == "__main__":
    main()
