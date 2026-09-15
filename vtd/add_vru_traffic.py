#!/usr/bin/env python3
"""**주변교통 + 취약대상**을 한 판에 넣는다 -> `scenarios/<코스>_TRV.xml`.

사용자 요청 2026-09-09: "대회에서 어떤 돌발 환경이 나올지 모르니 차량 있는 게 맞음.
그래서 사람(걸어다니는 사람, 자전거, 휠체어) 넣은 시나리오 만들었어?" -> 이게 그 판이다.

바탕은 `<코스>_TR.xml`(주변교통 50대)이고, 그 위에 셋을 얹는다.

  · **걷는 사람** — 경로가 지나는 횡단보도에서 자차가 다가오면 건넌다(`add_crosswalk_peds`)
  · **달리는 자전거** — 내 차로를 4.4m/s(16km/h)로 앞서 간다
  · **정지한 휠체어** — 내 차로 가장자리에 서 있다

VTD 문법(2026-09-09 실측, 샘플 `BicycleDemo.xml` · `TrafficDemo.xml`):
  · 사람과 자전거는 **Character** 다. 동작은 `Motion Move=` 로 고르는데, 샘플에 실제로
    쓰인 값은 walk · jog · run · wait · **bicycle_ride** · listen · talk 뿐이다.
    `bicycle_ride` 가 곧 **자전거를 탄 사람**이다(모델 자체가 자전거+사람).
  · **휠체어는 동작이 없다.** 카탈로그의 `wheelchair_adult` 는 오브젝트라 제자리에 있다
    (`add_vru.py` 와 같은 방식으로 `MovingObjectsControl` 에 넣는다).
  · 자차 접근 트리거는 `PosRelative Pivot="Ego" Distance=...`.

무엇을 보려는 판인가:
  ① 횡단보도 위 보행자 앞에서 서는가(항목⑩·⑫)
  ② **달리는 자전거를 안전하게 지나가는가** — 이게 핵심이다. 오프라인 `vru_bicycle_slow`
     에서 이미 **미완주(85점)** 로 재현됐다: `overtake.py` 가 움직이는 것을 전부 추월
     대상에서 빼기 때문에 자전거 뒤에 붙어 끝까지 따라간다. 주변교통까지 있는 실제
     코스에서 그게 얼마나 비싼지(5분 안에 완주하나, 뒤에 정체가 쌓이나)를 본다.
  ③ 정지한 휠체어를 옆으로 비켜 가는가(H 0.92 라 '작은 물건'으로 오인하면 밟는다)

    python3 vtd/add_vru_traffic.py <xodr> HL_FMA_NEW_A
    python3 vtd/add_vru_traffic.py <xodr> HL_FMA_NEW_A --bikes 1 --peds 3

올릴 때는 **이름을 준다**: `bash vtd/push_scenarios.sh HL_FMA_NEW_A_TRV`
돌릴 때는 `bash drive.sh HL_FMA_NEW_A_TRV` (경로는 접미사를 떼고 원래 코스 것을 쓴다).

⚠️ VTD 에서 자전거 모델이 뜨는지·달리는지는 **실측 전이다.** 안 보이면 `--appearance`
   로 외형을 바꾸고, 서 있기만 하면 `--bike-trigger` 를 넓힌다.
"""
import argparse
import json
import math
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from add_crosswalk_peds import (APPEAR, ON_ROUTE, MIN_SEP, SKIP_HEAD, SKIP_TAIL, Z_LIFT,
                                character_block, crossing_path, crosswalks_on_route,
                                cumdist, insert, pick)                   # noqa: E402
from add_hazards import elevation                            # noqa: E402
from build_lane_plan import Map, road_point                              # noqa: E402
from plan_route import locate_lanes_all                                  # noqa: E402

HERE = __file__.rsplit("/", 1)[0]
BIKE_V = 4.4          # 자전거 속도[m/s] = 16km/h (오프라인 vru_bicycle_slow 와 같은 값)
BIKE_LEN = 150.0      # 자전거가 앞서 갈 거리[m] — 이만큼은 우리 앞을 막는다
BIKE_STEP = 10.0      # 자전거 경로 웨이포인트 간격[m]
BIKE_EXIT_M = 14.0    # 길 끝에서 **차도 밖으로 빠지는** 거리[m] — 그 자리에 서면 우리 차로를 막는다
BIKE_EXIT_PAD = 2.5   # 차도 오른쪽 끝에서 이만큼 더 밖까지[m]
NEED_ROOM = 3.2       # 놓을 곳에 필요한 옆 여유[m](`add_vru.NEED_ROOM` 과 같은 기준)
WC_EDGE = 1.0         # 휠체어를 차로 중심에서 이만큼 오른쪽에 놓는다[m]


def spot(rt, lp, cum, frac, used, min_sep=MIN_SEP, need_room=NEED_ROOM):
    """교차로 밖 + 옆 여유가 있는 자리(경로 인덱스). `add_vru.pick_spot` 과 같은 기준."""
    start = min(range(len(rt)), key=lambda i: abs(cum[i] - cum[-1] * frac))
    for i in range(start, len(rt) - 30):
        p = lp[i] if i < len(lp) else None
        if not p or p.get("j") or p.get("jx"):
            continue
        if max(p.get("l", 0.0) or 0.0, p.get("r", 0.0) or 0.0) < need_room:
            continue
        if cum[i] < SKIP_HEAD or cum[i] > cum[-1] - SKIP_TAIL:
            continue
        if any(abs(cum[i] - cum[j]) < min_sep for j in used):
            continue
        return i
    return None


def road_z(mp, x, y, lift):
    c = locate_lanes_all(mp, x, y)
    return (elevation(mp.roads[c[0][0]], c[0][2]) + lift) if c else 0.0


def bike_path(rt, cum, i, lp=None):
    """자전거가 갈 길 = 그 지점부터 앞으로 BIKE_LEN, BIKE_STEP 간격의 경로점.

    ★★**마지막에 차도 밖으로 뺀다**(2026-09-11). 안 그러면 길 끝에 도달한 자전거가
      **우리 차로 한복판에 그대로 선다**. 실측 코스 D TRV: 경로기준 횡 0.00m 에
      2.0x0.6x1.7 물체가 속도 0 으로 서 있어, 교차로 한복판에서 한 차로(3.45m) 옆으로
      비켜 가는 기동이 걸렸다 — 조향 35° 로 13초 기어갔다(사용자 "왜 지랄남").
      코스 A TRV 도 같은 꼴로 42초 막혔다. 차도 오른쪽 끝(`r`) 밖으로 내보낸다.
    """
    pts, last, j = [rt[i]], cum[i], i
    for k in range(i + 1, len(rt)):
        if cum[k] - last >= BIKE_STEP:
            pts.append(rt[k]); last = cum[k]; j = k
        if cum[k] - cum[i] >= BIKE_LEN:
            j = k
            break
    if len(pts) < 2:
        return None
    q = (lp[j] if (lp and j < len(lp)) else None) or {}
    h = heading_at(rt, j)
    fx, fy = math.cos(h), math.sin(h)
    rx, ry = math.sin(h), -math.cos(h)              # 경로 오른쪽
    x0, y0 = rt[j][0], rt[j][1]
    base = float(q.get("r") or 0.0)
    end = None
    # ★경로가 되돌아오는 코스(H)는 **나중에 다시 지나갈 자리**에 세워도 막힌다.
    #   끝점이 경로 어디의 차도 안이면 더 밖으로 민다.
    for extra in (BIKE_EXIT_PAD, BIKE_EXIT_PAD + 3.0, BIKE_EXIT_PAD + 7.0, BIKE_EXIT_PAD + 12.0):
        room = base + extra
        cand = (x0 + fx * BIKE_EXIT_M + rx * room, y0 + fy * BIKE_EXIT_M + ry * room)
        end = (cand, room)
        if off_road(cand, rt, lp):
            break
    (ex, ey), room = end
    pts.append((x0 + fx * BIKE_EXIT_M * 0.5 + rx * room * 0.5,
                y0 + fy * BIKE_EXIT_M * 0.5 + ry * room * 0.5))
    pts.append((ex, ey))
    return pts


def off_road(pt, rt, lp, pad=1.5):
    """그 점이 경로 **어느 지점의** 차도 밖인가(가장 가까운 경로점 기준)."""
    if not lp:
        return True
    k = min(range(len(rt)), key=lambda n: (rt[n][0] - pt[0]) ** 2 + (rt[n][1] - pt[1]) ** 2)
    q = (lp[k] if k < len(lp) else None) or {}
    edge = max(float(q.get("r") or 0.0), float(q.get("l") or 0.0))
    return math.dist(rt[k], pt) > edge + pad


def heading_at(rt, i):
    a, b = rt[max(i - 3, 0)], rt[min(i + 3, len(rt) - 1)]
    return math.atan2(b[1] - a[1], b[0] - a[0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("xodr")
    ap.add_argument("course", help="예: HL_FMA_NEW_A")
    ap.add_argument("--peds", type=int, default=4, help="횡단보도를 건너는 사람 수")
    ap.add_argument("--bikes", type=int, default=2, help="내 차로를 달리는 자전거 수")
    ap.add_argument("--wheelchairs", type=int, default=1, help="차로 가장자리에 선 휠체어 수")
    ap.add_argument("--trigger", type=float, default=45.0, help="보행자 출발 거리[m]")
    ap.add_argument("--bike-trigger", type=float, default=80.0, help="자전거 출발 거리[m]")
    ap.add_argument("--speed", type=float, default=1.3, help="보행 속도[m/s]")
    ap.add_argument("--bike-speed", type=float, default=BIKE_V, help="자전거 속도[m/s]")
    ap.add_argument("--appearance", default=None, help="외형 하나로 통일(모델이 안 뜰 때)")
    ap.add_argument("--src", default=None, help="기본 scenarios/<course>_TR.xml")
    ap.add_argument("--out", default=None, help="기본 scenarios/<course>_TRV.xml")
    a = ap.parse_args()

    rt = json.load(open(f"{HERE}/../routes/{a.course}.json", encoding="utf-8"))["ego_route"]
    lp = json.load(open(f"{HERE}/../routes/{a.course}_lane.json", encoding="utf-8"))["pts"]
    cws = json.load(open(f"{HERE}/../routes/crosswalks.json", encoding="utf-8"))["crosswalks"]
    mp = Map(a.xodr)
    cum = cumdist(rt)
    blocks, made, used = [], [], []

    # ── 걷는 사람(횡단보도) ────────────────────────────────────────────────
    chosen = pick(crosswalks_on_route(rt, cum, cws), cum[-1], a.peds)
    for k, (s_cw, cw) in enumerate(chosen, 1):
        got = crossing_path(mp, cw, start_right=(k % 2 == 1))
        if got is None:
            print(f"⚠️ road {cw['road']} s={cw['s']}: 건널 길을 못 잡아 생략", file=sys.stderr)
            continue
        p0, p1, _yaw, z, width = got
        ctype, appear = APPEAR[(k - 1) % len(APPEAR)]
        blocks.append(character_block(k, f"CwPed{k}", ctype, a.appearance or appear,
                                      [p0, p1], z, a.trigger, a.speed, move="walk"))
        used.append(min(range(len(rt)), key=lambda i: abs(cum[i] - s_cw)))
        made.append(f"CwPed{k} 걷는사람 s={s_cw:.0f}m 폭 {width:.1f}m "
                    f"{'신호' if cw.get('signal') else '무신호'}")

    # ── 달리는 자전거(내 차로) ────────────────────────────────────────────
    for k in range(1, a.bikes + 1):
        i = spot(rt, lp, cum, 0.30 + 0.35 * (k - 1), used)
        if i is None:
            print(f"⚠️ Bike{k}: 자리를 못 찾아 생략", file=sys.stderr)
            continue
        pts = bike_path(rt, cum, i, lp)
        if pts is None:
            continue
        used.append(i)
        ctype, appear = APPEAR[(k - 1) % len(APPEAR)]
        z = road_z(mp, rt[i][0], rt[i][1], Z_LIFT)
        blocks.append(character_block(10 + k, f"Bike{k}", ctype, a.appearance or appear,
                                      pts, z, a.bike_trigger, a.bike_speed, move="bicycle_ride"))
        _far = math.dist(pts[0], pts[-1])
        made.append(f"Bike{k} 자전거 {a.bike_speed:.1f}m/s s={cum[i]:.0f}m "
                    f"웨이포인트 {len(pts)}개 · 직선거리 {_far:.0f}m · 끝에서 갓길로 이탈")

    # ── 정지한 휠체어(차로 가장자리) ──────────────────────────────────────
    for k in range(1, a.wheelchairs + 1):
        i = spot(rt, lp, cum, 0.55 + 0.2 * (k - 1), used)
        if i is None:
            print(f"⚠️ Wheelchair{k}: 자리를 못 찾아 생략", file=sys.stderr)
            continue
        used.append(i)
        h = heading_at(rt, i)
        x = rt[i][0] + WC_EDGE * math.sin(h)          # 경로 오른쪽(-t 방향)
        y = rt[i][1] - WC_EDGE * math.cos(h)
        z = road_z(mp, x, y, Z_LIFT)
        # ★★휠체어는 **물체가 아니라 사람의 동작**이다(2026-09-11 OMEN 카탈로그 실측).
        #   `PedCal3DCfg.xml` 의 male_adult 에 `wheelchair_idle`(0m/s) · `wheelchair_ride`
        #   (1.35m/s) 동작이 있고, 둘 다 `wheelchair_adult` 골격을 붙이는 compound 다 —
        #   자전거(`bicycle_ride`)와 같은 방식. 예전엔 `<Object Definition="wheelchair_adult">`
        #   로 넣어 VTD 가 그 이름의 물체를 못 찾고 **아예 안 띄웠다**(사용자: "휠체어가 안 나와").
        #   제자리에 서 있게 `wheelchair_idle` · 속도 0 이고, 길은 형식상 1m 짜리다.
        ctype, appear = ("male_adult", "Christian")
        pts = [(x, y), (x + math.cos(h) * 1.0, y + math.sin(h) * 1.0)]
        blocks.append(character_block(20 + k, f"Wheelchair{k}", ctype, a.appearance or appear,
                                      pts, z, a.trigger, 0.0, move="wheelchair_idle"))
        made.append(f"Wheelchair{k} 정지 휠체어(사람+wheelchair_idle) s={cum[i]:.0f}m "
                    f"(차로 중심 우 {WC_EDGE:.1f}m)")

    if not blocks:
        raise SystemExit(f"{a.course}: 아무것도 못 심었다")
    src = a.src or f"{HERE}/../scenarios/{a.course}_TR.xml"
    out = a.out or f"{HERE}/../scenarios/{a.course}_TRV.xml"
    open(out, "w", encoding="utf-8").write(insert(open(src, encoding="utf-8").read(),
                                                  "".join(blocks)))
    print(f"-> {out}\n   " + "\n   ".join(made))


if __name__ == "__main__":
    main()
