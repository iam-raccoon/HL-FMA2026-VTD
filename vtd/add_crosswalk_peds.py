#!/usr/bin/env python3
"""주변교통(TR) 판에 **횡단보도를 건너는 보행자**를 심는다 -> `scenarios/<코스>_TRP.xml`.

사용자 요청 2026-09-09: "횡단보도에 사람 돌아다니는 것도 추가해줘. TR 에다가 횡단보도
사람도 추가해서 하나 새로 만들어줘."

지금까지 보행자는 **서 있기만** 했다(`add_hazards.add_ped` / `add_vru.add_object` —
MovingObjectsControl 의 `Object`). 걷는 사람은 VTD 의 **Character** 다. 문법은 VTD 자체
샘플(`Data/Projects/Default/Scenarios/TrafficDemo.xml`, 2026-09-09 실측)에서 그대로 가져왔다:

    <MovingObjectsControl>
      <PathShape ShapeId="1" ShapeType="polyline" Closed="false" Name="CwPath1">
        <Waypoint X Y Z Yaw .../> ...                 # 건널 길(인도 -> 인도)
      </PathShape>
      <Character CharacterType="male_adult" Class="pedestrian" Appearance="Christian" Name="CwPed1">
        <StartPosAbs X Y Z Direction/>               # 길 시작점(인도 가장자리)에서 대기
      </Character>
      <CharacterActions Character="CwPed1">
        <Action Name="">
          <PosRelative ... Distance="45" Pivot="Ego"/>   # 자차가 45m 안에 오면
          <Motion Move="walk" Speed="1.3" .../>          # 걷기 시작
          <CharacterPath PathShape="1" .../>             # 그 길을 따라 건넌다
        </Action>
      </CharacterActions>
    </MovingObjectsControl>

  ⚠️ 친구가 시도한 `<Player><Description Type="Hannah">` 는 **차량용** 문법이라 안 떴다
     (`add_hazards.py` 주석). "Hannah" 는 CharacterType 이 아니라 **Appearance** 이름이다
     (`Distros/Current/Config/Players/Pedestrians/PedCal3DCfg.xml`: 골격 female_adult 의
     외형 Hannah/Hannah1~3, male_adult 의 Christian/Christian1~2, 아이 Julia·Martin).

어디에 심나: 경로가 실제로 지나는 횡단보도(`routes/crosswalks.json`, 경로점에서 4m 안)
가운데 서로 `MIN_SEP` 이상 떨어진 곳을 최대 `--count` 곳 고른다. 신호 유무를 가리지 않는다
— 신호 횡단보도에서 우리 녹색에 건너는 사람은 **무단횡단자**인데, 그래도 서야 한다
[도교법 제27조①: 횡단보도를 통행하는 보행자 앞 일시정지]. 그게 이 판이 보려는 것이다.

    python3 vtd/add_crosswalk_peds.py <xodr> HL_FMA_NEW_A            -> scenarios/HL_FMA_NEW_A_TRP.xml
    python3 vtd/add_crosswalk_peds.py <xodr> HL_FMA_NEW_A --count 3 --trigger 40

방향: 첫 사람은 **우측(가까운) 인도**에서 출발해 왼쪽으로, 다음은 반대로 — 번갈아 둔다.
트리거 45m: 50km/h(13.9m/s)로 오면 3.2초 뒤 도착. 제동 19m(5m/s²) 가 들어가고도 남는다.
보행 1.3m/s: 7~14m 도로를 5~11초에 건넌다. 즉 자차는 **사람이 길 위에 있는 동안** 도착한다.

⚠️ **VTD 에서 실제로 뜨고 걷는지는 실측 전이다.** Character 문법은 샘플 그대로지만
   이 맵·이 배포판에서 모델이 로드되는지, `ClampToGround` 가 이 노면 고도에서 맞는지는
   화면으로 확인해야 한다. 안 보이면 `--appearance Hannah1` 처럼 바꿔 다시 만들어 본다.
⚠️ 올릴 때는 **반드시 이름을 준다**: `bash vtd/push_scenarios.sh HL_FMA_NEW_A_TRP`
   (인자 없이 부르면 전부 올린다).
"""
import argparse
import json
import math
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from add_hazards import elevation                                   # noqa: E402
from build_lane_plan import Map, road_point, CELL, DRIVABLE          # noqa: E402

HERE = __file__.rsplit("/", 1)[0]
ON_ROUTE = 4.0        # 횡단보도 중심이 경로점에서 이 안이면 '경로가 지나는 횡단보도'[m]
MIN_SEP = 150.0       # 심는 곳끼리 최소 간격[m] — 한 사람을 피한 반응이 다음 사람에 섞이지 않게
SKIP_HEAD = 80.0      # 출발 직후는 뺀다(출발 준비·가속 구간)[m]
SKIP_TAIL = 60.0      # 종점 붙기 구간도 뺀다[m]
EDGE_OUT = 0.8        # 인도 바깥쪽으로 이만큼 더 나가서 서고, 건너서도 그만큼 더 간다[m]
Z_LIFT = 0.12         # 샘플이 쓰는 값. 길은 ClampToGround 라 노면에 붙는다
GAP_BRIDGE = 1.0      # 차도 사이 이보다 좁은 틈(중앙분리대·경계선)은 한 차도로 본다[m]
APPEAR = (("male_adult", "Christian"), ("female_adult", "Hannah"),
          ("male_adult", "Christian1"), ("female_adult", "Hannah2"))


def cumdist(rt):
    c = [0.0]
    for i in range(1, len(rt)):
        c.append(c[-1] + math.dist(rt[i], rt[i - 1]))
    return c


def crosswalks_on_route(rt, cum, cws):
    """경로가 지나는 횡단보도 [(s_route, cw)] — 경로 순서."""
    out = []
    for c in cws:
        i = min(range(len(rt)), key=lambda k: (rt[k][0] - c["x"]) ** 2 + (rt[k][1] - c["y"]) ** 2)
        if math.dist(rt[i], (c["x"], c["y"])) <= ON_ROUTE:
            out.append((cum[i], c))
    out.sort(key=lambda q: q[0])
    return out


def pick(cands, total, count, min_sep=MIN_SEP):
    """앞에서부터 서로 min_sep 이상 떨어진 것을 count 개까지."""
    chosen = []
    for s, c in cands:
        if s < SKIP_HEAD or s > total - SKIP_TAIL:
            continue
        if any(abs(s - q[0]) < min_sep for q in chosen):
            continue
        chosen.append((s, c))
        if len(chosen) >= count:
            break
    return chosen


def drivable_at(mp, px, py):
    """그 점이 **어느 도로든** 주행차로 위인가(교차로는 연결로가 겹쳐 도로가 여럿이다)."""
    gx, gy = int(px // CELL), int(py // CELL)
    for a in range(gx - 1, gx + 2):
        for b in range(gy - 1, gy + 2):
            for x, y, h, rd, s in mp.grid.get((a, b), ()):
                if math.hypot(px - x, py - y) > 30.0:
                    continue
                u = (px - x) * math.cos(h) + (py - y) * math.sin(h)
                if abs(u) > 1.5:
                    continue
                t = (px - x) * -math.sin(h) + (py - y) * math.cos(h)
                for _lid, typ, lo, hi, _m in mp.lanes(rd, s + u):
                    if typ in DRIVABLE and lo - 1e-6 <= t <= hi + 1e-6:
                        return True
    return False


def carriageway_span(mp, x, y, h, step=0.25, maxd=25.0):
    """(x,y) 를 지나 도로 횡(t)축으로 **차도가 이어지는 구간** [t_lo, t_hi].

    ⚠️ 그 횡단보도가 놓인 road 의 차로만 보면 안 된다. 교차로 안 횡단보도는 **연결로**
       (폭 3~4.5m) 위에 찍혀 있는데, 실제 건널 차도는 옆 연결로들까지 합친 폭이다
       (실측 2026-09-09: road 2898 의 신호 횡단보도가 '폭 4.5m' 로 나와 사람이 교차로
       한가운데서 출발할 뻔했다). 어느 도로든 주행차로면 차도로 본다.
    """
    nx, ny = -math.sin(h), math.cos(h)
    if not drivable_at(mp, x, y):
        return None

    def reach(sign):
        far, k, gap = 0.0, 1, 0
        while k * step <= maxd:
            d = k * step
            if drivable_at(mp, x + sign * nx * d, y + sign * ny * d):
                far, gap = d, 0
            else:
                gap += 1
                if gap * step > GAP_BRIDGE:        # 좁은 중앙분리대·경계선은 건너뛴다
                    break
            k += 1
        return far
    return -reach(-1.0), reach(+1.0)


def crossing_path(mp, cw, start_right):
    """횡단보도 하나의 건널 길 (x0,y0)->(x1,y1) 와 걷는 방향(rad), 노면 고도.

    횡단보도 본체 좌표(DB 의 x,y = 도색 본체 중심)를 지나는 도로 횡(t)축이 곧 횡단보도
    축이다. **차도가 이어지는 구간**(carriageway_span) 양끝 바깥 EDGE_OUT(인도 위)에서
    시작해 반대편 그만큼 밖까지 간다. start_right 면 도로 진행방향 기준 **오른쪽(t<0)**
    끝에서 출발한다.
    """
    rd = mp.roads.get(str(cw["road"]))
    if rd is None:
        return None
    s = float(cw["s"])
    _rx, _ry, h = road_point(rd, s)
    x, y = float(cw["x"]), float(cw["y"])
    span = carriageway_span(mp, x, y, h)
    if span is None:
        # 본체 좌표가 차도 밖(도색이 인도에 걸친 경우) — 그 road 의 차로 범위로 대신한다
        lanes = mp.lanes(rd, s)
        if not lanes:
            return None
        x, y = _rx, _ry
        span = (min(l[2] for l in lanes), max(l[3] for l in lanes))
    lo, hi = span[0] - EDGE_OUT, span[1] + EDGE_OUT
    nx, ny = -math.sin(h), math.cos(h)           # t 축(+ 가 도로 왼쪽)
    t0, t1 = (lo, hi) if start_right else (hi, lo)
    p0 = (x + nx * t0, y + ny * t0)
    p1 = (x + nx * t1, y + ny * t1)
    yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0]) % (2 * math.pi)
    z = elevation(rd, s) + Z_LIFT
    return p0, p1, yaw, z, (hi - lo)


def waypoints(pts, z):
    """PathShape 안의 `<Waypoint>` 들. pts = [(x, y), ...] (2점 이상)."""
    f = "{:.16e}".format
    out = []
    for k, (x, y) in enumerate(pts):
        nxt = pts[min(k + 1, len(pts) - 1)]
        prv = pts[max(k - 1, 0)]
        yaw = math.atan2(nxt[1] - prv[1], nxt[0] - prv[0]) % (2 * math.pi)
        out.append(f'<Waypoint X="{f(x)}" Y="{f(y)}" Options="0x00000000" Z="{f(z)}" '
                   f'Weight="1.0000000000000000e+00" Yaw="{f(yaw)}" '
                   f'Pitch="0.0000000000000000e+00" Roll="0.0000000000000000e+00"/>')
    return "".join(out)


def character_block(shape_id, name, ctype, appear, pts, z, trig, speed, move="walk"):
    """길을 따라 가는 캐릭터 하나(PathShape + Character + CharacterActions).

    `move` 는 VTD 가 받는 동작 이름이다. 샘플 시나리오에서 실제로 쓰인 것만 쓴다
    (2026-09-09 실측: walk · jog · run · wait · **bicycle_ride** · listen · talk).
    ★`bicycle_ride` 가 **달리는 자전거**다 — `BicycleDemo.xml` 이 그렇게 만든다.
      휠체어는 동작이 없다(정지 오브젝트 `wheelchair_adult` 로만 놓을 수 있다).
    """
    f = "{:.16e}".format
    x0, y0 = pts[0]
    yaw0 = math.atan2(pts[1][1] - y0, pts[1][0] - x0) % (2 * math.pi)
    return (f'<PathShape ShapeId="{shape_id}" ShapeType="polyline" Closed="false" '
            f'Name="Path{name}">{waypoints(pts, z)}</PathShape>'
            f'<Character CharacterType="{ctype}" Class="pedestrian" Appearance="{appear}" Name="{name}">'
            f'<StartPosAbs X="{f(x0)}" Y="{f(y0)}" Z="{f(z)}" Direction="{f(yaw0)}"/></Character>'
            f'<CharacterActions Character="{name}"><Action Name="">'
            f'<PosRelative CounterID="" CounterComp="COMP_EQ" NetDist="false" Distance="{f(trig)}" '
            f'CounterVal="0" Pivot="Ego"/>'
            f'<Motion Move="{move}" Rate="0.0000000000000000e+00" Speed="{f(speed)}" Force="true" '
            f'ExecutionTimes="1" ActiveOnEnter="true" DelayTime="0.0000000000000000e+00"/>'
            f'<CharacterPath Loop="false" PathShape="{shape_id}" ExecutionTimes="1" ActiveOnEnter="true" '
            f'DelayTime="0.0000000000000000e+00" Beam="true" ClampToGround="true"/>'
            f'</Action></CharacterActions>')


def xml_block(i, p0, p1, yaw, z, ctype, appear, trig, speed):
    return character_block(i, f"CwPed{i}", ctype, appear, [p0, p1], z, trig, speed, move="walk")


def insert(src, block):
    if "<MovingObjectsControl />" in src:
        return src.replace("<MovingObjectsControl />",
                           f"<MovingObjectsControl>{block}</MovingObjectsControl>", 1)
    if "</MovingObjectsControl>" in src:
        return src.replace("</MovingObjectsControl>", block + "</MovingObjectsControl>", 1)
    raise SystemExit("MovingObjectsControl 앵커가 없다 — 이 판에는 못 심는다")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("xodr")
    ap.add_argument("course", help="예: HL_FMA_NEW_A (scenarios/<course>_TR.xml 에 심는다)")
    ap.add_argument("--count", type=int, default=4, help="심을 횡단보도 수")
    ap.add_argument("--trigger", type=float, default=45.0, help="자차가 이 거리 안에 오면 건너기 시작[m]")
    ap.add_argument("--speed", type=float, default=1.3, help="보행 속도[m/s]")
    ap.add_argument("--src", default=None, help="기본 scenarios/<course>_TR.xml")
    ap.add_argument("--out", default=None, help="기본 scenarios/<course>_TRP.xml")
    ap.add_argument("--appearance", default=None,
                    help="외형 하나로 통일(예: Hannah1). 모델이 안 뜰 때 바꿔 본다")
    a = ap.parse_args()

    rt = json.load(open(f"{HERE}/../routes/{a.course}.json", encoding="utf-8"))["ego_route"]
    cws = json.load(open(f"{HERE}/../routes/crosswalks.json", encoding="utf-8"))["crosswalks"]
    mp = Map(a.xodr)
    cum = cumdist(rt)
    cands = crosswalks_on_route(rt, cum, cws)
    chosen = pick(cands, cum[-1], a.count)
    if not chosen:
        raise SystemExit(f"{a.course}: 경로 위 횡단보도를 못 골랐다(후보 {len(cands)})")

    src_path = a.src or f"{HERE}/../scenarios/{a.course}_TR.xml"
    out_path = a.out or f"{HERE}/../scenarios/{a.course}_TRP.xml"
    src = open(src_path, encoding="utf-8").read()
    blocks, made = [], []
    for k, (s, cw) in enumerate(chosen, 1):
        start_right = (k % 2 == 1)
        got = crossing_path(mp, cw, start_right)
        if got is None:
            print(f"⚠️ road {cw['road']} s={cw['s']}: 도로를 못 찾아 생략", file=sys.stderr)
            continue
        p0, p1, yaw, z, width = got
        ctype, appear = APPEAR[(k - 1) % len(APPEAR)]
        if a.appearance:
            appear = a.appearance
        blocks.append(xml_block(k, p0, p1, yaw, z, ctype, appear, a.trigger, a.speed))
        made.append(f"CwPed{k} [{ctype}/{appear}] s={s:.0f}m road {cw['road']} "
                    f"{'신호' if cw.get('signal') else '무신호'} 폭 {width:.1f}m "
                    f"{'우->좌' if start_right else '좌->우'} ({p0[0]:.1f},{p0[1]:.1f})->({p1[0]:.1f},{p1[1]:.1f}) Z={z:.1f}")
    if not blocks:
        raise SystemExit("아무것도 못 심었다")
    open(out_path, "w", encoding="utf-8").write(insert(src, "".join(blocks)))
    print(f"-> {out_path}\n   경로 위 횡단보도 {len(cands)}곳 중 {len(blocks)}곳:\n   " + "\n   ".join(made))


if __name__ == "__main__":
    main()
