#!/usr/bin/env python3
"""주행 CSV를 **차로 기준**으로 채점한다 — 화면을 안 보고도 잡히게.

왜 만들었나(2026-08-15): 기존 채점기(check_run.py)는 도로·차로 개념이 없어서
신호위반 0 · 접촉 0 으로 만점이 나오는데도 실제 주행은 엉망일 수 있었다.
사용자가 화면을 보고 직접 잡아낸 항목들 — 지정차로 위반, 직진 녹색에 좌회전,
깜빡이 없이 차선 변경, 차선을 밟은 채 주행 — 이 전부 여기 해당한다.

검사 항목
  ① 차선 밟기      : OBB가 차선 경계를 잠정 0.10m·0.30초 이상 침범한 구간
  ② 중앙선 침범    : OBB가 정상 차로군 안쪽 경계를 0.6m 이상 넘은 구간
  ③ 지정차로       : **미구현.** 경로 단계에서 `eval/check_route.py` 의 ⑨ 가 본다
                     (주행 CSV 만으로는 '그 차로에서 그 회전이 허용되나'를 못 판정한다)
  ④ 신호 종류      : 좌회전을 직진 녹색에 했는가 / 직진을 좌회전 화살표에 했는가
  ⑤ 방향지시등     : 차로를 바꾸면서 깜빡이를 켰는가 (CSV 에 turn 기록 필요)
  ⑥ 보도 침범      : OBB와 OpenDRIVE sidewalk가 0.5m 이상 겹쳤는가

사용: python3 check_lanes.py <xodr> <run.csv> [run2.csv ...]
      (CSV 여러 개를 주면 지도를 한 번만 읽고 연달아 채점한다 — 지도 로딩이 몇 분 걸린다)
"""
import csv
import math
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0] + "/../vtd")
sys.path.insert(0, __file__.rsplit("/", 1)[0] + "/../src")
from build_lane_plan import Map                                  # noqa: E402
from build_tl_map import road_point                              # noqa: E402
from run_logger import (EGO_CX, EGO_HALF_L, EGO_HALF_W)          # noqa: E402

TL_GREEN, TL_LEFT, TL_GREEN_LEFT = 3, 4, 5
TURN_MIN = math.radians(25)
SIGNAL_WINDOW = 3.0        # 차로 변경 전후 이 시간 안에 깜빡이가 켜져 있으면 인정
SPAWN_SKIP_M = 8.0         # 출발점에서 이만큼 갈 때까지는 판정 제외(스폰이 차선을 문다)
CENTER_M, CENTER_S = 0.6, 0.6
WALK_M, WALK_S = 0.5, 0.3
# 주최측은 차선을 밟는 즉시가 아니라 "유의미하게 이탈"했을 때 감점한다고만
# 답했고 내부 문턱은 공개하지 않았다. 아래 둘은 그 불확실성을 한곳에 격리한 잠정값이다.
# 공식/실측 기준을 확보하면 이 두 값만 교정한다.
LANE_EDGE_M, LANE_EDGE_S = 0.10, 0.30


def obb_lateral_interval(ego_t, ego_heading, road_heading):
    """도로 횡축에 투영한 ego OBB의 ``(중심, 반경, 최솟값, 최댓값)``.

    로그의 위치는 차체 중심이 아니라 후륜축 중심이다. 차체 치수와 후륜축 오프셋은
    ``run_logger``를 단일 원천으로 사용한다. 한 road의 ego s 접선에서 선형화한 값이라
    급곡선에서는 전후 모서리의 실제 road-t와 작은 차이가 날 수 있다.
    """
    delta = ego_heading - road_heading
    body_center_t = ego_t + EGO_CX * math.sin(delta)
    lateral_radius = (EGO_HALF_L * abs(math.sin(delta))
                      + EGO_HALF_W * abs(math.cos(delta)))
    return (body_center_t, lateral_radius,
            body_center_t - lateral_radius, body_center_t + lateral_radius)


def _interval_overlap_depth(lo, hi, ranges):
    """1차원 구간 ``[lo, hi]``와 대상 구간들의 최대 겹침 깊이."""
    return max((max(0.0, min(hi, b) - max(lo, a)) for a, b in ranges),
               default=0.0)


def load(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append(dict(t=float(r["t"]), x=float(r["x"]), y=float(r["y"]),
                             h=float(r["heading"]), v=float(r["v"]),
                             tl=int(r["tl_state"]), tid=int(r["tl_id"]),
                             reason=r.get("reason", ""),
                             sig=int(r["sig"]) if r.get("sig") not in (None, "") else None,
                             # 경로 기준 횡위치. 차로 유지/스폰 진단용이며 옛 로그엔 없을 수 있다.
                             d_ego=float(r["d_ego"]) if r.get("d_ego") not in (None, "") else None))
    return rows


def _runs_of(rows, pred, min_sec=0.3):
    """조건이 연속으로 참인 구간. ``min_sec`` 미만은 판정 잡음으로 버린다."""
    out, cur = [], None
    for r in rows:
        if pred(r):
            cur = cur or [r["t"], r["t"], r["x"], r["y"]]
            cur[1] = r["t"]
        elif cur:
            if cur[1] - cur[0] >= min_sec - 1e-9:
                out.append(cur)
            cur = None
    if cur and cur[1] - cur[0] >= min_sec - 1e-9:
        out.append(cur)
    return out


def centerline_violations(rows):
    """OBB가 중앙선을 0.6m 이상 침범한 0.6초 이상 구간."""
    return _runs_of(rows,
                    lambda r: (r.get("center_intrusion") is not None
                               and r["center_intrusion"] + 1e-9 >= CENTER_M),
                    CENTER_S)


def sidewalk_violations(rows):
    """OBB가 sidewalk와 0.5m 이상 겹친 0.3초 이상 구간."""
    return _runs_of(rows,
                    lambda r: (r.get("sidewalk_intrusion") is not None
                               and r["sidewalk_intrusion"] + 1e-9 >= WALK_M),
                    WALK_S)


def lane_change_events(rows, boundary_tolerance=0.5):
    """OBB가 기존 driving-lane 경계에 실제로 진입한 사건을 반환한다.

    반환값은 ``[(진입행, 직전행, roadMark)]``다. 후륜 기준점의 ``lane`` 변경은
    완료 차로를 설명하는 보조 정보일 뿐 사건의 선행조건이 아니다. 따라서 OBB가 실선을
    넘었지만 후륜축은 원래 차로에 남은 경우도 검출한다. 처음부터 선과 겹친 채 시작했거나
    laneSection에서 경계가 새로 생긴 경우는 '가로지름'으로 세지 않는다.
    """
    raw = []
    prev = None
    for row_index, r in enumerate(rows):
        if r.get("lane") is None:
            prev = None                       # 교차로/미매핑 구간을 가로질러 잇지 않는다
            continue
        if (prev is not None and r.get("road") == prev.get("road")
                and not prev.get("off") and not r.get("off")
                and not prev.get("classification_hold_reason")
                and not r.get("classification_hold_reason")):
            plo, phi = prev.get("obb_t_min"), prev.get("obb_t_max")
            rlo, rhi = r.get("obb_t_min"), r.get("obb_t_max")
            if None not in (plo, phi, rlo, rhi):
                for b, mk in (r.get("_bnd") or []):
                    prior = min((q for q, _ in (prev.get("_bnd") or [])),
                                key=lambda q: abs(q - b), default=None)
                    if prior is None or abs(prior - b) > boundary_tolerance:
                        continue                    # laneSection에서 방금 생긴 경계
                    # 직전 프레임에는 OBB 전체가 경계 한쪽에 있었고 현재 프레임에는
                    # 반대쪽으로 일부라도 들어갔을 때만 '진입'이다. 계속 선을 문 상태,
                    # 정지 상태, 시작 프레임부터 겹친 상태는 사건을 만들지 않는다.
                    entered_from_low = (phi <= prior + 1e-9 and rhi > b + 1e-9)
                    entered_from_high = (plo >= prior - 1e-9 and rlo < b - 1e-9)
                    if entered_from_low or entered_from_high:
                        r["lane_change_boundary_t"] = b
                        r["lane_change_boundary_mark"] = mk
                        r["obb_boundary_crossing"] = True
                        r["lane_change_direction"] = 1 if entered_from_low else -1
                        r["lane_change_from_lane"] = prev.get("lane")

                        # 후륜축까지 경계를 넘었다면 완료 차로를 부가 정보로 남긴다.
                        # 아직 OBB 앞/옆 모서리만 넘은 사건은 None인 채로도 유효하다.
                        direction = r["lane_change_direction"]
                        rpos = r.get("lat")
                        to_lane = (r.get("lane")
                                   if (r.get("lane") != prev.get("lane")
                                       and rpos is not None
                                       and direction * (rpos - b) > 0.0)
                                   else None)
                        if to_lane is None:
                            for q in rows[row_index + 1:]:
                                if (q.get("road") != r.get("road") or q.get("off")
                                        or q.get("classification_hold_reason")):
                                    break
                                qb = min((x for x, _ in (q.get("_bnd") or [])),
                                         key=lambda x: abs(x - b), default=None)
                                if qb is None or abs(qb - b) > boundary_tolerance:
                                    break
                                qpos = q.get("lat")
                                if (q.get("lane") != prev.get("lane")
                                        and qpos is not None
                                        and direction * (qpos - qb) > 0.0):
                                    to_lane = q.get("lane")
                                    break
                                # OBB가 원래 쪽으로 완전히 빠졌으면 이 시도는 미완료다.
                                qlo, qhi = q.get("obb_t_min"), q.get("obb_t_max")
                                if qlo is None or qhi is None:
                                    break
                                if ((direction > 0 and qhi <= qb + 1e-9)
                                        or (direction < 0 and qlo >= qb - 1e-9)):
                                    break
                        r["lane_change_to_lane"] = to_lane
                        raw.append((r, prev, mk))
        prev = r
    return raw


def main():
    args = sys.argv[1:]
    # 첫 인자가 xodr 이면 다중 모드, 아니면 옛 방식(csv xodr)
    if args and args[0].endswith(".xodr"):
        xodr, paths = args[0], args[1:]
    else:
        xodr, paths = args[1], args[:1]
    mp = Map(xodr)
    for k, path in enumerate(paths):
        if len(paths) > 1:
            print(f"\n{'='*12} {path.rsplit('/',1)[-1]} {'='*12}")
        score_one(path, mp)


def classify(rows, mp):
    """CSV 각 프레임에 **차로 판정**을 붙인다(제자리 수정).

    붙는 키: road/junc/lane/lat/to_left/to_right/road_w/off/edge/mark/wrong_way/_bnd,
    body_center_t/lateral_radius/obb_t_min/obb_t_max/center_intrusion/
    sidewalk_intrusion/classification_hold_reason.
    `eval/score_fma.py`(대회 채점기)도 이 판정을 그대로 쓴다 — 지도 해석이 두 벌이면
    두 채점기가 서로 다른 답을 내고, 그러면 둘 다 못 믿는다.
    """
    for r in rows:
        # 재분류해도 이전 프레임의 진단값이 남지 않도록 전부 초기화한다.
        r.update(road=None, junc=None, lane=None, lat=None, road_heading=None,
                 heading_delta=None, body_center_t=None, lateral_radius=None,
                 obb_t_min=None, obb_t_max=None, center_boundary_t=None,
                 center_intrusion=None, sidewalk_intrusion=None,
                 sidewalk_ranges=[], classification_hold_reason=None,
                 edge=None, opp=False, off=False, wrong_way=False,
                 mark=None, _bnd=[], _obb_bnd=[],
                 lane_change_boundary_t=None, lane_change_boundary_mark=None,
                 obb_boundary_crossing=False, lane_change_direction=None,
                 lane_change_from_lane=None, lane_change_to_lane=None,
                 lane_intrusion=None)
        loc = mp.locate(r["x"], r["y"])
        if loc is None:
            r["classification_hold_reason"] = "road_not_located"
            continue
        rd, s, t, h = loc
        r["road"] = rd.get("id"); r["junc"] = rd.get("junction", "-1")
        r["lat"] = t
        r["road_heading"] = h
        r["heading_delta"] = r["h"] - h
        (_ct, _rad, _obb_lo, _obb_hi) = obb_lateral_interval(t, r["h"], h)
        r["body_center_t"], r["lateral_radius"] = _ct, _rad
        r["obb_t_min"], r["obb_t_max"] = _obb_lo, _obb_hi
        if r["junc"] != "-1":
            r["classification_hold_reason"] = "junction_overlap"
            continue                                   # 교차로는 연결로 중첩 -> 판정 보류
        lanes = mp.lanes(rd, s)
        if not lanes:
            r["classification_hold_reason"] = "lanes_not_found"
            continue

        # 보도는 border/shoulder 등 다른 비주행 lane과 구분한다. 겹침 깊이는 OBB를
        # road-t에 투영한 구간과 OpenDRIVE sidewalk lane 구간의 교집합 길이다.
        _walks = [(l[2], l[3]) for l in lanes if l[1] == "sidewalk"]
        r["sidewalk_ranges"] = _walks
        r["sidewalk_intrusion"] = _interval_overlap_depth(_obb_lo, _obb_hi, _walks)

        # 차량 방위가 가리키는 정상 진행방향의 driving lane군을 고르고 그 안쪽 경계를
        # 중앙선으로 본다. 후륜축이 반대 차로에 들어갈 때까지 기다리지 않는다.
        _fwd = math.cos(r["h"] - h) >= 0.0
        _normal = [l for l in lanes if l[1] == "driving" and (l[0] < 0) == _fwd]
        _opposite = [l for l in lanes if l[1] == "driving" and (l[0] > 0) == _fwd]
        if _normal and _opposite:
            if _fwd:
                _center = max(l[3] for l in _normal)
                _depth = max(0.0, _obb_hi - _center)
            else:
                _center = min(l[2] for l in _normal)
                _depth = max(0.0, _center - _obb_lo)
            r["center_boundary_t"] = _center
            r["center_intrusion"] = _depth
        else:
            # 편도에는 중앙선 자체가 없다. 이는 판정보류가 아니라 침범 대상 없음이다.
            r["center_intrusion"] = 0.0

        mine = next((l for l in lanes if l[2] - 1e-6 <= t <= l[3] + 1e-6), None)
        if mine is None:
            r["off"] = True
            continue
        r["lane"] = mine[0]
        # ★같은 방향 차도의 **좌우 끝까지 거리**. 차로 번호는 차로가 늘거나 줄면
        #   차가 가만히 있어도 바뀌지만, 이 값은 안 바뀐다(아래 ⑤ 판정에 쓴다).
        _same = [l for l in lanes if l[1] == "driving" and (l[0] > 0) == (mine[0] > 0)]
        if _same:
            _lo, _hi = min(l[2] for l in _same), max(l[3] for l in _same)
            r["to_left"], r["to_right"] = ((_hi - t, t - _lo) if _fwd else (t - _lo, _hi - t))
            r["road_w"] = _hi - _lo
        # ★★**넘은 경계는 좌표로 센다.** 차로 번호는 laneSection 이 바뀌거나 차로가
        #   늘면 차가 가만히 있어도 재부여된다 — 실측 2026-09-04 사전테스트1 정차 구간:
        #   `-3 -> -4` 로 번호가 바뀌었는데 도로기준 t 는 -7.37 -> -7.57, **0.2m** 였다
        #   (road_w 가 5.95 -> 8.99 로 넓어지는 중이었다). 번호만 보면 그게 '실선
        #   차로변경 -3' 으로 찍힌다. 경계는 좌표라 그런 식으로 흔들리지 않는다.
        #   각 항목 = (경계 t, 그 경계의 표시). 표시는 **|id| 가 작은 쪽 차로**의
        #   roadMark 다(roadMark = 그 차로의 바깥쪽 경계).
        _bnd = []
        for _a in sorted(_same, key=lambda l: abs(l[0])):
            _out = _a[3] if _a[0] > 0 else _a[2]
            if any(abs(_out - (_b[2] if _b[0] > 0 else _b[3])) < 1e-6
                   and abs(_b[0]) > abs(_a[0]) for _b in _same):
                _bnd.append((_out, _a[4]))
        r["_bnd"] = _bnd
        r["_obb_bnd"] = [(b, mk) for b, mk in _bnd
                          if _obb_lo - 1e-9 <= b <= _obb_hi + 1e-9]
        r["off"] = mine[1] != "driving"
        # 차체가 경계를 무는가. `_gap` = 뒷축에서 가까운 경계까지[m] — **얼마나** 물었는지를
        # 남겨야 지도 해상도 문제와 경로가 치우친 것을 나중에 가를 수 있다.
        r["_gap"] = min(t - mine[2], mine[3] - t)
        r["_lane_w"] = mine[3] - mine[2]
        # 현재 차로 양쪽 경계를 기준으로 OBB가 차로 밖으로 나간 최대 깊이. 단순히
        # 후륜축 거리와 반폭만 비교하지 않아 차량이 비스듬한 경우도 차체 전체로 본다.
        r["lane_intrusion"] = max(0.0, mine[2] - _obb_lo, _obb_hi - mine[3])
        r["edge"] = r["lane_intrusion"] > 1e-9
        r["mark"] = mine[4]
        # ⚠️ 차로 번호의 부호는 **도로마다** 규칙이 다르다(좌측 차선군은 도로 s 와 반대로 달림).
        #    도로가 바뀌면 번호를 비교하면 안 되고, '역주행'도 부호로 판정하면 안 된다.
        #    주행 방위와 도로 방위를 비교해 순/역행을 직접 본다.
        r["wrong_way"] = (mine[1] == "driving"
                          and (mine[0] > 0) == _fwd)  # 기존 기준점 역주행 진단(채점에는 미사용)
    return rows


def score_one(path, mp):
    rows = load(path)
    if not rows:
        print("빈 CSV"); return
    classify(rows, mp)

    known = [r for r in rows if r.get("lane") is not None]
    if not known:
        print("차로를 판정할 수 있는 프레임이 없음"); return
    t0 = rows[0]["t"]

    # ★★**출발 구간은 판정에서 뺀다.** ego 는 차선을 물고 스폰된다 — 실측 2026-08-27:
    #   주변교통 6판 **전부** `t=0.0~2.5s` 가 '보도/경계 침범'으로, 그리고 첫 차로판정이
    #   `+1 -> +2`(또는 `+1 -> -1`) '무신호 차로변경'으로 찍혔다. 우리가 한 게 아니라
    #   시뮬레이터가 거기에 놓은 것이다. 고칠 수 없는 것을 위반으로 세면 영구 감점으로 남는다.
    #   (횡단보도 채점기도 같은 이유로 출발지점 정지선 하나를 이미 빼고 있다.)
    #   ⚠️ 시간이 아니라 **이동거리**로 자른다 — 출발이 늦어지는 판이 있다.
    sx, sy = rows[0]["x"], rows[0]["y"]
    t_go = None
    for q in rows:
        if math.hypot(q["x"] - sx, q["y"] - sy) > SPAWN_SKIP_M:
            t_go = q["t"]
            break
    if t_go is not None and t_go > t0:
        n0 = len(rows)
        rows = [q for q in rows if q["t"] >= t_go]
        known = [r for r in rows if r.get("lane") is not None]
        print(f"    (출발 {SPAWN_SKIP_M:.0f}m 까지 {t_go - t0:.1f}초 {n0 - len(rows)}프레임 제외 — 스폰이 차선을 문다)")

    def runs_of(pred):
        """조건이 연속으로 참인 구간 [(t시작, t끝, 대표 x,y)] — 0.3초 미만은 잡음으로 버린다."""
        return _runs_of(rows, pred)

    print(f"프레임 {len(rows)}  차로판정 가능 {len(known)}  "
          f"(교차로/판정보류 {len(rows)-len(known)})")

    # ① 차선 밟기 — 단, **차로를 바꾸는 중**에 선을 넘는 건 정상이라 제외한다.
    detected_changes = lane_change_events(rows)
    chg_t = [r["t"] for r, _p, _mk in detected_changes]

    def changing(r):
        return any(abs(r["t"] - c) < 2.5 for c in chg_t)

    edge = _runs_of(
        rows,
        lambda r: (r.get("lane_intrusion") is not None
                   and r["lane_intrusion"] + 1e-9 >= LANE_EDGE_M
                   and not changing(r)),
        LANE_EDGE_S)
    tot = sum(b - a for a, b, _, _ in edge)
    print(f"\n[① 차선 밟기·잠정 {LANE_EDGE_M:.2f}m/{LANE_EDGE_S:.2f}s]  "
          f"{len(edge)}구간 {tot:.1f}초"
          + ("  ← ❌ 차체가 차선을 물고 달림" if edge else "  ✅"))
    for a, b, x, y in edge[:4]:
        print(f"    {a-t0:6.1f}~{b-t0:6.1f}s ({x:.0f},{y:.0f})")

    # 일반 도로 이탈은 참고 진단으로 유지하되, 공식 보도 침범과 섞지 않는다.
    off = runs_of(lambda r: r.get("off") is True)
    tot_off = sum(b - a for a, b, _, _ in off)
    print(f"[참고: 기준점 주행차로 이탈]  {len(off)}구간 {tot_off:.1f}초")
    for a, b, x, y in off[:4]:
        print(f"    {a-t0:6.1f}~{b-t0:6.1f}s ({x:.0f},{y:.0f})")

    walk = sidewalk_violations(rows)
    tot_walk = sum(b - a for a, b, _, _ in walk)
    print(f"[⑥ 보도 침범]  {len(walk)}구간 {tot_walk:.1f}초"
          + ("  ← ❌" if walk else "  ✅"))
    for a, b, x, y in walk[:4]:
        deep = max(r.get("sidewalk_intrusion") or 0.0 for r in rows if a <= r["t"] <= b)
        print(f"    {a-t0:6.1f}~{b-t0:6.1f}s ({x:.0f},{y:.0f}) 깊이 {deep:.2f}m")

    # ② 중앙선 침범 = 정상 차로군 안쪽 경계를 넘은 OBB 깊이.
    # ★주최측 공식 답변: 장애물 유무, 점선/실선, 복귀 시간과 관계없이 전부 위반이다.
    opp = centerline_violations(rows)
    tot_opp = sum(b - a for a, b, _, _ in opp)
    print(f"[② 중앙선 침범]  {len(opp)}구간 {tot_opp:.1f}초"
          + ("  ← ❌" if tot_opp > 0.5 else "  ✅"))
    for a, b, x, y in opp[:4]:
        deep = max(r.get("center_intrusion") or 0.0 for r in rows if a <= r["t"] <= b)
        print(f"    {a-t0:6.1f}~{b-t0:6.1f}s ({x:.0f},{y:.0f}) 깊이 {deep:.2f}m")

    # ③⑤ 차로 변경: 언제 몇 번에서 몇 번으로, 깜빡이는 켰나
    print("\n[⑤ 차로 변경과 방향지시등]")
    changes = [(r, r.get("lane_change_from_lane", p.get("lane")),
                r.get("lane_change_to_lane"))
               for r, p, _mk in detected_changes]
    if not changes:
        print("    차로 변경 없음")
    has_sig = any(r["sig"] is not None for r in rows)
    bad_sig = 0
    for r, a, b in changes:
        target = f"{b:+d}" if b is not None else "경계 반대편(후륜 미전환)"
        if not has_sig:
            print(f"    {r['t']-t0:6.1f}s  lane {a:+d} -> {target}  "
                  "(구버전 로그 — sig 컬럼 없음)")
            continue
        ok = any(q["sig"] for q in rows if abs(q["t"] - r["t"]) < SIGNAL_WINDOW)
        if not ok:
            bad_sig += 1
        print(f"    {r['t']-t0:6.1f}s  lane {a:+d} -> {target}  "
              f"깜빡이 {'켬' if ok else '❌안 켬'}  ({r['x']:.0f},{r['y']:.0f}) [{r['reason']}]")
    if changes and has_sig:
        print(f"    -> 깜빡이 없이 변경 {bad_sig}/{len(changes)}건"
              + ("  ← ❌" if bad_sig else "  ✅"))

    # ④ 신호 종류 대비 진행방향
    print("\n[④ 신호 종류]")
    # ⚠️ 회전 판정은 **거리 기준**으로 봐야 한다. 프레임 수로 보면 창이 너무 짧아
    #    8초에 걸친 56° 좌회전을 '직진'으로 읽는다(첫 판에서 실제로 그랬다).
    cumd = [0.0]
    for i in range(1, len(rows)):
        cumd.append(cumd[-1] + math.hypot(rows[i]["x"]-rows[i-1]["x"], rows[i]["y"]-rows[i-1]["y"]))

    def at_dist(i, d):
        j = i
        while 0 <= j < len(rows) and (cumd[j] - cumd[i]) * (1 if d > 0 else -1) < abs(d):
            j += 1 if d > 0 else -1
        return max(0, min(len(rows) - 1, j))

    # 방위는 CSV 의 heading 을 그대로 쓴다. 좌표로 다시 계산하면 저속 구간에서
    # 기준 거리가 몇 cm 라 잡음투성이가 되고, 실제로 56° 좌회전을 '직진'으로 읽었다.
    def hd(i):
        return rows[i]["h"]
    viol = 0
    for i in range(1, len(rows)):
        a, b = rows[i - 1], rows[i]
        if a["tid"] == 0 or b["tid"] == a["tid"]:
            continue                                     # 신호를 막 통과한 순간만 본다
        # 통과 전 25m 부터, 통과 후 여러 거리까지의 방위 변화 중 **가장 큰 것**으로 판정.
        # ⚠️ 고정 창(±25m)이면 회전이 뒤쪽에 몰린 경우를 놓친다 — 실측: 경로가 좌회전
        #    중간에 끝나서 56° 회전을 +25m 지점에서 11° 로만 읽고 '직진'이라고 했다.
        back = hd(at_dist(i, -25.0))
        dh = 0.0
        for d in (15.0, 25.0, 40.0, 60.0):
            cand = (hd(at_dist(i, d)) - back + math.pi) % (2 * math.pi) - math.pi
            if abs(cand) > abs(dh):
                dh = cand
        kind = "좌회전" if dh > TURN_MIN else ("우회전" if dh < -TURN_MIN else "직진")
        st = a["tl"]
        bad = ""
        if kind == "좌회전" and st == TL_GREEN:
            bad = "  ← ❌ 직진 녹색에 좌회전"; viol += 1
        elif kind == "직진" and st == TL_LEFT:
            bad = "  ← ❌ 좌회전 화살표에 직진"; viol += 1
        name = {0: "UNSET", 1: "RED", 2: "YELLOW", 3: "GREEN", 4: "LEFT",
                5: "GREEN_LEFT", 6: "FLASH"}.get(st, st)
        print(f"    tl{a['tid']:>4}  {name:11s} 에서 {kind}{bad}")
    if not viol:
        print("    ✅ 신호 종류 위반 없음")

    print(f"\n=== 차선밟기 {tot:.1f}s · 이탈 {tot_off:.1f}s · 중앙선 {tot_opp:.1f}s · "
          f"무신호변경 {bad_sig}건 · 신호종류위반 {viol}건 ===")


if __name__ == "__main__":
    main()
