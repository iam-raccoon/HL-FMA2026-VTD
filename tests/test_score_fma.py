"""대회 채점기(`eval/score_fma.py`) **자체**를 검증한다.

왜 필요한가: 검증 안 된 채점기는 없느니만 못하다. 실제로 처음 돌렸을 때
**세 건이 오탐**이었다 — 스폰 수렴을 차로변경으로, 옆 차로 자전거(17km/h)를
'보행자 무정차 통과' 로, 법이 시키는 대향차 양보를 '무의미 정차' 로 찍었다.
그 셋을 각각 여기서 못박는다. 합성 궤적으로 문턱을 하나씩 두드린다.
"""
import inspect
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "eval"))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

import score_fma as S                                   # noqa: E402


def straight(n=400, step=1.0):
    return [[i * step, 0.0] for i in range(n)]


def frames(n, v, x0=10.0, dt=0.04, **kw):
    out = []
    x = x0
    for i in range(n):
        x += v * dt
        r = dict(t=i * dt, x=x, y=0.0, h=0.0, v=v, tl=0, tid=-1,
                 reason="LANE_KEEP", sig=0, clr=99.9, d_ego=0.0, objs="", cap_by="")
        r.update({k: (val(i) if callable(val) else val) for k, val in kw.items()})
        out.append(r)
    return out


def run(rows, route=None, n_sec=1, lims=None, tl_stops=None, cws=None):
    route = route or straight()
    secs = S.Sections(route, n_sec)
    sheet = S.Sheet(secs.n)
    notes = []
    S.item_speed(rows, sheet, secs, lambda x, y: (lims or 13.889))
    S.item_traffic_light(rows, sheet, secs, tl_stops or {}, notes)
    S.item_contact(rows, sheet, secs)
    S.item_pedestrian(rows, sheet, secs)
    S.item_crosswalk_stop(rows, sheet, secs, cws or [])
    S.item_turn_signal(rows, sheet, secs)
    return sheet, notes


# ---------------------------------------------------------------- ① ② 속도
def test_제한속도_1kmh_까지는_봐준다():
    sheet, _ = run(frames(50, (50 + 0.9) / 3.6))
    assert sheet.penalty(0, 1) == 0, sheet.why[0]


def test_제한속도_1kmh_넘으면_경미():
    sheet, _ = run(frames(50, (50 + 1.5) / 3.6))
    assert sheet.penalty(0, 1) == S.MINOR, sheet.why[0]


def test_제한속도_20kmh_넘으면_중대():
    sheet, _ = run(frames(50, (50 + 21) / 3.6))
    assert sheet.penalty(0, 1) == S.MAJOR, sheet.why[0]


def test_보호구역은_항목2로_따로_센다():
    sheet, _ = run(frames(50, (30 + 2) / 3.6), lims=30 / 3.6)
    assert sheet.penalty(0, 2) == S.MINOR and sheet.penalty(0, 1) == 0


def test_보호구역_5kmh_초과는_중대():
    sheet, _ = run(frames(50, (30 + 6) / 3.6), lims=30 / 3.6)
    assert sheet.penalty(0, 2) == S.MAJOR


def test_같은_항목은_구간당_한번만_깎는다():
    rows = frames(400, (50 + 2) / 3.6)
    sheet, _ = run(rows, n_sec=1)
    assert sheet.score(0) == 100 - S.MINOR, sheet.score(0)


def test_구간이_나뉘면_구간마다_다시_깎인다():
    rows = frames(400, (50 + 2) / 3.6)
    sheet, _ = run(rows, n_sec=2)
    assert sheet.penalty(0, 1) == S.MINOR and sheet.penalty(1, 1) == S.MINOR


def test_경미가_중대로_심화되면_합_6점():
    rows = frames(100, (50 + 2) / 3.6) + frames(100, (50 + 25) / 3.6, x0=200.0)
    for k, r in enumerate(rows):
        r["t"] = k * 0.04
    sheet, _ = run(rows)
    assert sheet.penalty(0, 1) == S.MAJOR, sheet.why[0]


def _lane_rows(lat_of, bnd_of, lane_of, n=300, **extra):
    """차로변경 판정용 합성 프레임.

    ★`lat`(도로기준 횡위치)과 `_bnd`(경계 좌표+표시)가 있어야 한다 — 판정이
      **차로 번호가 아니라 경계 좌표**로 돌기 때문이다(2026-09-04).
    """
    rows = []
    for i in range(n):
        t = i * 0.04
        lat = lat_of(t)
        r = dict(t=t, x=10.0 + 8.0 * t, y=0.0, h=0.0, v=8.0, tl=0, tid=-1,
                 reason="LANE_KEEP", sig=0, clr=99.9, d_ego=0.0, objs="",
                 road="9", junc="-1", edge=False, off=False,
                 _gap=1.5, _lane_w=3.2,
                 lat=lat, obb_t_min=lat - S.HALF_W, obb_t_max=lat + S.HALF_W,
                 center_intrusion=0.0, sidewalk_intrusion=0.0,
                 classification_hold_reason=None,
                 _bnd=bnd_of(t), lane=lane_of(t))
        r.update({k: (v(t) if callable(v) else v) for k, v in extra.items()})
        rows.append(r)
    return rows


def test_차로가_늘어_번호만_바뀐_것은_차로변경이_아니다():
    """★★실측 2026-09-04 사전테스트1 정차 구간. 차로 번호는 `-3 -> -4` 로 바뀌었는데
    도로기준 t 는 -7.37 -> -7.57, **0.2m** 였다(road_w 5.95 -> 8.99, 차도가 넓어지는 중).
    번호만 보면 '실선 차로변경 -3' 으로 찍힌다. 실제로 그렇게 찍혔었다.

    여기서는 t=6.0 에 **안쪽에 경계가 하나 생기고** 번호가 재부여된다. 차는 안 움직인다."""
    route = straight()
    secs = S.Sections(route, 1)
    sheet = S.Sheet(1)
    rows = _lane_rows(
        lat_of=lambda t: -7.4 - 0.2 * min(max(t - 6.0, 0.0), 1.0),   # 0.2m 만 흐른다
        bnd_of=lambda t: ([(-6.4, "broken")] if t < 6.0
                          else [(-3.4, "broken"), (-6.4, "broken")]),  # 안쪽에 하나 생김
        lane_of=lambda t: -3 if t < 6.0 else -4)
    S.item_lane_geometry(rows, sheet, secs)
    assert sheet.penalty(0, 6) == 0, sheet.why[0]


def test_방금_생긴_경계는_넘은_것으로_안_센다():
    """차가 서 있는 **자리에** 경계가 생기는 경우. 넘은 게 아니다."""
    route = straight()
    secs = S.Sections(route, 1)
    sheet = S.Sheet(1)
    rows = _lane_rows(
        lat_of=lambda t: -5.0,
        bnd_of=lambda t: [] if t < 6.0 else [(-5.0, "solid")],
        lane_of=lambda t: -2 if t < 6.0 else -3)
    S.item_lane_geometry(rows, sheet, secs)
    assert sheet.penalty(0, 6) == 0, sheet.why[0]


def test_연석으로_밀린_것은_차로변경이_아니다():
    """★실측 코스 A 두 곳(t=215.6 road 2574 lane 1->2, t=291.7 road 2626 lane 2->3)이
    전부 `border`(연석)로 들어간 것이었다. 거기 붙은 `solid` 는 차로 사이 선이 아니라
    **길가장자리 구역선**이다. 그걸 '실선 차로변경' 으로 세면 안 된다(항목 5 가 볼 일)."""
    route = straight()
    secs = S.Sections(route, 1)
    sheet = S.Sheet(1)
    rows = _lane_rows(
        lat_of=lambda t: 1.6 if t < 6.0 else 4.8,       # 실제로 3.2m 옮겨간다
        bnd_of=lambda t: [(3.2, "solid")],
        lane_of=lambda t: 1 if t < 6.0 else 2,
        off=lambda t: t >= 6.0, d_ego=lambda t: 0.0 if t < 6.0 else 1.2)
    S.item_lane_geometry(rows, sheet, secs)
    assert sheet.penalty(0, 6) == 0, sheet.why[0]


def test_주행차로끼리_실선을_넘으면_잡는다():
    route = straight()
    secs = S.Sections(route, 1)
    sheet = S.Sheet(1)
    rows = _lane_rows(
        lat_of=lambda t: 1.6 if t < 6.0 else 4.8,
        bnd_of=lambda t: [(3.2, "solid")],
        lane_of=lambda t: 1 if t < 6.0 else 2,
        d_ego=lambda t: 0.0 if t < 6.0 else 3.2)
    S.item_lane_geometry(rows, sheet, secs)
    assert sheet.penalty(0, 6) == S.MINOR, sheet.why[0]


def test_점선을_넘는_것은_감점이_아니다():
    """실측 2026-09-04 사전테스트1: 정차하려고 -2 -> -3 으로 옮겼고 그 경계는 점선이었다."""
    route = straight()
    secs = S.Sections(route, 1)
    sheet = S.Sheet(1)
    rows = _lane_rows(
        lat_of=lambda t: 1.6 if t < 6.0 else 4.8,
        bnd_of=lambda t: [(3.2, "broken")],
        lane_of=lambda t: 1 if t < 6.0 else 2,
        d_ego=lambda t: 0.0 if t < 6.0 else 3.2)
    S.item_lane_geometry(rows, sheet, secs)
    assert sheet.penalty(0, 6) == 0, sheet.why[0]


def test_목표차로_전환없이_실선에_가까이_간_것은_감점이_아니다():
    route = straight()
    secs = S.Sections(route, 1)
    sheet = S.Sheet(1)
    rows = _lane_rows(
        lat_of=lambda t: 3.0,                         # OBB는 3.2m 실선과 겹친다
        bnd_of=lambda t: [(3.2, "solid")],
        lane_of=lambda t: 1)
    S.item_lane_geometry(rows, sheet, secs)
    assert sheet.penalty(0, 6) == 0, sheet.why[0]


def test_OBB는_실선을_넘었지만_후륜축은_원래차로여도_감점한다():
    route = straight()
    secs = S.Sections(route, 1)
    sheet = S.Sheet(1)
    rows = _lane_rows(
        lat_of=lambda t: 1.6 if t < 6.0 else 2.4,   # OBB 끝 3.343m, 경계 3.2m
        bnd_of=lambda t: [(3.2, "solid")],
        lane_of=lambda t: 1)
    S.item_lane_geometry(rows, sheet, secs)
    assert sheet.penalty(0, 6) == S.MINOR, sheet.why[0]
    assert "후륜 미전환" in sheet.why[0][6]


def test_실선을_넘어갔다_복귀하면_두번의_OBB_통과다():
    route = straight()
    secs = S.Sections(route, 1)
    sheet = S.Sheet(1)
    rows = _lane_rows(
        lat_of=lambda t: 1.6 if t < 6.0 or t >= 6.8 else 4.8,
        bnd_of=lambda t: [(3.2, "solid")],
        lane_of=lambda t: 1 if t < 6.0 or t >= 6.8 else 2)
    S.item_lane_geometry(rows, sheet, secs)
    assert sheet.penalty(0, 6) == S.MAJOR, sheet.why[0]


def test_실선_OBB_통과횟수는_구간마다_독립적으로_센다():
    route = straight(n=100)
    secs = S.Sections(route, 2)
    sheet = S.Sheet(2)
    rows = _lane_rows(
        lat_of=lambda t: 1.6 if t < 3.0 or t >= 7.0 else 4.8,
        bnd_of=lambda t: [(3.2, "solid")],
        lane_of=lambda t: 1 if t < 3.0 or t >= 7.0 else 2,
        n=250)
    S.item_lane_geometry(rows, sheet, secs)
    assert sheet.penalty(0, 6) == S.MINOR, sheet.why[0]
    assert sheet.penalty(1, 6) == S.MINOR, sheet.why[1]


def test_실험옵션_로그여도_중앙선침범은_항상_감점한다():
    """주행 전략 이름이나 점선 표시는 항목4 면책 사유가 아니다."""
    route = straight()
    secs = S.Sections(route, 1)
    sheet = S.Sheet(1)
    rows = _lane_rows(
        lat_of=lambda t: 1.6,
        bnd_of=lambda t: [],
        lane_of=lambda t: 1,
        center_intrusion=lambda t: 0.6 if 2.0 <= t <= 3.0 else 0.0,
        wrong_way=False,
        mark="broken",
        reason="CENTERLINE_ESCAPE_EXPERIMENT")
    S.item_lane_geometry(rows, sheet, secs)
    assert sheet.penalty(0, 4) == S.MAJOR, sheet.why[0]


def _metric_rows(metric, depth, duration):
    """스폰 제외 뒤 정확한 두 끝점으로 깊이/지속시간 경계를 두드린다."""
    times = [0.0, 1.1, 2.0, 2.0 + duration, 3.5]
    rows = []
    for t in times:
        r = dict(t=t, x=10.0 + 8.0 * t, y=0.0, h=0.0, v=8.0, tl=0, tid=-1,
                 reason="LANE_KEEP", sig=0, clr=99.9, d_ego=0.0, objs="",
                 road="9", junc="-1", lane=-1, lat=-1.6, off=False, edge=False,
                 _gap=1.5, _lane_w=3.2, _bnd=[],
                 obb_t_min=-2.543, obb_t_max=-0.657,
                 center_intrusion=0.0, sidewalk_intrusion=0.0,
                 classification_hold_reason=None)
        if 2.0 <= t <= 2.0 + duration:
            r[metric] = depth
        rows.append(r)
    return rows


def _lane_penalty(rows, item):
    secs = S.Sections(straight(), 1)
    sheet = S.Sheet(1)
    S.item_lane_geometry(rows, sheet, secs)
    return sheet.penalty(0, item)


def _lane_edge_rows(depth, duration):
    rows = _metric_rows("lane_intrusion", depth, duration)
    for r in rows:
        r.setdefault("lane_intrusion", 0.0)
        r["edge"] = r["lane_intrusion"] > 0.0
    return rows


def test_차로유지는_단일프레임_침범을_감점하지_않는다():
    rows = _lane_edge_rows(0.2, 0.0)
    assert _lane_penalty(rows, 3) == 0


def test_차로유지_잠정_깊이문턱_미만은_감점하지_않는다():
    assert _lane_penalty(_lane_edge_rows(S.LANE_EDGE_M - 0.01, 0.3), 3) == 0
    assert _lane_penalty(_lane_edge_rows(S.LANE_EDGE_M, 0.3), 3) == S.MINOR


def test_차로유지_잠정_시간문턱_미만은_감점하지_않는다():
    assert _lane_penalty(_lane_edge_rows(0.2, S.LANE_EDGE_S - 0.01), 3) == 0
    assert _lane_penalty(_lane_edge_rows(0.2, S.LANE_EDGE_S), 3) == S.MINOR


def test_차로유지_횟수는_구간마다_독립적으로_센다():
    route = straight(n=100)
    secs = S.Sections(route, 2)
    sheet = S.Sheet(2)
    rows = _lane_rows(
        lat_of=lambda _t: 1.6,
        bnd_of=lambda _t: [],
        lane_of=lambda _t: 1,
        n=250,
        lane_intrusion=lambda t: 0.2 if 2.5 <= t <= 3.0 or 7.0 <= t <= 7.5 else 0.0,
        edge=lambda t: 2.5 <= t <= 3.0 or 7.0 <= t <= 7.5)
    S.item_lane_geometry(rows, sheet, secs)
    assert sheet.penalty(0, 3) == S.MINOR, sheet.why[0]
    assert sheet.penalty(1, 3) == S.MINOR, sheet.why[1]


def test_같은구간의_두번째_차로유지_이탈은_중대로_심화한다():
    secs = S.Sections(straight(), 1)
    sheet = S.Sheet(1)
    rows = _lane_rows(
        lat_of=lambda _t: 1.6,
        bnd_of=lambda _t: [],
        lane_of=lambda _t: 1,
        lane_intrusion=lambda t: 0.2 if 2.5 <= t <= 3.0 or 7.0 <= t <= 7.5 else 0.0,
        edge=lambda t: 2.5 <= t <= 3.0 or 7.0 <= t <= 7.5)
    S.item_lane_geometry(rows, sheet, secs)
    assert sheet.penalty(0, 3) == S.MAJOR, sheet.why[0]


def test_차로유지_이탈이_구간경계를_걸치면_양쪽에_반영한다():
    route = straight(n=100)
    secs = S.Sections(route, 2)
    sheet = S.Sheet(2)
    rows = _lane_rows(
        lat_of=lambda _t: 1.6,
        bnd_of=lambda _t: [],
        lane_of=lambda _t: 1,
        n=200,
        lane_intrusion=lambda t: 0.2 if 4.8 <= t <= 5.4 else 0.0,
        edge=lambda t: 4.8 <= t <= 5.4)
    S.item_lane_geometry(rows, sheet, secs)
    assert sheet.penalty(0, 3) == S.MINOR, sheet.why[0]
    assert sheet.penalty(1, 3) == S.MINOR, sheet.why[1]


def test_중앙선_깊이_059는_미검출_060은_거리조건충족():
    assert _lane_penalty(_metric_rows("center_intrusion", 0.59, 0.6), 4) == 0
    assert _lane_penalty(_metric_rows("center_intrusion", 0.60, 0.6), 4) == S.MAJOR


def test_중앙선_06초_미만은_미검출_정확히_06초는_검출():
    assert _lane_penalty(_metric_rows("center_intrusion", 0.6, 0.59), 4) == 0
    assert _lane_penalty(_metric_rows("center_intrusion", 0.6, 0.60), 4) == S.MAJOR


def test_보도_깊이_049는_미검출_050은_거리조건충족():
    assert _lane_penalty(_metric_rows("sidewalk_intrusion", 0.49, 0.3), 5) == 0
    assert _lane_penalty(_metric_rows("sidewalk_intrusion", 0.50, 0.3), 5) == S.MAJOR


def test_보도_03초_미만은_미검출_정확히_03초는_검출():
    assert _lane_penalty(_metric_rows("sidewalk_intrusion", 0.5, 0.29), 5) == 0
    assert _lane_penalty(_metric_rows("sidewalk_intrusion", 0.5, 0.30), 5) == S.MAJOR


def test_CENTER_M과_WALK_M이_실제_채점경로의_깊이문턱이다(monkeypatch):
    monkeypatch.setattr(S, "CENTER_M", 0.61)
    monkeypatch.setattr(S, "WALK_M", 0.51)
    assert _lane_penalty(_metric_rows("center_intrusion", 0.60, 0.6), 4) == 0
    assert _lane_penalty(_metric_rows("sidewalk_intrusion", 0.50, 0.3), 5) == 0


# ---------------------------------------------------------------- ⑦ ⑨ 정지
def _tl_rows(n, state, v_fn, x0=0.0):
    """정지선(20,0) 을 향해 다가가는 프레임. `v_fn(i)` 로 속도를 준다."""
    out, x = [], x0
    for i in range(n):
        v = v_fn(i)
        x += v * 0.1
        out.append(dict(t=i * 0.1, x=x, y=0.0, h=0.0, v=v, tl=state, tid=7,
                        reason="RED_STOP", sig=0, clr=99.9, d_ego=0.0, objs=""))
    return out


def _stop_at_bumper_gap_then_cross(gap, state=1):
    """앞범퍼~정지선 `gap`에서 0.6초 정지한 뒤 정지선을 통과한다."""
    # 정지 좌표를 0으로 두어 `line - x - FRONT`가 경계값을 직접
    # 만들게 한다. 20m에서 역산하면 2.0이 1.9999999999999987이 된다.
    stop_line = S.FRONT + gap
    stop_x = 0.0

    def row(t, x, v):
        return dict(t=t, x=x, y=0.0, h=0.0, v=v, tl=state, tid=7,
                    reason="RED_STOP", sig=0, clr=99.9, d_ego=0.0,
                    objs="", cap_by="")

    rows = [row(0.0, stop_line - S.FRONT - 3.0, 3.0)]
    rows.extend(row(0.1 + i * 0.1, stop_x, 0.0) for i in range(7))
    rows.append(row(0.9, stop_line - S.FRONT + 1.1, 3.0))
    return rows, stop_line


def test_적신호_05초_못_서면_중대():
    #  정지선 20m. 앞범퍼 기준 2m 안에서 0.4초만 1km/h 이하 -> 안내문 미달
    rows = _tl_rows(120, 1, lambda i: 0.05 if 50 <= i < 54 else 3.0, x0=0.0)
    sheet, _ = run(rows, tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 7) == S.MAJOR, sheet.why[0]


def test_적신호_05초_서면_감점없음():
    #  x=15.0 에서 서면 앞범퍼~정지선 = 20 - 15 - 3.81 = 1.19m (2m 안)
    rows = _tl_rows(200, 1, lambda i: 0.05 if 50 <= i < 100 else 3.0, x0=0.0)
    sheet, _ = run(rows, tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 7) == 0, sheet.why[0]


def test_적색점멸은_항목9로_센다():
    rows = _tl_rows(120, 6, lambda i: 0.05 if 50 <= i < 54 else 3.0, x0=0.0)
    sheet, _ = run(rows, tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 9) == S.MAJOR and sheet.penalty(0, 7) == 0


def test_적신호_앞범퍼_1m_정지는_정상():
    rows, stop_line = _stop_at_bumper_gap_then_cross(1.0)
    sheet, _ = run(rows, tl_stops={"7": [stop_line, 0.0]})
    assert sheet.penalty(0, 7) == 0, sheet.why[0]


def test_적신호_앞범퍼_2m와_2m_초과_정지는_경미():
    for gap in (2.0, 2.01):
        rows, stop_line = _stop_at_bumper_gap_then_cross(gap)
        sheet, _ = run(rows, tl_stops={"7": [stop_line, 0.0]})
        assert sheet.penalty(0, 7) == S.MINOR, (gap, sheet.why[0])


def test_적신호_정지선_통과_후_정지는_정상으로_인정하지_않음():
    rows, stop_line = _stop_at_bumper_gap_then_cross(-0.01)
    sheet, _ = run(rows, tl_stops={"7": [stop_line, 0.0]})
    assert sheet.penalty(0, 7) == S.MAJOR, sheet.why[0]


def test_적색점멸도_2m_경계를_정상으로_인정하지_않음():
    rows, stop_line = _stop_at_bumper_gap_then_cross(2.0, state=6)
    sheet, _ = run(rows, tl_stops={"7": [stop_line, 0.0]})
    assert sheet.penalty(0, 9) == S.MINOR, sheet.why[0]


def test_정지선을_안_넘었으면_아직_위반이_아니다():
    rows = _tl_rows(60, 1, lambda i: 0.0, x0=0.0)     # 계속 서 있음
    sheet, _ = run(rows, tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 7) == 0


# ---------------------------------------------------------------- ⑧ 녹색 정차
def _green_idle(reason, cap_by="", objs=""):
    rows = []
    for i in range(400):                               # 16초
        rows.append(dict(t=i * 0.04, x=5.0, y=0.0, h=0.0, v=0.0, tl=3, tid=7,
                         reason=reason, sig=0, clr=99.9, d_ego=0.0, objs=objs,
                         cap_by=cap_by))
    return rows


def test_녹색_무의미_정차는_중대까지_간다():
    sheet, notes = run(_green_idle("LANE_KEEP"), tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 8) == S.MAJOR, (sheet.why[0], notes)


def test_법이_시키는_대기는_감점이_아니라_확인목록으로():
    """WAIT_LEFT_ARROW(좌회전 화살표 대기)는 객체가 아니라 신호 그 자체가 근거다 —
    2026-09-04 주최측 공식 답변이 확정한 건 "차량·보행자·장애물" 대응뿐이라 이런
    순수 신호 대기는 아직 이 범위 밖이고, 기존처럼 '판정 불명' 확인 목록에만 남는다."""
    sheet, notes = run(_green_idle("WAIT_LEFT_ARROW"), tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 8) == 0, sheet.why[0]
    assert notes and "판정 불명" in notes[0], notes


# ---- 2026-09-04 공식 답변: 차량·보행자·장애물 대응 정차는 객체 근거가 있으면
#   면책 확정 — 감점도 '판정 불명' 경고도 없어야 한다. `reason`/`cap_by` 문자열만
#   보고 믿지 않고 `objs` 로 실제 근거를 확인하므로, 각 사유에 맞는 위치의 객체를
#   같이 채운다.
def test_YIELD_PED_보행자_객체가_있으면_면책():
    o = "5.0:0.0:0.5:0.5:1.7:0.8:5.0:0.0:0.0:0"          # 전방 5m 사람 치수
    sheet, notes = run(_green_idle("YIELD_PED", objs=o), tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 8) == 0, sheet.why[0]
    assert not notes, notes


def test_JUNCTION_JAM_전방_차량이_있으면_면책():
    o = "10.0:0.0:4.5:1.8:1.5:0.0:5.0:0.0:3.0:0"         # 내 진로(dpath=0) 정지차
    sheet, notes = run(_green_idle("JUNCTION_JAM", objs=o), tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 8) == 0, sheet.why[0]
    assert not notes, notes


def test_FOLLOW_BRAKE_선행차가_있으면_면책():
    o = "8.0:0.0:4.5:1.8:1.5:0.0:5.0:0.0:3.0:0"
    sheet, notes = run(_green_idle("FOLLOW_BRAKE", objs=o), tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 8) == 0, sheet.why[0]
    assert not notes, notes


def test_OBSTACLE_STOP_정지_장애물이_있으면_면책():
    o = "10.0:0.0:1.0:1.0:0.6:0.0:5.0:0.0:0.0:0"         # 사람 치수가 아닌 정지 장애물
    sheet, notes = run(_green_idle("OBSTACLE_STOP", objs=o), tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 8) == 0, sheet.why[0]
    assert not notes, notes


def test_NARROW_BLOCK_막은_장애물이_있으면_면책():
    """옛 `LAWFUL_WAIT` 에 있어 무조건 면책이던 것 — 빼기만 하면 회귀다(위 AHEAD_SAFE
    선언부 주석 참고). 실제 장애물이 있을 때만 면책되는지 확인한다."""
    o = "6.0:0.0:1.5:1.2:1.0:0.0:5.0:0.0:0.0:0"
    sheet, notes = run(_green_idle("NARROW_BLOCK", objs=o), tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 8) == 0, sheet.why[0]
    assert not notes, notes


def test_YIELD_ONCOMING_대향차가_있으면_면책():
    """★실측 코스 E t=718.5~730.9(위 주석)와 같은 상황. 이제는 공식 답변으로 객체
    근거만 있으면 '판정 불명' 이 아니라 아예 면책이다."""
    o = "15.0:-3.0:4.5:1.8:1.5:8.0:10.0:0.0:0.0:170"     # 마주 오는 8m/s 차
    sheet, notes = run(_green_idle("YIELD_ONCOMING", objs=o), tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 8) == 0, sheet.why[0]
    assert not notes, notes


def test_LC_REAR_YIELD_후방_접근차가_있으면_면책():
    o = "-10.0:3.0:4.5:1.8:1.5:10.0:8.0:0.0:0.0:0"       # 뒤 10m, 10m/s 로 접근
    sheet, notes = run(_green_idle("LC_REAR_YIELD", objs=o), tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 8) == 0, sheet.why[0]
    assert not notes, notes


def test_LANE_KEEP_옆차로_무관객체는_면책되지_않는다():
    """reason 이 애초에 안전 사유가 아니면(LANE_KEEP), 객체가 뭐가 있든 면책하지
    않는다 — reason/cap_by 대조를 건너뛰고 객체 유무만으로 봐주면 안 된다."""
    o = "10.0:6.0:4.5:1.8:1.5:8.0:5.0:0.0:0.0:0"          # 옆 차로(fy=6.0), 무관한 차
    sheet, notes = run(_green_idle("LANE_KEEP", objs=o), tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 8) == S.MAJOR, sheet.why[0]


def test_안전reason만_있고_objs가_비어있으면_면책되지_않는다():
    """reason 문자열만 안전 사유고 뒷받침할 객체가 로그에 없으면(조작·유실 포함),
    공식 객체 면책으로 처리하지 않는다 — 단순 allowlist 로 되돌아가면 안 된다."""
    sheet, notes = run(_green_idle("YIELD_PED", objs=""), tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 8) == S.MAJOR, sheet.why[0]


def test_capby만_안전사유여도_실제객체가_있으면_면책():
    """추월·회피 중엔 `reason` 이 `OVT:*`/`*+N` 으로 덮이고 `cap_by` 만 실제로 이긴
    제약을 남긴다(drive.py:1613·1644) — 그래서 reason 뿐 아니라 cap_by 도 본다."""
    o = "10.0:0.0:4.5:1.8:1.5:0.0:5.0:0.0:3.0:0"
    sheet, notes = run(_green_idle("OVT:WAIT", cap_by="JUNCTION_JAM", objs=o),
                       tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 8) == 0, sheet.why[0]
    assert not notes, notes


def test_옛_로그처럼_cap_by_dpath_route_width가_없어도_동작한다():
    """`load()` 는 옛 CSV(컬럼 자체가 없음)에 빈 문자열을 기본값으로 쓰고,
    `parse_objs_detail` 은 `objs` 셀에 dpath·route_width·dh 세 칸이 아예 없어도
    (7필드짜리 옛 형식) 예외 없이 돈다."""
    rows = []
    for i in range(400):
        rows.append(dict(t=i * 0.04, x=5.0, y=0.0, h=0.0, v=0.0, tl=3, tid=7,
                         reason="OBSTACLE_STOP", sig=0, clr=99.9, d_ego=0.0,
                         objs="10.0:0.0:1.0:1.0:0.6:0.0:5.0"))   # cap_by 키 자체가 없다
    secs = S.Sections(straight(), 1)
    sheet = S.Sheet(1)
    notes = []
    S.item_traffic_light(rows, sheet, secs, {"7": [20.0, 0.0]}, notes)
    assert sheet.penalty(0, 8) == 0, sheet.why[0]
    assert not notes, notes


def test_load는_cap_by_컬럼이_없는_옛_CSV도_읽는다():
    """`cap_by` 컬럼이 아예 없던 옛 로그 형식 — `load()` 가 예외 없이 빈 문자열을
    채워야 다른 항목들처럼 항목 8 도 옛 판을 재채점할 수 있다."""
    header = "t,x,y,heading,v,tl_id,tl_state,reason,nobj,objs,d_ego\n"
    row = "0.04,5.0,0.0,0.0,0.0,7,3,OBSTACLE_STOP,1,10.0:0.0:1.0:1.0:0.6:0.0:5.0,0.0\n"
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as f:
        f.write(header + row)
        path = f.name
    try:
        rows = S.load(path)
    finally:
        os.remove(path)
    assert rows[0]["cap_by"] == ""
    assert rows[0]["objs"] == "10.0:0.0:1.0:1.0:0.6:0.0:5.0"


# ---------------------------------------------------------------- ⑩ 보행자
def _with_obj(v, fx, fy, ol, ow, sp):
    o = f"{fx}:{fy}:{ol}:{ow}:1.7:{sp}:5.0:0.0:0.0:0"
    return frames(30, v, objs=o)


def test_앞의_보행자를_안_서고_지나면_중대():
    sheet, _ = run(_with_obj(8.0, 5.0, 1.0, 2.0, 0.6, 1.2))
    assert sheet.penalty(0, 10) == S.MAJOR, sheet.why[0]


def test_옆_차로_자전거는_보행자가_아니다():
    """★실측 코스 E t=306.2: fx=-1.0 fy=2.5 로 **17km/h** 로 나란히 달리던 자전거를
    '보행자 3m 무정차 통과' 로 찍었다. VTD 는 사람과 자전거를 같은 상자로 준다 —
    가르는 건 **어디에 있고 얼마나 빠른가**다."""
    sheet, _ = run(_with_obj(8.0, -1.0, 2.5, 2.0, 0.6, 4.8))
    assert sheet.penalty(0, 10) == 0, sheet.why[0]


def test_보행자_앞에서_한번_섰으면_위반이_아니다():
    """★안내문은 "**무정차** 통과" 다 — 한 번도 안 섰나를 묻는다. 실측 코스 A t=368.0:
    YIELD_PED 로 0.7km/h 까지 떨어뜨렸는데 2.1km/h 이던 프레임 하나로 찍혔다."""
    o = "5.0:1.0:2.0:0.6:1.7:1.2:5.0:0.0:0.0:0"
    rows = frames(15, 0.1, objs=o)              # 먼저 완전히 선다
    rows += frames(20, 5.0, x0=rows[-1]["x"], objs=o)
    for k, r in enumerate(rows):
        r["t"] = k * 0.04
    sheet, _ = run(rows)
    assert sheet.penalty(0, 10) == 0, sheet.why[0]


def test_뒤에_있는_사람은_대상이_아니다():
    sheet, _ = run(_with_obj(8.0, -4.0, 1.0, 2.0, 0.6, 1.2))
    assert sheet.penalty(0, 10) == 0


def test_서_있었으면_보행자_위반이_아니다():
    sheet, _ = run(_with_obj(0.0, 5.0, 1.0, 2.0, 0.6, 1.2))
    assert sheet.penalty(0, 10) == 0


def test_차는_보행자로_안_센다():
    sheet, _ = run(_with_obj(8.0, 5.0, 1.0, 4.4, 1.8, 1.2))
    assert sheet.penalty(0, 10) == 0


# ---------------------------------------------------------------- ⑪ ⑭ 접촉
def test_사람과_접촉은_중대():
    o = "1.0:0.5:2.0:0.6:1.7:1.0:-0.10:0.0:0.0:0"
    sheet, _ = run(frames(10, 5.0, objs=o, clr=-0.10))
    assert sheet.penalty(0, 14) == S.MAJOR, sheet.why[0]


def test_낮은_정지물_접촉은_경미():
    o = "1.0:0.5:0.5:0.5:0.6:0.0:-0.10:0.0:0.0:0"
    sheet, _ = run(frames(10, 5.0, objs=o, clr=-0.10))
    assert sheet.penalty(0, 11) == S.MINOR and sheet.penalty(0, 14) == 0


# ---------------------------------------------------------------- ⑫ 횡단보도
def test_횡단보도_위_3초_정지는_경미():
    rows = []
    for i in range(200):                               # 8초
        rows.append(dict(t=i * 0.04, x=50.0, y=0.0, h=0.0, v=0.0, tl=0, tid=-1,
                         reason="LANE_KEEP", sig=0, clr=99.9, d_ego=0.0, objs=""))
    sheet, _ = run(rows, cws=[{"x": 51.9, "y": 0.0}])
    assert sheet.penalty(0, 12) == S.MINOR, sheet.why[0]


def test_횡단보도_앞에_서는_건_위반이_아니다():
    rows = []
    for i in range(200):
        rows.append(dict(t=i * 0.04, x=30.0, y=0.0, h=0.0, v=0.0, tl=0, tid=-1,
                         reason="CROSSWALK_STOP", sig=0, clr=99.9, d_ego=0.0, objs=""))
    sheet, _ = run(rows, cws=[{"x": 60.0, "y": 0.0}])
    assert sheet.penalty(0, 12) == 0


# ---------------------------------------------------------------- ⑬ 지시등
def _lane_change(sig_from_t):
    """t=5.0 부터 3초에 걸쳐 왼쪽으로 3.3m. `sig_from_t` 부터 좌측 점등."""
    rows = []
    for i in range(400):
        t = i * 0.04
        d = 0.0 if t < 5.0 else min(3.3, (t - 5.0) / 3.0 * 3.3)
        rows.append(dict(t=t, x=10.0 + 8.0 * t, y=0.0, h=0.0, v=8.0, tl=0, tid=-1,
                         reason="LANE_KEEP", sig=1 if t >= sig_from_t else 0,
                         clr=99.9, d_ego=d, objs=""))
    return rows


def test_차로변경_3초_전에_안_켜면_경미():
    sheet, _ = run(_lane_change(4.5))                  # 0.5초 전
    assert sheet.penalty(0, 13) == S.MINOR, sheet.why[0]


def test_3초_전에_켰으면_감점없음():
    sheet, _ = run(_lane_change(1.0))                  # 4초 전
    assert sheet.penalty(0, 13) == 0, sheet.why[0]


def test_출발_직후_경로수렴은_차로변경이_아니다():
    """★실측 코스 E: t=0 에 d_ego=3.20 -> t=4 에 -0.08. ego 는 차로 중앙이 아닌
    곳에 스폰되고 경로로 수렴한다. 그걸 '깜빡이 없는 차로변경' 으로 찍었다."""
    rows = []
    for i in range(300):
        t = i * 0.04
        rows.append(dict(t=t, x=10.0 + 8.0 * t, y=0.0, h=0.0, v=8.0, tl=0, tid=-1,
                         reason="LANE_KEEP", sig=0, clr=99.9,
                         d_ego=max(0.0, 3.2 - t), objs=""))
    sheet, _ = run(rows)
    assert sheet.penalty(0, 13) == 0, sheet.why[0]


# ---------------------------------------------------------------- ⑮ 리스폰
def test_리스폰은_구간당_1회_무료_그_뒤로는_누적():
    route = straight()
    secs = S.Sections(route, 1)
    rows = frames(300, 8.0)
    for idx in (100, 200):                             # 두 번 순간이동(그 뒤로 유지)
        for r in rows[idx:]:
            r["x"] += 50.0
    per = S.item_respawn(rows, S.Sheet(1), secs)
    assert len(per.get(0, [])) == 2, per


class _MonkeyPatch:
    """pytest 없이 `monkeypatch` 를 받는 테스트를 돌리기 위한 최소 대역.

    ⚠️ 이 저장소에서 pytest 는 **import 자체가 안 된다**(2026-09-09 실측:
       anyio 플러그인이 `_pytest.scope` 를 못 찾는다). 그래서 README 에 적힌
       `for f in tests/*.py; do python3 "$f"; done` 이 유일한 실행 경로인데,
       fixture 를 받는 테스트 하나가 `TypeError` 로 **파일 전체를 죽였다** —
       그 앞에서 통과한 결과까지 같이 버려지고, 화면에는 실패 건수도 안 남는다.
       테스트가 못 도는 것보다 나쁜 건 **안 돈 걸 모르는 것**이다.
    """

    def __init__(self):
        self._undo = []

    def setattr(self, obj, name, value):
        self._undo.append((obj, name, getattr(obj, name)))
        setattr(obj, name, value)

    def undo(self):
        for obj, name, old in reversed(self._undo):
            setattr(obj, name, old)
        self._undo.clear()


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_"):
            continue
        try:
            if "monkeypatch" in inspect.signature(fn).parameters:
                mp = _MonkeyPatch()
                try:
                    fn(mp)
                finally:
                    mp.undo()
            else:
                fn()
            print(f"  ✅ {name}")
        except AssertionError as e:
            fails += 1
            print(f"  ❌ {name}: {e}")
    print("실패 없음" if not fails else f"{fails}건 실패")
    sys.exit(1 if fails else 0)


def test_시작부터_정지선_뒤에_있던_신호는_통과로_안_센다():
    """실측 2026-09-06 코스 B t=0.0: 출발 위치가 신호147 정지선 94m 뒤인데 VTD 가 그 신호를
    보고했다. 다가간 적이 없는 선을 '정지 없이 통과' 로 셀 수는 없다(채점기 오탐 -6)."""
    rows = _tl_rows(60, 1, lambda i: 3.0, x0=30.0)      # 정지선(20,0) 을 이미 지난 자리에서 출발
    sheet, _ = run(rows, tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 7) == 0, sheet.why[0]


def test_다가간_뒤_넘은_것은_여전히_잡는다():
    rows = _tl_rows(120, 1, lambda i: 3.0, x0=0.0)
    sheet, _ = run(rows, tl_stops={"7": [20.0, 0.0]})
    assert sheet.penalty(0, 7) == S.MAJOR, sheet.why[0]
