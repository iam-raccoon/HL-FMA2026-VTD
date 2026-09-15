"""2026-09-09 6코스 스윕(A·B·D·E·G·H) 지적 — 행동층 셋.

E (917,211) 사고현장  8.4m 앞·우 2.1m 에 118° 로 선 차. SAT 는 종방향 거리 때문에 늘 '여유'라
                      10km/h 로 닿을 때까지 갔다(항목14 -6). "왜 얼굴부터 들이미냐".
E 굽은 구간           대향차(방위차 ~179°)를 직선 외삽하니 내 원호와 만나 TURN_CUT — 코너를 기어갔다.
A 신호100             황색에 0.7초 서고 적색 0.1초에 정지 '완료' -> 적색 정지로 인정 -> 바로 RIGHT_ON_RED.
                      채점 "적색에서 0.5초 정지" 미달로 -6.
"""
import math
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from behavior import Behavior                                          # noqa: E402
from vtd_io import Obj, State, TL_RED, TL_YELLOW                       # noqa: E402


def _ego(v=0.0, tl_state=0, tl_id=0, t=0.0):
    s = State(); s.speed = v; s.tl_state = tl_state; s.tl_id = tl_id; s.t = t
    s.heading = 0.0
    return s


def _arc_path(radius=25.0, v=8.0, horizon=4.0, step=0.25):
    out, prev, t = [], None, 0.0
    while t < horizon - 1e-9:
        t += step
        th = v * t / radius
        x, y = radius * math.sin(th), radius * (1.0 - math.cos(th))
        h = 0.0 if prev is None else math.atan2(y - prev[1], x - prev[0])
        out.append((t, x, y, h))
        prev = (x, y)
    return out


# ---------------------------------------------------------------- 비스듬한 정지차, 깊은 침범
def _sideways(fx=8.4, d=-2.13, deg=118.0, L=4.6, W=1.8):
    yaw = math.radians(deg)
    olen = L * abs(math.cos(yaw)) + W * abs(math.sin(yaw))     # 경로축 외접상자
    owid = L * abs(math.sin(yaw)) + W * abs(math.cos(yaw))
    return (fx, d, 0.0, olen, owid, 1.4, 7, W, L, yaw)


def test_경로를_깊이_막은_비스듬한_정지차는_미리_선다():
    """E 사고현장: 경로 폭 4.98m 가 우리 차로를 1.3m 침범. SAT 여유(2.7m)로 덮으면 닿을 때까지 간다."""
    b = Behavior()
    v0, _o, _t, r0 = b.plan(_ego(v=2.9), 8.33)
    v, _o, _t, r = b.plan(_ego(v=2.9), 8.33, rf_objs=[_sideways()])
    assert r == "NARROW_BLOCK", r
    # 뒷모서리(8.4-1.87=6.5m) 앞 body_stop_gap(1.5m) 에 서도록 감속 — 지금 속도보다 낮아야 한다
    assert v < 2.9, v
    reach_edge = 8.4 - _sideways()[3] / 2.0 - b.front_overhang
    assert abs(v - b._stop_target_speed(reach_edge - b.body_stop_gap)) < 1e-6, (v, reach_edge)


def test_살짝_걸치는_비스듬한_차는_예전대로_SAT_로_본다():
    """G 사고현장(2026-09-06)의 139° 차: 경로 폭 침범 0.26m 는 문턱(0.5m) 아래 -> 서지 않는다."""
    b = Behavior()
    yaw = math.radians(139.0)
    inflated_w = 4.4 * abs(math.sin(yaw)) + 1.8 * abs(math.cos(yaw))
    ob = (7.6, 2.8, 0.0, 4.4, inflated_w, 1.5, 9, 1.8, 4.4, yaw)
    v, _o, _t, r = b.plan(_ego(v=5.0), 8.33, rf_objs=[ob])
    assert r not in ("NARROW_BLOCK", "NARROW_PASS"), r
    assert 2.8 - b.half_width - inflated_w / 2.0 > -b.body_sweep_block


# ---------------------------------------------------------------- TURN_CUT 대향차 제외
def test_굽은_길의_대향차는_TURN_CUT_대상이_아니다():
    """왼쪽으로 도는 경로 + 반대 차로에서 마주 오는 차(방위차 179°). 직선 외삽하면 원호를 가로지른다."""
    b = Behavior()
    e = _ego(v=8.0)
    e.objects = [Obj(1, 30.0, 3.5, 0.0, math.radians(179.0), 10.0, 4.6, 1.8, 1.5)]
    assert b._turn_cut_conflict(e, path_ahead=_arc_path()) is None
    # 대조: 같은 자리의 **같은 방향** 차(옆 차로 추월차)는 그대로 잡힌다 — 안전망은 살아 있다
    e.objects = [Obj(1, -15.0, 3.6, 0.0, 0.0, 14.4, 4.57, 1.8, 1.5)]
    assert b._turn_cut_conflict(e, path_ahead=_arc_path()) is not None


# ---------------------------------------------------------------- 적색 우회전 정지: 적색이 통째로
def test_황색에_서다_적색_직후_정지가_끝나면_적색_정지가_아니다():
    """A 신호100: 황색 0.7초 + 적색 0.1초 = 정지 완료 순간만 적색. 적색 stop_hold_s 를 다시 채워야 한다."""
    b = Behavior()
    b.plan(_ego(v=8.33, tl_state=TL_YELLOW, tl_id=5), 8.33, tl_stop_dist=20.0, route_turn=-1)
    d = b.front_overhang + 1.0
    t = 1.0
    while t < 1.75:                                     # 황색에 0.7초 정지(stop_hold_s 0.8 미달)
        b.plan(_ego(v=0.0, tl_state=TL_YELLOW, tl_id=5, t=t), 8.33, tl_stop_dist=d, route_turn=-1)
        t += 0.1
    assert not b._rt_done
    _v, _o, _t, r = b.plan(_ego(v=0.0, tl_state=TL_RED, tl_id=5, t=1.8), 8.33,
                           tl_stop_dist=d, route_turn=-1)  # 이 프레임에 정지 완료(0.8초) — 적색은 0초
    assert b._rt_done and not b._rt_done_red, (b._rt_done, b._rt_done_red)
    assert r != "RIGHT_ON_RED", f"적색 0초에 출발한다 ({r})"
    seen = []
    for k in range(1, 12):                              # 적색에서 다시 채운다
        _v, _o, _t, r = b.plan(_ego(v=0.0, tl_state=TL_RED, tl_id=5, t=1.8 + 0.1 * k), 8.33,
                               tl_stop_dist=d, route_turn=-1)
        seen.append((round(1.8 + 0.1 * k, 1), r))
    assert seen[-1][1] == "RIGHT_ON_RED", seen
    first_go = next(tt for tt, rr in seen if rr == "RIGHT_ON_RED")
    assert first_go - 1.8 >= b.stop_hold_s - 1e-6, (first_go, seen)


def test_적색이_이미_stop_hold_s_이상_이어진_채_서면_바로_인정한다():
    b = Behavior()
    b.plan(_ego(v=8.33, tl_state=TL_RED, tl_id=5, t=0.0), 8.33, tl_stop_dist=20.0, route_turn=-1)
    d = b.front_overhang + 1.0
    r = None
    seen = []
    for k in range(0, 12):                              # 2.0~2.8 정지 채움 -> 2.9 완료 -> 3.0 출발
        _v, _o, _t, r = b.plan(_ego(v=0.0, tl_state=TL_RED, tl_id=5, t=2.0 + 0.1 * k), 8.33,
                               tl_stop_dist=d, route_turn=-1)
        seen.append(r)
    assert b._rt_done and b._rt_done_red
    assert seen[-1] == "RIGHT_ON_RED", seen
    assert seen[9] != "RIGHT_ON_RED" and seen[10] == "RIGHT_ON_RED", seen   # 완료 다음 프레임에 바로
