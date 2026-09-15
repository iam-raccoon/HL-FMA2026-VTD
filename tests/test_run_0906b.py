"""2026-09-06 낮 A·E·G 재주행(PR #10 반영 뒤)에서 나온 것들.

E 신호114: 적신호 대기 17초가 '갇힘'으로 세어져 녹색 직후 절박 모드로 오른쪽(여유 1.36m = 인도)
          추월 -> 리스폰. "아무리 앞차가 멈춰도 인도로 추월은 아니지... 신호 기다리는 건데"
G 신호198: 녹색 서행 우회전이 정지선을 지나 신호 보고가 끊기자 무신호 우회전으로 보고 급정지.
G 사고현장: 139° 로 선 옆차로 차의 경로축 상자가 4.3m 로 부풀어 33초 정지 -> 절박 추월 -> 리스폰.
G t=237: 추월 시작과 동시에 지시등 -> 선행 1.8초 < 3초(항목13 -3).
G 리스폰 뒤: 전역 재정위가 같은 길의 첫 지나감(idx 104)을 집어 그쪽 프로파일 26km/h 로 기었다.
G 신호137: DB 정지점(기둥 아래)이 진입부 도색 정지선보다 28m 뒤 — 28km/h 로 지났다.
"""
import math
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from behavior import Behavior                                          # noqa: E402
from overtake import Overtaker, TS_OFF                                 # noqa: E402
from drive import DrivingStack, straight_route                         # noqa: E402
from vtd_io import State, TL_GREEN, TL_UNSET                           # noqa: E402


def _ego(v=0.0, tl_state=TL_UNSET, tl_id=0, t=0.0, x=0.0, y=0.0, heading=0.0):
    s = State(); s.speed = v; s.tl_state = tl_state; s.tl_id = tl_id; s.t = t
    s.x, s.y, s.heading = x, y, heading
    return s


# ---------------------------------------------------------------- 녹색 우회전 래치
def test_녹색으로_서행한_우회전은_신호가_끊겨도_다시_서지_않는다():
    b = Behavior()
    _v, _o, _t, r = b.plan(_ego(v=4.2, tl_state=TL_GREEN, tl_id=198), 8.33,
                           tl_stop_dist=10.0, route_turn=-1)
    assert r == "RIGHT_TURN_SLOW", r
    # 정지선을 지나 tl_id=0, 정지점은 교차로(junc_dist) — 예전엔 여기서 RIGHT_TURN_STOP
    _v, _o, _t, r = b.plan(_ego(v=4.2, tl_state=TL_UNSET, tl_id=0), 8.33,
                           junc_dist=5.0, route_turn=-1)
    assert r != "RIGHT_TURN_STOP", f"정지선 지나서 또 선다 ({r})"
    assert not b._rt_armed


def test_새_신호가_보고되면_래치가_풀린다():
    b = Behavior()
    b.plan(_ego(v=4.2, tl_state=TL_GREEN, tl_id=198), 8.33, tl_stop_dist=10.0, route_turn=-1)
    b.plan(_ego(v=8.33, tl_state=TL_UNSET, tl_id=0), 8.33, junc_dist=60.0, route_turn=0)   # 회전 뒤
    _v, _o, _t, r = b.plan(_ego(v=8.33, tl_state=TL_UNSET, tl_id=0), 8.33,
                           junc_dist=20.0, route_turn=-1)      # 다음 무신호 우회전
    assert r == "RIGHT_TURN_STOP", r


# ---------------------------------------------------------------- 절박 모드 폭
def _plan(l, r, xl=0.0, xr=0.0):
    return {"l": l, "r": r, "xl": xl, "xr": xr, "need": 0.0, "w": 3.3}


def test_절박해도_차_한_대_폭이_없는_쪽으로는_안_나간다():
    st = DrivingStack(route=straight_route(200.0))
    # 왼쪽은 막혀서 제외(exclude=+1), 오른쭉 여유 1.36m = 인도 — 예전엔 여기로 나갔다
    assert st._side_for(_plan(4.36, 1.36), 3.5, desperate=True, exclude=+1.0) == 0.0
    # ★2026-09-08: 같은 방향 차도 안에서 **차 한 대 폭(2.0m)** 이 있으면 나간다.
    #   주최측 답변은 **중앙선**에 예외가 없다는 것이지 자기 차도 안 회피를 금지한 게 아니다.
    #   이것까지 막으니 오프라인 회귀 4판이 미완주로 떨어졌다(static_obstacle · v3_leadbrake ·
    #   hz_blocker_leaves · hz_wrongway). 미완주는 0점이다.
    assert st._side_for(_plan(4.36, 2.4), 3.5, desperate=True, exclude=+1.0) == -1.0
    # 종전의 물리적 폭 우선은 명시적 비정상/실험 옵션에서만 복원한다.
    experimental = DrivingStack(route=straight_route(200.0), allow_centerline_escape=True)
    assert experimental._side_for(
        _plan(4.36, 2.4), 3.5, desperate=True, exclude=+1.0) == -1.0


# ---------------------------------------------------------------- 추월 지시등 3초 선행
def _stopped_car(ds):
    return (7, ds, 0.0, 0.0, 4.4, 1.8, 1.5)


def test_추월은_지시등을_켠_뒤_PASS_SIG_LEAD_가_지나야_나간다():
    o = Overtaker()
    e = State(); e.speed = 0.0
    t = 100.0
    o.plan(e, [_stopped_car(15.0)], now=t)               # FOLLOW -> WAIT (나갈 조건은 이미 충족)
    assert o.state == "WAIT"
    _off, turn, st, _b = o.plan(e, [_stopped_car(15.0)], now=t + 0.5)
    assert turn != TS_OFF, "나갈 조건이면 지시등은 바로 켠다"
    assert st == "WAIT", "켠 직후에 바로 나가면 선행 0초다"
    # 기산점은 WAIT 안에서 조건이 처음 확인된 프레임(t+0.5)이다
    o.plan(e, [_stopped_car(15.0)], now=t + 0.5 + o.PASS_SIG_LEAD - 0.1)
    assert o.state == "WAIT"
    o.plan(e, [_stopped_car(15.0)], now=t + 0.5 + o.PASS_SIG_LEAD + 0.05)
    assert o.state == "PASS", o.state


# ---------------------------------------------------------------- 리스폰 재정위
def test_리스폰_뒤에도_직전_인덱스_창에서_다시_잡는다():
    st = DrivingStack(route=straight_route(300.0))
    st._bi_prev = 120
    st.reset(now=1.0)
    assert st._bi_prev == 120, "리스폰이 인덱스를 지우면 겹치는 경로에서 엉뚱한 구간으로 튄다"


# ---------------------------------------------------------------- 앞선 도색 정지선
def test_DB_정지점_앞_도색선이_있으면_그게_정지선이다():
    """신호 137: DB 점 (30,0), 진입부 도색선 (2,0) — 28m 앞."""
    st = DrivingStack(route=straight_route(200.0),
                      tl_stops_extra={137: [30.0, 0.0]},
                      stoplines_all=[[2.0, 0.0, 0.0]])
    st.sc = True                                          # _tl_stop_dist 는 시나리오가 있을 때만
    s = _ego(v=8.0, tl_id=137, x=-10.0, y=0.0, heading=0.0)
    d, passed = st._tl_stop_dist(s)
    assert passed is False and abs(d - 12.0) < 1e-6, (d, passed)    # DB 점(40m)이 아니라 도색선(12m)
    # 도색선을 지나 DB 점 사이에 있으면 DB 점을 본다(교차로 안에서 새로 서지 않게 하는 건 다른 규칙)
    s2 = _ego(v=8.0, tl_id=137, x=5.0, y=0.0, heading=0.0)
    d2, passed2 = st._tl_stop_dist(s2)
    assert passed2 is False and abs(d2 - 25.0) < 1e-6, (d2, passed2)


def test_DB_정지점_바로_위_도색선은_앞선_선이_아니다():
    st = DrivingStack(route=straight_route(200.0),
                      tl_stops_extra={7: [30.0, 0.0]},
                      stoplines_all=[[29.0, 0.0, 0.0]])
    st.sc = True
    d, _p = st._tl_stop_dist(_ego(tl_id=7, x=0.0))
    assert abs(d - 30.0) < 1e-6, d


def test_너무_먼_도색선은_이_신호_것이_아니다():
    st = DrivingStack(route=straight_route(200.0),
                      tl_stops_extra={7: [80.0, 0.0]},
                      stoplines_all=[[20.0, 0.0, 0.0]])                 # 60m 앞 — 다른 데 것
    st.sc = True
    d, _p = st._tl_stop_dist(_ego(tl_id=7, x=0.0))
    assert abs(d - 80.0) < 1e-6, d


# ---------------------------------------------------------------- 비스듬한 정지차 SAT
def test_옆차로에_비스듬히_선_차는_통로를_막지_않는다():
    """G 사고현장: 7.6m 앞 좌 2.8m, 139° 로 선 4.4x1.8 차. 경로축 상자 폭 4.3m 로 부풀어 막혔다."""
    b = Behavior()
    yaw = math.radians(139.0)
    inflated_w = 4.4 * abs(math.sin(yaw)) + 1.8 * abs(math.cos(yaw))    # 경로축 외접상자 폭
    ob = (7.6, 2.8, 0.0, 4.4, inflated_w, 1.5, 9, 1.8, 4.4, yaw)         # ..., id, raw_w, raw_l, hdg
    v0, _o, _t, _r = b.plan(_ego(v=5.0), 8.33)
    v, _o, _t, r = b.plan(_ego(v=5.0), 8.33, rf_objs=[ob])
    assert r not in ("NARROW_BLOCK", "NARROW_PASS"), r
    assert v >= 2.9, (v, r)          # 서지 않는다 — 옆 차로 정지차 옆 서행(ADJ_SLOW 10.8km/h)까지는 허용
    # 같은 상자를 **정렬된** 차로 주면(부풀림이 실제 폭) 예전처럼 막는다 — 안전망은 그대로
    ob2 = (7.6, 2.8, 0.0, 4.4, inflated_w, 1.5, 9, inflated_w, 4.4, 0.0)
    b2 = Behavior()
    v2, _o, _t, r2 = b2.plan(_ego(v=5.0), 8.33, rf_objs=[ob2])
    assert v2 < v0, (v2, v0)


# ---------------------------------------------------------------- 목표차선 먼 정지차
def test_추월_구간_너머에_서_있는_차는_목표차선을_막지_않는다():
    """회귀 hz_blocker_leaves: 막는 차 15m 앞, 목표차선(W=+3.5) 에 50m 더 앞 주차 차 → 나가야 한다."""
    o = Overtaker(lane_width=3.5, side=+1.0)
    far = (9, 65.0, 3.5, 0.0, 4.4, 1.8, 1.4)               # 막는 차(15m) 보다 50m 앞, 서 있음
    near = (9, 25.0, 3.5, 0.0, 4.4, 1.8, 1.4)              # 10m 앞 — 추월하고 돌아올 자리
    assert o._target_lane_clear([far], 0.0, 0.0, block_fx=15.0) is True
    assert o._target_lane_clear([near], 0.0, 0.0, block_fx=15.0) is False
    assert o._target_lane_clear([far], 0.0, 0.0) is False      # 막는 차를 모르면 예전처럼 보수적
    moving = (9, 65.0, 3.5, 8.0, 4.4, 1.8, 1.4)
    assert o._target_lane_clear([moving], 0.0, 0.0, block_fx=15.0) is False   # 움직이는 차는 그대로 막는다


# ---------------------------------------------------------------- 3차 재주행(2026-09-06 낮 2)
from vtd_io import Obj, TL_YELLOW                                       # noqa: E402


def test_경로_옆에_투영된_먼_정지물은_사람이_아니다():
    """G 미완주(248초): 오른쭉 62m 의 0.2x0.4x1.4 물체가 경로가 77m 뒤에 그 옆을 지나 rf 횡 1.6m 로
    투영됐고, 정지거리는 직선 9.9m 를 써 '코앞 6m 의 사람' 이 됐다. 직선 횡거리가 멀면 직선 전방거리를 쓰지 않는다."""
    b = Behavior()
    e = _ego(v=5.0)
    e.objects = [Obj(5, 9.9, -61.7, 0.0, 0.0, 0.0, 0.2, 0.4, 1.4)]     # 세계=차체 기준(원점, heading 0)
    rf = [(77.0, 1.6, 0.0, 0.2, 0.4, 1.4, 5, 0.4, 0.2, 0.0)]           # 경로 기준: 77m 앞, 옆 1.6m
    v0, _o, _t, _r = b.plan(_ego(v=5.0), 8.33)
    v, _o, _t, r = b.plan(e, 8.33, rf_objs=rf)
    assert v >= v0 - 1e-6, (v, v0, r)                 # 라벨은 붙을 수 있어도 속도를 깎지 않는다(77m 앞)
    # 정말 가까운 사람(직선으로도 옆 1.6m, 앞 9.9m)은 여전히 선다
    e2 = _ego(v=5.0); e2.objects = [Obj(6, 9.9, 1.6, 0.0, 0.0, 0.0, 2.0, 0.6, 1.7)]
    rf2 = [(12.0, 1.6, 0.0, 2.0, 0.6, 1.7, 6, 0.6, 2.0, 0.0)]
    v2, _o, _t, r2 = Behavior().plan(e2, 8.33, rf_objs=rf2)
    assert r2 == "YIELD_PED", r2


def test_황색에_서기로_했으면_반응시간_없이_계속_선다():
    """E 신호121 -6: YELLOW_STOP 으로 제동 중 정지선 4.5m 앞 14km/h 에서 d_brake(반응 0.8초 포함) 4.7m
    > 4.5m 라 YELLOW_GO 로 뒤집혀 재가속, 적색 순간 범퍼가 선 위. 이미 제동 중이면 제동거리만 본다."""
    b = Behavior()
    _v, _o, _t, r = b.plan(_ego(v=8.0, tl_state=TL_YELLOW, tl_id=121), 8.33, tl_stop_dist=25.0)
    assert r == "YELLOW_STOP", r
    _v, _o, _t, r = b.plan(_ego(v=3.83, tl_state=TL_YELLOW, tl_id=121), 8.33, tl_stop_dist=4.5 + 3.81)
    assert r == "YELLOW_STOP", f"이미 서던 차가 뒤집혔다 ({r})"
    # 같은 상태를 **처음** 보는 차는 반응시간 포함 판정 -> 딜레마존(예전 동작 그대로)
    _v, _o, _t, r0 = Behavior().plan(_ego(v=3.83, tl_state=TL_YELLOW, tl_id=121), 8.33, tl_stop_dist=4.5 + 3.81)
    assert r0 == "YELLOW_GO", r0


def _po_stack(rs, r_goal, w=3.1):
    """직선 200m 경로, 마지막 len(rs) m 의 차로계획 r 값을 준다(경로가 왼쪽으로 옮기면 r 가 커진다)."""
    n = 200
    lp = [{"lane": -1, "w": w, "l": 1.5, "r": 1.5, "need": 0.0, "sig": 0, "j": 0, "jx": 0, "lim": 13.889} for _ in range(n)]
    for k, rr in enumerate(rs):
        lp[n - len(rs) + k] = dict(lp[0], r=rr, lane=-1)
    lp[-1]["r"] = r_goal
    return DrivingStack(route=straight_route(200.0), lane_plan=lp), lp


def test_종점_붙기는_도로_기준으로_고정된다():
    """A 종점: 경로가 -3 -> -2 -> -1 로 두 차로 왼쪽 합류(r 1.5 -> 4.6 -> 7.7), 종점 오른쪽엔 한 차로.
    차는 오른쪽 끝에서 **1.5m** 를 지켜야 한다(경로 따라 왼쪽으로 끌려가면 안 된다)."""
    rs = [1.5] * 20 + [1.5 + 3.1 * (k / 30.0) for k in range(30)] + [4.6] * 10 + [4.6 + 3.1 * (k / 20.0) for k in range(20)] + [7.7] * 10
    st, lp = _po_stack(rs, r_goal=7.7)
    st._po_goal_plan = lp[-1]
    off, d_ego = 0.0, 0.0
    edges = []
    for k in range(len(rs)):
        bi = 200 - len(rs) + k
        s_remain = float(len(rs) - k)
        off = st._pullover_offset(lp[bi], s_remain, [], d_ego, 0.125, 8.0, bi=bi)   # 1m/프레임 = 8m/s
        d_ego = off                                             # 차가 오프셋을 그대로 따른다고 가정
        edges.append(lp[bi]["r"] + off)
    assert st._po_edge is not None and abs(st._po_edge - 1.5) < 0.2, st._po_edge          # 지금 있는 맨 오른쪽 차로를 지킨다
    assert all(abs(e - st._po_edge) < 0.6 for e in edges[25:]), edges[25::10]            # 도로 기준으로 붙어 있다
    assert st._po_net_left in (0, -1)                                                    # 왼쪽으로 갈 일이 없다


def test_서_있는_사람이_차로_가장자리에_있으면_서행_통과한다():
    # G (1348,103): 경로 옆 1.6m 에 선 0.2x0.4x1.4 — 실여유 0.46m. 예전엔 YIELD_PED 영구 정지.
    b = Behavior()
    e = _ego(v=3.0); e.objects = [Obj(7, 9.9, 1.6, 0.0, 0.0, 0.0, 0.2, 0.4, 1.4)]
    rf = [(9.9, 1.6, 0.0, 0.2, 0.4, 1.4, 7, 0.4, 0.2, 0.0)]
    v, _o, _t, r = b.plan(e, 8.33, rf_objs=rf)
    assert r == "PED_STILL_ASIDE", r
    assert 1.0 < v <= b.ped_edge_pass_v + 1e-6, v
    # 차로 안(옆 0.8m)에 선 사람은 여전히 선다
    e2 = _ego(v=3.0); e2.objects = [Obj(8, 9.9, 0.8, 0.0, 0.0, 0.0, 0.2, 0.4, 1.4)]
    rf2 = [(9.9, 0.8, 0.0, 0.2, 0.4, 1.4, 8, 0.4, 0.2, 0.0)]
    v2, _o, _t, r2 = Behavior().plan(e2, 8.33, rf_objs=rf2)
    assert r2 in ("YIELD_PED", "NARROW_BLOCK"), r2       # 어느 규칙이 라벨을 갖든 **정지 규칙**이 잡는다(9.9m 앞이라 아직 감속 중)
