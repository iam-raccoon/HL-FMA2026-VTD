"""비켜 갈 때 **뒤·옆을 본다** + 횡단보도 위 사람 앞에서는 정지선에 선다 (2026-09-10 코스 H).

실측 두 가지(주변교통+보행자 판 `HL_FMA_NEW_H_TRV`):
  ① (956,653) 신호151: 적신호에 섰다가 녹색이 되자 **횡단보도 한복판에 서 있는 사람
     옆 2.3m 를 7.7km/h 로 기어 지나갔다**(PED_STILL_ASIDE). 사용자: "사람이 횡단보도
     건널 때는 정지선에서 대기하고".
  ② (966,657): 그 사람을 피해 우측으로 3.0m 나가는 동안, 오른쪽 뒤 10.4m 에서 6.8m/s 로
     오던 차가 옆에 붙어 **실여유 -0.24m 접촉**. 사용자: "차선 바꿀 때 뒷차를 봐야지
     걍 들이밀어서 사고남". `_lat_free` 는 뒤 3m 까지만 본다 — 그게 구멍이었다.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from drive import DrivingStack, straight_route                         # noqa: E402


def _row(ds, d, spd, olen=4.4, owid=1.8, hgt=1.5, oid=1):
    """rf_objs 한 줄: (ds, d, speed, olen, owid, hgt, id, raw_w, raw_l, dh)."""
    return (ds, d, spd, olen, owid, hgt, oid, owid, olen, 0.0)


def _ped(ds, d, spd=0.0, oid=9):
    return (ds, d, spd, 2.0, 0.6, 1.7, oid, 0.6, 2.0, 0.0)


def _stack(**kw):
    return DrivingStack(route=straight_route(300.0), **kw)


# ---------------------------------------------------------------- 뒤·옆 확인
def test_뒤에서_오는_차가_있으면_그쪽으로_안_비킨다():
    st = _stack()
    rear = _row(-10.4, -3.0, 6.8)                       # 코스 H 실측: 우측 뒤 10.4m, 6.8m/s
    assert not st._nudge_rear_clear(-3.0, [rear], d_ego=0.0, ego_speed=2.1)


def test_나보다_충분히_느린_뒤차는_막지_않는다():
    st = _stack()
    slow = _row(-10.0, -3.0, 1.0)
    assert st._nudge_rear_clear(-3.0, [slow], d_ego=0.0, ego_speed=8.0)   # 7m/s 느리다


def test_이미_옆에_붙은_차는_속도와_무관하게_막는다():
    st = _stack()
    beside = _row(-1.0, -3.0, 0.0)
    assert not st._nudge_rear_clear(-3.0, [beside], d_ego=0.0, ego_speed=5.0)


def test_다른_차로의_뒤차는_상관없다():
    st = _stack()
    far = _row(-10.0, -6.5, 9.0)                        # 두 차로 건너
    assert st._nudge_rear_clear(-3.0, [far], d_ego=0.0, ego_speed=2.0)


def test_사람과_작은_물건은_이_검사의_대상이_아니다():
    st = _stack()
    assert st._nudge_rear_clear(-3.0, [_ped(-8.0, -3.0, 1.2)], d_ego=0.0, ego_speed=2.0)
    cone = _row(-8.0, -3.0, 0.0, olen=0.4, owid=0.4, hgt=0.6)
    assert st._nudge_rear_clear(-3.0, [cone], d_ego=0.0, ego_speed=2.0)


def test_거의_안_비키는_것은_검사하지_않는다():
    st = _stack()
    rear = _row(-10.0, -0.2, 9.0)
    assert st._nudge_rear_clear(-0.2, [rear], d_ego=0.0, ego_speed=2.0)


def test_비킬_수_없으면_지금_자리를_유지한다():
    """0 으로 되돌리지 않는다 — 반쯤 나간 채 급히 돌아오는 것도 위험하다."""
    plan = {"lane": -1, "w": 3.0, "l": 1.5, "r": 4.5, "xl": 0.0, "xr": 0.0, "need": 0.0,
            "sig": 0, "j": 0, "jx": 0, "lim": 13.889}
    st = _stack()
    objs = [_ped(12.0, 1.0), _row(-10.4, -3.0, 6.8, oid=2)]
    st._nudge = -1.5                                     # 이미 우측으로 1.5m 나가 있다
    got = st._nudge_offset(objs, d_ego=0.0, dt=0.1, plan=plan, ped_bypass=True, ego_speed=2.1)
    assert abs(got - (-1.5)) < 1e-6, got                 # 더 들어가지도, 돌아오지도 않는다
    st2 = _stack()
    st2._nudge = -1.5
    free = st2._nudge_offset([objs[0]], d_ego=0.0, dt=0.1, plan=plan,
                             ped_bypass=True, ego_speed=2.1)
    assert free < -1.5                                   # 뒤가 비면 계속 나간다


# ---------------------------------------------------------------- 횡단보도 위 사람
def test_횡단보도_위_사람만_센다():
    st = _stack()
    objs = [_ped(30.0, 2.3), _ped(30.0, 8.0), _ped(50.0, 0.0), _row(30.0, 0.0, 0.0)]
    on = st._ped_on_crosswalk(objs, cw_fwd=32.0, d_ego=0.0)
    assert on == [True], on                              # 옆 8m·앞 20m·차는 제외(2.3m 는 채점 3m 안)
    assert st._ped_on_crosswalk(objs, cw_fwd=100.0, d_ego=0.0) == []


def test_걷는_사람과_서_있는_사람을_구분해_돌려준다():
    st = _stack()
    on = st._ped_on_crosswalk([_ped(30.0, 1.0, spd=1.3), _ped(31.0, -1.0, spd=0.0)],
                              cw_fwd=30.0, d_ego=0.0)
    assert sorted(on) == [False, True]                   # 걷는 중 · 서 있음


# ---------------------------------------------------------------- 신호에 서 있는 동안엔 안 민다
def test_적신호에_서_있는_동안에는_오프셋을_안_바꾼다():
    """실측 2026-09-10 코스 H (1331,203): 적신호 정지 중 오프셋이 -3.16 -> -0.90 으로 되돌아오며
    **조향이 ±35°로 돌고 지시등이 좌우로 뒤집혔다**(차는 1cm 도 안 움직임). 사용자: "에바임"."""
    plan = {"lane": -1, "w": 3.0, "l": 1.5, "r": 4.5, "xl": 0.0, "xr": 0.0, "need": 0.0,
            "sig": 0, "j": 0, "jx": 0, "lim": 13.889}
    st = _stack()
    st._nudge = -3.16
    same = st._nudge_offset([_ped(10.0, 0.0)], d_ego=0.0, dt=0.1, plan=plan,
                            ped_bypass=True, ego_speed=0.0, hold_lat=True)
    assert same == -3.16                                   # 한 프레임도 안 움직인다
    moved = st._nudge_offset([_ped(10.0, 0.0)], d_ego=0.0, dt=0.1, plan=plan,
                             ped_bypass=True, ego_speed=2.0, hold_lat=False)
    assert moved != -3.16                                  # 움직이기 시작하면 다시 민다


# ---------------------------------------------------------------- 사고현장 탈출은 차로가 아니라 폭으로 민다
def test_탈출_폭은_차로_판정을_건너뛰고_그대로_민다():
    """`force` 는 물리 통로로 나가는 값이라 `_side_for`·`_lat_free` 의 차로 판정을 안 받는다.
    실측 2026-09-10 코스 H (1462,929): 합법 폭이 1.5m 뿐이라 차로 기반 nudge 는 0 을 냈고,
    추월기는 사람을 추월 대상에서 빼서 FOLLOW 에 머물렀다 — 아무도 안 움직여 249초 교착."""
    plan = {"lane": -1, "w": 3.0, "l": 1.5, "r": 1.5, "xl": 0.0, "xr": 0.0, "need": 0.0,
            "sig": 0, "j": 0, "jx": 0, "lim": 13.889, "pl": 1.5, "pr": 4.3}
    objs = [_ped(8.9, 0.0)]
    st = _stack()
    assert st._nudge_offset(objs, d_ego=0.0, dt=0.1, plan=plan, ego_speed=0.0) == 0.0
    st2 = _stack()
    got = st2._nudge_offset(objs, d_ego=0.0, dt=0.1, plan=plan, ego_speed=0.0, force=-2.24)
    assert -0.13 < got < 0.0                               # 변화율 제한 안에서 우측으로 시작
    for _ in range(40):
        got = st2._nudge_offset(objs, d_ego=0.0, dt=0.1, plan=plan, ego_speed=0.0, force=-2.24)
    assert abs(got - (-2.24)) < 1e-6                       # 목표까지 간다


def test_탈출_폭도_뒤차가_있으면_안_민다():
    plan = {"lane": -1, "w": 3.0, "l": 1.5, "r": 1.5, "xl": 0.0, "xr": 0.0, "need": 0.0,
            "sig": 0, "j": 0, "jx": 0, "lim": 13.889, "pl": 1.5, "pr": 4.3}
    st = _stack()
    objs = [_ped(8.9, 0.0), _row(-12.0, -2.3, 7.0, oid=2)]
    for _ in range(40):
        got = st._nudge_offset(objs, d_ego=0.0, dt=0.1, plan=plan, ego_speed=1.0, force=-2.24)
    assert got == 0.0                                      # 뒤차가 지나갈 때까지 안 나간다


def test_횡단보도로_걸어오는_사람은_9m_까지_센다():
    """옆 5m 로만 보면 인도에서 걸어오는 사람이 5m 안에 들어왔을 때 우리는 이미 정지선 코앞이라
    선을 넘어 선다(실측 코스 H: 1.8m 초과). 걷는 중 + 가로지르는 방향이면 9m 까지 본다."""
    st = _stack()
    coming = (30.0, 6.9, 1.3, 2.0, 0.6, 1.7, 9, 0.6, 2.0, math.radians(-95.0))
    assert st._ped_on_crosswalk([coming], cw_fwd=31.0, d_ego=0.0) == [False]   # 왼쪽에서 내 쪽으로
    going = (30.0, 6.9, 1.3, 2.0, 0.6, 1.7, 9, 0.6, 2.0, math.radians(95.0))
    assert st._ped_on_crosswalk([going], cw_fwd=31.0, d_ego=0.0) == []         # 이미 건너 멀어지는 중
    standing = (30.0, 6.9, 0.0, 2.0, 0.6, 1.7, 9, 0.6, 2.0, math.radians(-95.0))
    assert st._ped_on_crosswalk([standing], cw_fwd=31.0, d_ego=0.0) == []      # 서 있으면 3m 기준
    along = (30.0, 6.9, 1.3, 2.0, 0.6, 1.7, 9, 0.6, 2.0, 0.0)
    assert st._ped_on_crosswalk([along], cw_fwd=31.0, d_ego=0.0) == []         # 나란히 걷는 사람
    far = (30.0, 12.0, 1.3, 2.0, 0.6, 1.7, 9, 0.6, 2.0, math.radians(-95.0))
    assert st._ped_on_crosswalk([far], cw_fwd=31.0, d_ego=0.0) == []           # 9m 밖


def test_뒤에_서_있는_정체는_비키기를_막지_않는다():
    """실측 2026-09-10 코스 H (1462,929) 135초 교착: 내가 멈춰 있으면 `spd > ego_speed - 1.0`
    이 `0 > -1.0` 이라 **내 뒤에 선 정체 차량이 전부 '따라붙는 차'** 가 됐다. 뒤 6.4m 의 정지차
    하나가 사고현장 탈출을 영영 막았다. 사용자: "왜 안 되냐"."""
    st = _stack()
    queued = _row(-6.4, -0.35, 0.0, olen=4.2, owid=2.04)     # 앞끝 -4.3 = 내 뒷범퍼(-1.04) 뒤
    assert st._nudge_rear_clear(-2.49, [queued], d_ego=0.09, ego_speed=0.0)


def test_몸이_겹치는_정지차는_서_있어도_막는다():
    """완전히 뒤에 선 차와 달리, 앞끝이 내 뒷범퍼를 넘어와 있으면 옆으로 가는 순간 긁는다."""
    st = _stack()
    overlap = _row(-1.0, -3.0, 0.0, olen=4.4, owid=1.8)      # 앞끝 +1.2
    assert not st._nudge_rear_clear(-3.0, [overlap], d_ego=0.0, ego_speed=5.0)


# ---------------------------------------------------------------- 신호 있는 횡단보도도 본다
def _h_stack():
    import json
    here = os.path.dirname(os.path.abspath(__file__))
    R = os.path.join(here, "..", "routes")
    rt = json.load(open(os.path.join(R, "HL_FMA_NEW_H.json"), encoding="utf-8"))["ego_route"]
    cw = json.load(open(os.path.join(R, "crosswalks.json"), encoding="utf-8"))["crosswalks"]
    return DrivingStack(route=rt, crosswalks=cw), rt


def _h_state(rt, x, y):
    from vtd_io import State
    s = State(); s.x, s.y = x, y
    i = min(range(len(rt)), key=lambda k: (rt[k][0] - x) ** 2 + (rt[k][1] - y) ** 2)
    s.heading = math.atan2(rt[i + 3][1] - rt[i - 3][1], rt[i + 3][0] - rt[i - 3][0])
    s.speed, s.t = 0.0, 27.4
    return s


def test_신호_있는_횡단보도_위의_사람도_세운다():
    """실측 2026-09-10 코스 H (960,655): 신호151 녹색이 되자 횡단보도 한복판에 선 사람 옆
    2.3m 를 7.7km/h 로 지나갔다. `self.cw` 는 제27조⑦(보호구역 무신호)용이라 **신호 있는
    횡단보도가 통째로 빠져 있었다** — 이 맵은 195곳 중 121곳이 신호다.
    제27조① 은 신호와 무관하다: 우리 녹색에도 아직 건너는 사람이 있으면 선다."""
    st, rt = _h_stack()
    s = _h_state(rt, 960.4, 655.4)
    s_ego = st.rf.project(s.x, s.y)[0]
    assert st._crosswalk_ahead(s, 13.9, s_ego, zone_only=False)[0] is None       # 옛 동작
    key, _ = st._crosswalk_ahead(s, 13.9, s_ego, zone_only=False, all_cw=True)
    assert key == ("3102", 8.03), key
    body = st._cw_body_fwd(s, key)
    assert 8.5 < body < 9.5, body                        # 앞범퍼 기준 실제 거리(투영은 0.75 였다)
    ped = (8.9, 2.3, 0.0, 1.9, 1.09, 1.6, 7, 0.6, 1.9, 0.21)
    assert st._ped_on_crosswalk([ped], body, d_ego=0.0) == [True]
    assert st._cw_stop_fwd(s, key, body) < 0.0           # 정지선이 코앞 -> 여기서 선다


def test_정지선이_있으면_그_앞을_목표로_잡는다():
    """`_crosswalk_ahead` 의 거리는 경로투영이라 코너에서 실제와 크게 다르다(코스 H: 본체
    9.0m 인데 투영 0.75m). 사람 판정도 정지 목표도 **자차 기준 실제 거리**로 잰다."""
    st, rt = _h_stack()
    s = _h_state(rt, 950.5, 650.1)
    key = ("3102", 8.03)
    body = st._cw_body_fwd(s, key)
    tgt = st._cw_stop_fwd(s, key, body)
    assert 19.0 < body < 21.0 and 6.0 < tgt < 8.5, (body, tgt)
    assert st._cw_stop_fwd(s, ("없는", 0.0), body) is None
