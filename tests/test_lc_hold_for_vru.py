"""경로가 **사람이 서 있는 차로로 차선을 바꿔 들어가면**, 지금 차로를 지키다 지나간 뒤 바꾼다 (2026-09-11 코스 H TRV).

실측 (1480,515): 경로가 3차로 -> 4차로로 옮겨 가는 바로 그 자리의 **4차로**에 휠체어(지도 xodr 로 확인:
휠체어 t=11.26 은 4차로 [9.83,13.29], 우리 차 t=8.22 는 3차로 [6.63,9.81]). 경로 기준으로는 휠체어가
'진로 우 1.0m' 라 반대쪽으로 한 차로를 통째로 비켜 **2차로 쪽으로 튀었다**. 3차로로 그대로 가면
몸 옆 1.65m 로 지나간다. 사용자: "옆 차선에 서있고만 왜 직진하면 부딪혀".
차로계획 수치(l·r·폭)는 그 판 routes/HL_FMA_NEW_H_lane.json idx 1088~1114 와 같은 모양이다
(l+r = 12.6 그대로, l 4.76 -> 7.95 로 경로가 오른쪽으로 3.2m 옮겨 감).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from drive import DrivingStack, straight_route                         # noqa: E402

W = 3.17
L0, R0 = 4.755, 7.925                    # 경로(3차로 중심)에서 차도 좌·우 끝까지
LC0, LC1 = 95.0, 122.0                   # 경로가 3 -> 4차로로 옮겨 가는 구간[m]


def _drift(s):
    if s <= LC0:
        return 0.0
    if s >= LC1:
        return W
    return W * (s - LC0) / (LC1 - LC0)


def _plan(drift=_drift, width_step=None):
    pts = []
    for i in range(300):
        dr = drift(float(i))
        extra = width_step(float(i)) if width_step else 0.0
        pts.append({"lane": 3, "w": W, "l": L0 + dr + extra, "r": R0 - dr, "xl": 0.0, "xr": 0.0,
                    "need": 0.0, "sig": -1, "j": 0, "jx": 0, "lim": 13.889})
    return pts


def _stack(**kw):
    return DrivingStack(route=straight_route(300.0), lane_plan=_plan(**kw))


def _wc(ds, d_path, d_ego, spd=0.0, oid=9):
    """rf_objs 한 줄: (ds, d[ego 기준], speed, olen, owid, hgt, id, raw_w, raw_l, dh)."""
    return (ds, d_path - d_ego, spd, 1.0, 0.75, 1.8, oid, 0.7, 1.0, 0.0)


def test_경로가_휠체어_차로로_옮겨_가면_지금_차로를_지킨다():
    """ego s=90(아직 3차로), 휠체어 s=114 경로 우 1.0m(=4차로)."""
    st = _stack()
    got = st._lc_hold_target([_wc(24.0, -1.0, 0.0)], d_ego=0.0, bi=90)
    assert got is not None
    assert abs(got - 0.0) < 0.05, got                    # 지금은 경로가 3차로 -> 거의 0(한 차로 비키기 3.17 아님)
    got2 = st._lc_hold_target([_wc(4.0, -1.0, 2.2)], d_ego=2.2, bi=110)
    exp = _drift(114.0)                                  # 4m 앞 경로가 옮겨 간 만큼 되돌린다
    assert abs(got2 - exp) < 0.05, (got2, exp)


def test_휠체어를_지나가면_풀린다():
    st = _stack()
    st._lc_hold_target([_wc(24.0, -1.0, 0.0)], d_ego=0.0, bi=90)
    assert st._lc_hold is not None
    assert st._lc_hold_target([_wc(-3.5, -1.0, 2.5)], d_ego=2.5, bi=118) is None
    assert st._lc_hold is None


def test_경로가_안_옮겨_가면_예전_비키기다():
    """차로 안 휠체어를 그냥 비켜야 하는 흔한 경우 — 이 규칙은 끼어들지 않는다."""
    st = _stack(drift=lambda s: 0.0)
    assert st._lc_hold_target([_wc(24.0, -1.0, 0.0)], d_ego=0.0, bi=90) is None


def test_차로가_늘어나는_자리는_경로_쏠림으로_안_본다():
    """왼쪽에 차로가 새로 생기면 l 만 늘고 차도 폭(l+r)이 바뀐다 — 그걸 경로 이동으로 착각하면 안 된다."""
    st = _stack(drift=lambda s: 0.0, width_step=lambda s: (W if s > 100 else 0.0))
    assert st._lc_hold_target([_wc(24.0, -1.0, 0.0)], d_ego=0.0, bi=90) is None


def test_지금_차로로도_못_지나가면_예전_비키기다():
    """사람이 내 차로 쪽으로 붙어 서 있으면(경로 우 1.0m 가 아니라 경로 좌 1.0m) 지켜도 스친다."""
    st = _stack()
    # s=114 에서 지금 차로(경로 기준 +2.2)와 사람(+1.3)의 몸 옆 여유 = 0.9 - 0.94 - 0.375 < 1.2
    assert st._lc_hold_target([_wc(24.0, 1.3, 0.0)], d_ego=0.0, bi=90) is None


def test_걷는_사람은_이_규칙이_아니다():
    st = _stack()
    assert st._lc_hold_target([_wc(24.0, -1.0, 0.0, spd=1.3)], d_ego=0.0, bi=90) is None


def test_nudge_는_경로가_옮겨_가는_만큼만_따라간다():
    """_nudge_offset 에 bi 를 주면 한 차로(3.17)를 통째로 비키지 않고 차로 유지 목표로 간다."""
    st = _stack()
    plan = st._plan_at(90)
    n = 0.0
    for _ in range(30):                                  # 1.2초
        n = st._nudge_offset([_wc(24.0, -1.0, 0.0)], d_ego=0.0, dt=0.04, plan=plan,
                             ped_bypass=True, ego_speed=2.0, bi=90)
    assert abs(n) < 0.05, n                              # 예전 코드는 여기서 +1.44(3.17 로 가는 중)


def test_차로를_지키는_동안_행동층에는_지금_차로_기준_위치를_넘긴다():
    """경로 기준 우 1.0m 는 경로가 옮겨 간 탓이다 — 지금 차로 기준으로는 우 3.23m(옆 차로).
    그걸 안 고치면 행동층이 45km/h 에서 a=-5 로 서다가 7km/h 로 긴다(실측)."""
    st = _stack()
    objs = [_wc(24.0, -1.0, 0.0), (10.0, 3.0, 5.0, 4.4, 1.8, 1.5, 2, 1.8, 4.4, 0.0)]
    st._lc_hold_target(objs, d_ego=0.0, bi=90)
    view = st._lc_hold_view(objs, 90)
    exp = -1.0 - (_drift(114.0) - _drift(90.0))
    assert abs(view[0][1] - exp) < 0.05, (view[0][1], exp)
    assert view[1] == objs[1]                                          # 붙잡지 않은 물체는 그대로
    st._lc_hold = None
    assert st._lc_hold_view(objs, 90) is objs                          # 안 붙잡으면 원래 목록
