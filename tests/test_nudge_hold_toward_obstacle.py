"""비켜 가던 것이 아직 앞에 있으면 그쪽으로 되돌아가지 않는다 (2026-09-11 코스 H TRV).

실측 `HL_FMA_NEW_H_TRV` (1478,495): 차로 안 오른쪽 1.0m 에 선 휠체어(1.0x0.7x1.8)를
좌측 차로(+3.17m)로 비키는 중이었다.
  298.80  뒤에서 온 차(5.2x1.9, 8.9m/s)가 우리와 휠체어 사이로 파고듦 — 목표 자리 여유 0.34m
          (< FIT_GAP_CAR 0.35). `_lat_free` 가 막힘이라며 target=0 -> nudge 3.17 -> 0,
          조향 -35° 로 **휠체어 쪽으로** 꺾음. 그 차(-0.12m)·자전거(-0.24m)와 스침.
  301~306 왼쪽 차로 23~31m 앞에 13.5m/s 로 **달아나는** 차 -> 또 막힘 -> nudge 0 유지.
          차로 중심으로 기어 들어가다
  306.9   휠체어와 **실여유 -0.18m**. 사용자: "사람보고 망함".
아래 수치는 그 판 로그 그대로다(경로축 폭 rw 까지).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from drive import DrivingStack, straight_route                         # noqa: E402

# 그 자리 차로계획 그대로(routes/HL_FMA_NEW_H_lane.json idx 1094~1099: w 3.17~3.18, l 5.2~5.9).
#  ⚠️ 폭이 중요하다 — 목표가 3.17 이라 파고든 차와의 여유가 0.34(<0.35)였다. 3.2 면 0.37 로 통과한다.
PLAN = {"lane": 3, "w": 3.17, "l": 5.4, "r": 7.2, "xl": 0.0, "xr": 0.0, "need": 0.0,
        "sig": 0, "j": 0, "jx": 0, "lim": 13.889}


def _wheelchair(ds, d_path, d_ego):
    """rf_objs 한 줄: (ds, d[ego 기준], speed, olen, owid, hgt, id, raw_w, raw_l, dh)."""
    return (ds, d_path - d_ego, 0.0, 1.0, 0.75, 1.8, 9, 0.7, 1.0, 0.0)


def _car(ds, d_path, d_ego, spd, rw=1.9, oid=2):
    return (ds, d_path - d_ego, spd, 5.2, rw, 1.5, oid, 1.9, 5.2, 0.0)


def _stack(nudge):
    st = DrivingStack(route=straight_route(300.0))
    st._nudge = nudge
    return st


def _step(st, objs, d_ego):
    return st._nudge_offset(objs, d_ego=d_ego, dt=0.04, plan=PLAN, ped_bypass=True,
                            ego_speed=2.0)


def test_비키는_중_옆자리가_막히면_휠체어_쪽으로_돌아가지_않는다():
    """298.80 그 프레임: 파고든 차 때문에 목표 자리가 막혔다."""
    de = 3.03
    objs = [_wheelchair(12.1, -1.0, de), _car(-2.7, 0.73, de, 8.9, rw=2.31)]
    st = _stack(3.17)
    assert not st._lat_free(3.17, objs, de)              # 전제: 예전엔 여기서 target=0
    got = _step(st, objs, de)
    assert abs(got - 3.17) < 1e-6, got


def test_앞으로_달아나는_차가_있어도_비킨_자리를_유지한다():
    """303.0 무렵: 왼쪽 차로 23m 앞에 13.4m/s 로 멀어지는 차."""
    de = 3.10
    objs = [_wheelchair(11.0, -1.0, de), _car(23.5, 2.92, de, 13.4, rw=1.77, oid=3)]
    st = _stack(3.10)
    got = _step(st, objs, de)
    assert abs(got - 3.10) < 1e-6, got


def test_휠체어를_지나가면_예전처럼_돌아온다():
    de = 3.10
    objs = [_wheelchair(-3.5, -1.0, de), _car(23.5, 2.92, de, 13.4, rw=1.77, oid=3)]
    st = _stack(3.10)
    got = _step(st, objs, de)
    assert got < 3.10 - 1e-3, got                         # 되돌아오기 시작한다


def test_아직_안_나갔으면_비킬_자리가_막힐_때_예전처럼_선다():
    """'0 = 안 비키고 선다'는 그대로다 — 나가기 전에는 붙잡을 자리가 없다."""
    de = 0.0
    objs = [_wheelchair(20.0, -1.0, de), _car(10.0, 3.2, de, 0.0, oid=4)]   # 옆 차로에 선 차
    st = _stack(0.0)
    got = _step(st, objs, de)
    assert abs(got) < 1e-6, got


def test_더_바깥_차로로_나가는_것은_막지_않는다():
    """되돌아가는 쪽만 막는다. 휠체어에서 멀어지는 쪽은 그대로 간다."""
    st = _stack(1.0)
    assert not st._return_blocked(3.2, [_wheelchair(10.0, -1.0, 1.0)], 1.0)


def test_이미_충분히_떨어진_사람은_돌아가는_길을_막지_않는다():
    """보도 쪽 3.5m 에 선 사람 — 차로 중심으로 돌아가도 몸 옆에 2.2m 가 남는다(문턱 1.2m)."""
    st = _stack(3.2)
    assert not st._return_blocked(0.0, [_wheelchair(10.0, -3.5, 3.2)], 3.2)


def test_움직이는_사람은_여기서_붙잡지_않는다():
    """횡단 중인 사람은 행동층이 세운다(비키는 대상이 아니다)."""
    st = _stack(3.2)
    walker = (10.0, -1.0 - 3.2, 1.3, 2.0, 0.6, 1.7, 7, 0.6, 2.0, 0.0)
    assert not st._return_blocked(0.0, [walker], 3.2)


def test_옆_차로_중앙으로_맞추는_것은_막지_않는다():
    """목표가 `_lat_free` 를 통과한 자리면 사람 쪽으로 조금 옮겨도 된다(몸 옆 1.76m 남음)."""
    st = _stack(-3.16)
    ped = (10.0, 0.0, 0.0, 2.0, 0.6, 1.7, 9, 0.6, 2.0, 0.0)
    assert not st._return_blocked(-3.0, [ped], 0.0)
