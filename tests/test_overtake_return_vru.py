"""추월을 마치고 돌아갈 원래 차로에 **서 있는 사람**이 아직 앞에 있으면 안 돌아간다 (2026-09-11 코스 H TRV).

실측 (1476,512): 차로 안에 정지차(우로 1.48m 치우침) -> 9m 뒤 휠체어(우 1.0m).
  336.6  정지차를 뒤 6.1m 로 넘기자 RETURN — 휠체어는 **2.8m 앞**. 사람은 추월 대상이 아니라 안 보였다.
  337~339 휠체어 쪽으로 -33° 꺾으며 2.1m 옆을 스침
  339.5  복귀 끝 -> 휠체어 몫으로 쌓인 nudge 2.17 이 이어받아 +20° 로 **다시 1차로**
사용자: "왜 굳이 1차선으로 갔다가 다시 2로 돌아왔다가 하는거임??"
아래 수치는 그 판 로그 그대로다.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from overtake import Overtaker                                          # noqa: E402
from vtd_io import State                                                # noqa: E402

DE = 3.19                                                               # 1차로에 다 들어가 있다


def _row(oid, ds, d_path, spd, L, W, H):
    """추월기 objs 한 줄: (id, ds, d[내 위치 기준], spd, len, wid, hgt, raw_w, raw_l)."""
    return (oid, ds, d_path - DE, spd, L, W, H, W, L)


def _car(ds):
    return _row(51, ds, -1.48, 0.0, 4.6, 1.8, 1.5)


def _wheelchair(ds, d_path=-1.0, spd=0.0):
    return _row(52, ds, d_path, spd, 1.0, 0.7, 1.8)


def _step(o, objs, now, v=3.0):
    s = State(); s.speed = v
    return o.plan(s, objs, now=now, d_ego=DE)


def _passing():
    o = Overtaker(lane_width=3.18, side=+1.0)
    o.state, o.W, o.block_id = "PASS", 3.18, 51
    return o


def test_정지차를_지났어도_휠체어가_앞에_있으면_1차로에_남는다():
    off, _t, st, _b = _step(_passing(), [_car(-6.5), _wheelchair(2.8)], 10.0)
    assert st == "PASS" and abs(off - 3.18) < 1e-9, (st, off)


def test_휠체어까지_뒤로_넘기면_돌아간다():
    _off, _t, st, _b = _step(_passing(), [_car(-12.0), _wheelchair(-3.5)], 10.0)
    assert st == "RETURN"


def test_사람이_없으면_예전처럼_정지차를_지나자마자_돌아간다():
    _off, _t, st, _b = _step(_passing(), [_car(-6.5)], 10.0)
    assert st == "RETURN"


def test_보도에_선_사람은_복귀를_막지_않는다():
    """원래 차로 우측 4.0m — 돌아가도 몸 옆에 2.8m 가 남는다(문턱 1.2m)."""
    _off, _t, st, _b = _step(_passing(), [_car(-6.5), _wheelchair(5.0, d_path=-4.0)], 10.0)
    assert st == "RETURN"


def test_걷는_사람은_여기서_붙잡지_않는다():
    """횡단 중인 사람은 행동층이 세운다."""
    _off, _t, st, _b = _step(_passing(), [_car(-6.5), _wheelchair(2.8, spd=1.3)], 10.0)
    assert st == "RETURN"


def test_막던_차가_사라져도_휠체어가_앞에_있으면_남는다():
    """block_lost 로 돌아가는 길도 같은 조건이다."""
    o = _passing()
    _step(o, [_wheelchair(2.8)], 10.0)
    _off, _t, st, _b = _step(o, [_wheelchair(2.6)], 10.6)             # block_lost_hold(0.4초) 지남
    assert st == "PASS"
    o2 = _passing()
    _step(o2, [], 10.0)
    _off, _t, st2, _b = _step(o2, [], 10.6)
    assert st2 == "RETURN"
