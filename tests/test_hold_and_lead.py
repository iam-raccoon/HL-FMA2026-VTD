"""2026-09-06 밤 6코스 실주행(주변교통)에서 나온 것 — 추월 대기 게이트와 제한속도 선행 감속.

G 신호 173·198·211: 적신호 앞에 선 앞차를 장애물로 봐 OVT:WAIT -> 추월할 쪽 지시등(L) 14초.
E (1226,-385): 50->30 경계에서 30.9km/h, 8m 지나서 27 — 경계에 정확히 닿게 짜인 역방향 패스.
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from overtake import Overtaker, TS_OFF                               # noqa: E402
from drive import DrivingStack, straight_route                        # noqa: E402
from vtd_io import State                                              # noqa: E402


def _ego(v=0.0):
    s = State(); s.speed = v
    return s


def _stopped_car(ds):
    return (7, ds, 0.0, 0.0, 4.4, 1.8, 1.5)          # (id, ds, d, speed, len, wid, hgt)


def _wait(o, hold=False):
    """정지차를 stopped_hold 보다 오래 보여준다."""
    out = None
    for t in (1.0, 2.0, 3.0, 4.0, 5.0):
        out = o.plan(_ego(3.0), [_stopped_car(15.0)], now=t, hold=hold)
    return out


def test_hold_없으면_예전처럼_대기한다():
    o = Overtaker(); _wait(o)
    assert o.state == "WAIT", o.state


def test_hold_면_대기에도_들어가지_않는다():
    o = Overtaker(); _off, turn, st, _b = _wait(o, hold=True)
    assert st == "FOLLOW", st
    assert turn == TS_OFF


def test_대기_중_hold_가_걸리면_추종으로_돌아온다():
    """적신호로 바뀌면 이미 켠 추월 지시등도 꺼야 한다."""
    o = Overtaker(); _wait(o)
    assert o.state == "WAIT"
    _off, turn, st, _b = o.plan(_ego(0.0), [_stopped_car(15.0)], now=6.0, hold=True)
    assert st == "FOLLOW" and turn == TS_OFF, (st, turn)


def test_hold_가_풀리면_다시_대기할_수_있다():
    o = Overtaker(); _wait(o, hold=True)
    assert o.state == "FOLLOW"
    o.plan(_ego(0.0), [_stopped_car(15.0)], now=6.0, hold=False)
    assert o.state == "WAIT"


def test_제한_하향_지점_앞에서_미리_낮춘다():
    st = DrivingStack(route=straight_route(200.0),
                      lane_plan=[{"lim": 13.889}] * 100 + [{"lim": 8.333}] * 100)
    lim, m = st._lim_idx, st.SPEED_MARGIN
    assert abs(lim[100] - (8.333 - m)) < 1e-3
    assert abs(lim[100 - int(st.LIM_LEAD_M) + 1] - (8.333 - m)) < 1e-3, "LEAD 안은 이미 30"
    assert abs(lim[100 - int(st.LIM_LEAD_M) - 2] - (13.889 - m)) < 1e-3, "LEAD 밖은 아직 50"
    assert abs(lim[150] - (8.333 - m)) < 1e-3                          # 뒤쪽은 그대로


def test_제한이_올라가는_지점은_당기지_않는다():
    """낮추는 쪽만 — 30->50 경계는 경계에서 올라간다(당기면 붉은 노면 위에서 50)."""
    st = DrivingStack(route=straight_route(200.0),
                      lane_plan=[{"lim": 8.333}] * 100 + [{"lim": 13.889}] * 100)
    lim, m = st._lim_idx, st.SPEED_MARGIN
    assert abs(lim[99] - (8.333 - m)) < 1e-3
    assert abs(lim[100] - (13.889 - m)) < 1e-3
