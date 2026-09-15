"""지시등 두 건 — 2026-09-11 v6 수정판(선 차를 좌회전 차로로 비켜 지나간 판).

① 사용자: "아무리 급박하게 차선변경해도 깜빡이는 켜야지"
   추월 복귀가 적신호에 **선 채로** 끝났다(명령 3.10 -> 0 을 0.3초에, RETURN 은 15초 뒤
   시간초과로 FOLLOW). 녹색에 차가 실제로 3.1m 를 돌아올 때는 명령이 이미 0 이라
   '명령이 변하는 중'만 보는 규칙(SIGNAL_RATE)에 안 걸려 **지시등 없이** 차로를 옮겼다.
② 사용자: "차선 들어와서 신호대기로 정지하면 깜빡이 바로 끄고"
   좌회전 차로로 다 들어가 적신호에 섰는데 **좌측 지시등을 13초 켠 채**였다.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from drive import DrivingStack, straight_route                         # noqa: E402
from overtake import Overtaker, TS_LEFT, TS_OFF                        # noqa: E402
from vtd_io import State                                               # noqa: E402

ROAD = dict(lane=2, w=3.1, l=4.75, r=10.85, xl=0.0, xr=0.0, need=0.0, sig=0, j=0, jx=0)
JUNC = dict(ROAD, j=1, jx=1)


def test_명령은_끝났는데_차체가_옆_차로면_그쪽을_켠다():
    st = DrivingStack(route=straight_route(100.0))
    assert st._pending_lat_signal(0.0, 0.1, "FOLLOW", ROAD) == 0      # 제자리(=한 번 들어옴)
    assert st._pending_lat_signal(0.0, 3.11, "FOLLOW", ROAD) == -1    # 오른쪽으로 돌아가야 한다


def test_출발_직후_경로_옆에_놓인_것에는_안_켠다():
    """출발·리스폰 직후 차는 경로 옆에 놓인다(v6 시작 3.6m). 녹화 6판 재생에서 이걸 안 막으면
    A·B·D·H 전부 첫 3초에 우측이 켜졌다."""
    st = DrivingStack(route=straight_route(100.0))
    assert st._pending_lat_signal(0.0, 2.5, "FOLLOW", ROAD) == 0


def test_옮기다_교차로에_들어가도_끝날_때까지_켠다():
    st = DrivingStack(route=straight_route(100.0))
    st._pending_lat_signal(0.0, 0.0, "FOLLOW", ROAD)
    st._pend_sig = st._pending_lat_signal(0.0, 3.11, "FOLLOW", ROAD)
    st._pend_sig = st._pending_lat_signal(0.0, 1.8, "FOLLOW", JUNC)
    assert st._pend_sig == -1                                          # 교차로 안에서도 유지
    assert st._pending_lat_signal(0.0, 0.3, "FOLLOW", JUNC) == 0       # 들어오면 끈다


def test_교차로_안에서_새로_켜지지는_않는다():
    """회전 중 코너를 파고드는 추종 오차에 켜지면 반대쪽 지시등이 된다."""
    st = DrivingStack(route=straight_route(100.0))
    st._pending_lat_signal(0.0, 0.0, "FOLLOW", ROAD)
    assert st._pending_lat_signal(0.0, -1.8, "FOLLOW", JUNC) == 0


def test_추월_중에는_추월기가_켠다():
    st = DrivingStack(route=straight_route(100.0))
    st._pending_lat_signal(0.0, 0.0, "FOLLOW", ROAD)
    assert st._pending_lat_signal(3.1, 0.0, "PASS", ROAD) == 0


def _passing(d_ego):
    o = Overtaker(lane_width=3.1, side=+1.0)
    o.state, o.W = "PASS", 3.1
    s = State(); s.speed = 0.0
    _off, turn, _st, _b = o.plan(s, [], now=10.0, d_ego=d_ego)
    return turn


def test_옆_차로에_다_들어갔으면_추월_지시등을_끈다():
    assert _passing(3.11) == TS_OFF                                    # 다 들어옴 -> 끈다
    assert _passing(1.5) == TS_LEFT                                    # 옮기는 중 -> 켠다
