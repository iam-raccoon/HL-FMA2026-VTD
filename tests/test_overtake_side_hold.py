"""추월 방향(`set_side`)이 매 프레임 뒤집혀 **깜빡이가 떠는** 걸 막는다.

실측 2026-09-05 코스 A t=264.8~289.8 (1389,-161) 적신호 앞 정지 중:
  `off`·`nudge`·`d_ego` 가 전부 0 — 차는 1cm 도 안 움직였는데
  깜빡이가 **25초에 7번** L↔R 로 뒤집혔다(주변 물체 6~7개).
  사용자 지적: "왜 직진 차선에서 잘 기다리면서 좌우 깜빡이가 난리가 남?"

원인: `drive.py` 가 갇힘을 피하려고 **매 프레임** 좌우를 다시 고르고
(2026-08-25 코스 E 250초 교착을 그걸로 고쳤다), WAIT 의 지시등은
`TS_LEFT if W > 0 else TS_RIGHT` 라 `W` 부호를 그대로 따라간다.
거기 주석의 "차는 안 흔들린다" 는 맞지만 **깜빡이는 흔들린다.**

처방: 재평가는 그대로 두고 **확정만 SIDE_HOLD 만큼 늦춘다.**
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from overtake import Overtaker, TS_LEFT, TS_RIGHT                  # noqa: E402


def _ov(state="WAIT", side=+1.0):
    o = Overtaker(lane_width=3.0, side=side)
    o.state = state
    return o


def _sign(o):
    return 1.0 if o.W >= 0 else -1.0


def test_좌우가_매프레임_번갈아_와도_방향이_안_바뀐다():
    """★이게 깜빡이 떨림의 원인이었다."""
    o = _ov()
    prev, flips, t = _sign(o), 0, 0.0
    for k in range(250):                       # 25Hz x 10초
        t += 0.04
        o.set_side(+1.0 if k % 2 == 0 else -1.0, t)
        if _sign(o) != prev:
            flips += 1
            prev = _sign(o)
    assert flips == 0, f"10초에 {flips}번 뒤집혔다"


def test_한쪽을_꾸준히_요청하면_SIDE_HOLD_뒤에_바뀐다():
    """갇힘 탈출은 살아 있어야 한다 — 영영 안 바뀌면 그게 더 나쁘다."""
    o = _ov()
    t = 0.0
    changed = None
    while t < 4.0 and changed is None:
        t += 0.04
        o.set_side(-1.0, t)
        if o.W < 0:
            changed = t
    assert changed is not None, "우측으로 영영 안 바뀐다"
    assert o.SIDE_HOLD <= changed <= o.SIDE_HOLD + 0.1, f"{changed:.2f}초"


def test_기동_중에는_절대_안_바뀐다():
    """PASS 중 방향이 뒤집히면 장애물을 가로질러 되돌아온다."""
    for st in ("PASS", "RETURN"):
        o = _ov(state=st)
        for k in range(200):
            o.set_side(-1.0, k * 0.04)
        assert o.W > 0, f"{st} 에서 방향이 바뀌었다"


def test_이미_그쪽이면_아무_일도_없다():
    o = _ov()
    for k in range(50):
        o.set_side(+1.0, k * 0.04)
    assert o.W > 0
    assert o._side_want is None


def test_요청이_끊기면_대기가_초기화된다():
    """0.7초쯤 우측을 요청하다 좌측으로 돌아가면, 다시 우측을 요청해도
    처음부터 0.8초를 채워야 한다 — 안 그러면 깜빡임이 새어 나온다."""
    o = _ov()
    t = 0.0
    for _ in range(17):                        # 0.68초
        t += 0.04
        o.set_side(-1.0, t)
    assert o.W > 0, "아직 바뀌면 안 된다"
    t += 0.04
    o.set_side(+1.0, t)                        # 요청이 끊긴다
    for _ in range(17):
        t += 0.04
        o.set_side(-1.0, t)
    assert o.W > 0, "끊겼는데 이어서 세고 있다"


def test_WAIT_지시등이_방향을_그대로_따른다():
    """이 테스트가 깨지면 위 히스테리시스가 지시등을 못 막는다는 뜻이다."""
    src = open(os.path.join(os.path.dirname(__file__), "..", "src",
                            "overtake.py"), encoding="utf-8").read()
    assert "TS_LEFT if self.W > 0 else TS_RIGHT" in src, \
        "WAIT/PASS 지시등이 W 부호를 따르지 않게 바뀌었다 — set_side 히스테리시스 재검토"
