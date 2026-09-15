"""녹색에 들어가다 막혀 **선을 넘은 자리**에 선 채 적색이 되면, 적색 정지를 채우고 우회전한다 (2026-09-11 코스 H 신호155).

실측: 녹색에 우회전하러 들어가다 앞차가 회전 중 멈춰 JUNCTION_JAM 으로 **앞범퍼가 정지선을 0.73m 넘은
자리**(tl_stop_dist 3.08 = 뒷축 기준)에 섰다. 황 -> 적. 우회전 일시정지(`_rt_done`)는 황색에 끝났는데
적신호 우회전 조건이 '선 **전** 0~2m' 라 적색 정지를 한 번도 못 셌다 -> 앞차가 빠진 뒤(90초) 녹색(105.8초)까지
RED_STOP. 사용자: "왜 우회전때 바로바로 안함?? 기달리다가 신호가 바뀌면 먼가 멈추는듯".
⚠️ **적색에 선을 넘어 선 것**(과주행)은 예전대로 안 된다 — test_behavior 의 09-08 테스트가 그걸 지킨다.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from behavior import Behavior                                           # noqa: E402
from vtd_io import State, TL_GREEN, TL_YELLOW, TL_RED                   # noqa: E402

TL = 155


def _ego(v, tl_state, t):
    s = State(); s.speed = v; s.tl_state = tl_state; s.tl_id = TL; s.t = t; s.heading = 0.0
    return s


def _run(bumper_gap, green_s=1.0, yellow_s=3.0, red_s=2.0):
    """녹색에 다가가 `bumper_gap`(앞범퍼~선, 음수=넘음)에 서고, 황 -> 적. 적색 동안의 사유 목록."""
    b = Behavior()
    d = b.front_overhang + bumper_gap
    b.plan(_ego(4.0, TL_GREEN, 0.0), 8.33, tl_stop_dist=b.front_overhang + 6.0, route_turn=-1)
    t = 0.1
    for state, dur in ((TL_GREEN, green_s), (TL_YELLOW, yellow_s)):
        for _ in range(int(round(dur / 0.1))):
            b.plan(_ego(0.0, state, t), 8.33, tl_stop_dist=d, route_turn=-1)
            t += 0.1
    seen = []
    for _ in range(int(round(red_s / 0.1))):
        seen.append(b.plan(_ego(0.0, TL_RED, t), 8.33, tl_stop_dist=d, route_turn=-1)[3])
        t += 0.1
    return b, seen


def test_선을_넘어_선_채_적색이_되면_적색_정지를_채우고_우회전한다():
    b, seen = _run(-0.73)                                              # 실측 그 자리
    assert "RIGHT_ON_RED" in seen, seen
    first = seen.index("RIGHT_ON_RED")
    assert first * 0.1 >= b.stop_hold_s - 1e-6, (first, seen)          # 적색에서 0.8초는 선다


def test_선_전에_선_것은_예전대로다():
    _b, seen = _run(1.0)
    assert "RIGHT_ON_RED" in seen, seen


def test_선을_2m_넘게_넘어_섰으면_인정하지_않는다():
    """횡단보도 쪽으로 깊이 들어간 자리는 '정지선에서 섰다'가 아니다."""
    _b, seen = _run(-2.5)
    assert "RIGHT_ON_RED" not in seen, seen
