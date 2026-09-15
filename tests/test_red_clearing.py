"""적색인데 앞범퍼가 이미 정지선을 넘은 자리 — 서면 횡단보도 위다 (2026-09-11 사전주행2 신호25).

실측(코스 사전주행2, 신호 25, 우회전 접근)
    t=95.5  우회전 감속 15.5km/h 로 정지선을 지나는 중 **황색**
    t=96.9  횡단보도 위에서 정지 -> t=97.5 `YELLOW_GO` 로 재출발
    t=98.5  **적색**. 앞범퍼가 정지선 3.0m 뒤(d_line -3.03)인데 `RED_STOP`
    t=99.4~116.5  **18초 정지 유지**(녹색까지). 그 자리는 횡단보도 위다(중심 3.5m 앞).
`tl_passed` 는 **뒷축**이 선을 넘어야 참이라, 앞범퍼가 3.808m 앞선 우리 차는 '아직 안 지났다'
로 판정됐다. `tl_stop_dist` 가 0.07m 만 더 줄었어도 빠져나갔을 칼날 위의 판정이었다.

고친 것: 적색에서 **앞범퍼**로 재고, 이미 굴러가는 중이면 빠져나간다(`RED_CLEARING`).
선 차를 적색에 출발시키지는 않는다 — 그건 항목7 -6 이다.
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from behavior import Behavior                                          # noqa: E402
from vtd_io import State, TL_RED, TL_GREEN                             # noqa: E402

FO = 3.81                       # front_overhang 기본값


def _ego(v, tl_state=TL_RED, tl_id=25, t=0.0):
    s = State()
    s.speed, s.tl_state, s.tl_id, s.t, s.heading = v, tl_state, tl_id, t, 0.0
    return s


def _plan(b, v, d_line, t=0.0, tl_state=TL_RED, tl_id=25):
    """`d_line` = 앞범퍼~정지선[m] (음수면 이미 넘었다). 반환 (속도, 사유)."""
    v_out, _off, _turn, reason = b.plan(_ego(v, tl_state, tl_id, t), 8.0,
                                        tl_stop_dist=d_line + b.front_overhang)
    return v_out, reason


def test_앞범퍼가_정지선을_넘었고_굴러가는_중이면_빠져나간다():
    b = Behavior()
    v, reason = _plan(b, 1.52, -3.03)                 # 실측 t=98.49: 5.5km/h, 앞범퍼 3.0m 초과
    assert reason == "RED_CLEARING", reason
    assert v > 0.0, v


def test_그_판정은_그_신호_동안_물고_있는다():
    """감속하다 stop_hold_v 아래로 떨어졌다고 다시 서면 횡단보도 위에서 멈춘다."""
    b = Behavior()
    assert _plan(b, 1.52, -3.03, t=0.0)[1] == "RED_CLEARING"
    assert _plan(b, 0.05, -2.90, t=0.1)[1] == "RED_CLEARING"           # 거의 멈췄어도 계속 나간다


def test_다음_신호로_물고_가지_않는다():
    b = Behavior()
    assert _plan(b, 1.52, -3.03, t=0.0, tl_id=25)[1] == "RED_CLEARING"
    _v, reason = _plan(b, 1.52, 6.0, t=1.0, tl_id=31)                   # 다음 신호, 아직 6m 앞
    assert reason != "RED_CLEARING", reason
    assert b._red_clear_tl is None


def test_적색이_풀리면_래치도_풀린다():
    b = Behavior()
    assert _plan(b, 1.52, -3.03, t=0.0)[1] == "RED_CLEARING"
    _plan(b, 1.52, -3.03, t=1.0, tl_state=TL_GREEN)
    assert b._red_clear_tl is None


def test_서_있는_차를_적색에_출발시키지는_않는다():
    """앞범퍼가 넘었어도 이미 서 있으면 그대로 둔다 — 적색 출발은 항목7 -6 이다."""
    b = Behavior()
    v, reason = _plan(b, 0.0, -3.03)
    assert reason != "RED_CLEARING", reason
    assert v == 0.0, v


def test_정지선_앞에_제대로_선_차는_그대로_선다():
    """정상 적색 정지는 앞범퍼가 선 앞 1.59m(stop_margin 5.4 - 앞오버행 3.81)다 — 안 건드린다."""
    b = Behavior()
    for v in (0.0, 1.0, 5.0):
        _out, reason = _plan(b, v, 1.59)
        assert reason in ("RED_STOP", "RTOR_YIELD"), (v, reason)


def test_문턱을_넘지_않은_미세_초과는_그대로_선다():
    """정지선을 0.2m 넘은 정도로 적색을 통과하면 안 된다 — red_clear_pad 0.3m."""
    b = Behavior()
    assert b.red_clear_pad == 0.3
    assert _plan(b, 1.52, -0.2)[1] != "RED_CLEARING"
