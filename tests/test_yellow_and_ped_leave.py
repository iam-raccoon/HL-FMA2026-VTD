"""2026-09-11 코스 B TRV 두 건.

B-1 신호173 (1283.0,420.1) — **정지선 0.96m 앞에 서 있다가 황색에 출발**해 적색에 건넜다.
    t=52.4~60.0 녹색 내내 횡단 보행자 때문에 정지(CROSSWALK_PED)
    t=60.4      보행자가 비켜나 출발(PED_STILL_ASIDE), 0.75m/s
    t=60.8      황색. 정지선까지 0.53m 인데 `d_brake` 는 **반응 0.8초**가 들어가 0.66m
                -> 13cm 차이로 '못 선다' -> YELLOW_GO -> 그대로 통과
    사용자: "빨간불에 왜 지나가"
    [시행규칙 별표2] 황색등화에 정지선 직전 정지가 원칙이고, 신속 통과는 **이미 교차로에
    일부라도 진입한 경우**뿐이다. 2.7km/h 짜리 차에 딜레마존은 없다.

B-2 (1306.8,480.1) — 횡단 중인 보행자가 **차도를 다 건너기 전에** 출발했다.
    보행자는 오른쪽 -7.1m 에서 왼쪽으로 1.3m/s 로 횡단. 우리 차도는 왼쪽 1.61m·오른쪽 4.91m.
    t=193.4 보행자 +3.0m 에서 '멀어진다'로 제외 -> 출발 -> 42km/h 로 그 옆을 지났다.
    사용자: "사람이 횡단보도를 다 지나가야 움직여야지 반 지나가자마자 움직이노"
"""
import math
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from behavior import Behavior                                          # noqa: E402
from drive import DrivingStack, straight_route                         # noqa: E402
from vtd_io import State, TL_YELLOW                                    # noqa: E402


def _ego(v, t=0.0, tl_id=173):
    s = State(); s.speed = v; s.tl_state = TL_YELLOW; s.tl_id = tl_id; s.t = t
    s.heading = 0.0
    return s


def _yellow(b, v, d_line, t=0.0):
    _v, _o, _tn, reason = b.plan(_ego(v, t), 8.0, tl_stop_dist=d_line + b.front_overhang)
    return reason


# ---------------------------------------------------------------- B-1 황색
def test_서_있던_차는_황색에_출발하지_않는다():
    """실측 t=60.8: 0.75m/s · 정지선 0.53m 앞. 반응거리가 들어가 13cm 차이로 GO 가 됐었다."""
    b = Behavior()
    assert b.yellow_go_min_v == 2.0
    assert _yellow(b, 0.75, 0.53) == "YELLOW_STOP"
    assert _yellow(Behavior(), 0.0, 0.96) == "YELLOW_STOP"


def test_이미_정지선을_넘었으면_빠져나간다():
    """[시행규칙 별표2] 교차로에 일부라도 진입했으면 신속히 통과. 앞범퍼가 넘었으면 그 경우다."""
    assert _yellow(Behavior(), 0.75, -1.0) == "YELLOW_GO"


def test_빠른_차의_딜레마존_판정은_그대로다():
    """50km/h 로 정지선 5m 앞이면 못 선다 — 그건 원래대로 통과다."""
    assert _yellow(Behavior(), 13.9, 5.0) == "YELLOW_GO"
    assert _yellow(Behavior(), 13.9, 40.0) == "YELLOW_STOP"


# ---------------------------------------------------------------- B-2 횡단보도 보행자
ROAD = dict(lane=1, w=3.23, l=1.61, r=4.91, xl=0.0, xr=0.0, need=0.0, sig=0, j=0, jx=0, lim=13.889)


def _ped(dp, deg=90.0, spd=1.3, ds=9.0):
    """경로기준 횡위치 `dp` 에 있는 걷는 사람. rf_objs 한 줄(len>9, [9]=상대방위rad)."""
    return (ds, dp, spd, 2.0, 0.6, 1.7, 42, 0.6, 2.0, math.radians(deg))


def test_차도를_다_건너기_전에는_계속_센다():
    """실측: +3.0m 에서 빠졌다. 우리 차도 왼쪽 끝은 1.61m, 한 차로 더 = 4.84m."""
    st = DrivingStack(route=straight_route(300.0))
    assert st.CW_PED_LEAVE_LANES == 1.0
    for dp in (1.0, 3.0, 4.0, 4.5):
        assert st._ped_on_crosswalk([_ped(dp)], 9.0, 0.0, plan=ROAD), dp


def test_차도를_한_차로_넘게_벗어나면_뺀다():
    """다 건넌 사람을 붙잡으면 코스 H 처럼 영영 못 간다(2026-09-11 PR #36)."""
    st = DrivingStack(route=straight_route(300.0))
    assert not st._ped_on_crosswalk([_ped(5.5)], 9.0, 0.0, plan=ROAD)
    assert not st._ped_on_crosswalk([_ped(8.0)], 9.0, 0.0, plan=ROAD)


def test_다가오는_사람은_방향과_무관하게_센다():
    """오른쪽(-)에서 왼쪽으로 오는 중 = 내 쪽으로 온다 — 원래대로 잡힌다."""
    st = DrivingStack(route=straight_route(300.0))
    assert st._ped_on_crosswalk([_ped(-6.0)], 9.0, 0.0, plan=ROAD)


def test_차로계획이_없으면_예전대로_횡_3m_다():
    """지도가 없으면(테스트·교차로 안) 가장자리를 모른다 — 한 차로 기본폭만 준다."""
    st = DrivingStack(route=straight_route(300.0))
    assert not st._ped_on_crosswalk([_ped(5.0)], 9.0, 0.0, plan=None)


# ---------------------------------------------------------------- 횡단보도 사람 앞 정지 (2026-09-11 코스 A·B)
def _run_latch(st, key, frames):
    """frames = [(v_cmd, v_cwp, d_tgt)] -> 프레임별 명령 속도."""
    return [st._cwp_latch(key, vc, vw, d) for vc, vw, d in frames]


def test_횡단보도_위_사람이_있는_동안은_다시_빨라지지_않는다():
    """실측 코스 A (986.9,253.6): YIELD_PED 7.2km/h -> 한 프레임 빠지자 CROSSWALK_PED 곡선이
    허용한 **13.2km/h 로 재가속** -> 다시 정지. 사용자: "횡단보도 사람 보고도 안 멈추고 계속 움직여"."""
    st = DrivingStack(route=straight_route(300.0))
    key = ("road", 1.0)
    out = _run_latch(st, key, [(2.0, 3.7, 5.0),          # YIELD_PED 7.2km/h 가 이긴다
                               (13.9, 3.67, 4.8)])       # YIELD_PED 가 빠져도 13.2 로 안 뛴다
    assert out[0] == 2.0 and out[1] < 2.0, out


def test_정지선까지_일정속도로_기지_않고_0_으로_줄여_간다():
    """실측 코스 B (1104.8,365.5): 7.2km/h 를 붙잡은 채 정지선까지 **6~7km/h 로 2.5m 를 기었다**.
    사용자: "사람이 차 정면을 건너는데 왜 기어가냐. 정지선이면 정지선에서 딱 멈춰야지".
    붙잡을 것은 속도가 아니라 '지금 속도에서 정지선에 0 이 되는 감속'이다."""
    st = DrivingStack(route=straight_route(300.0))
    key = ("road", 1.0)
    d = [3.0 - 0.08 * k for k in range(40)]              # 2m/s 로 0.04초마다 다가간다
    out = _run_latch(st, key, [(2.0 if k == 0 else 13.9, 9.0, x) for k, x in enumerate(d)])
    assert all(b <= a + 1e-9 for a, b in zip(out, out[1:])), "내려가기만 해야 한다"
    assert out[10] < 0.9 * out[0], out[:12]                # 옛 식은 여기서도 2.0 그대로였다
    assert out[-1] == 0.0, out[-5:]                        # 정지선(0.3m 안)에서 0


def test_한_번_섰으면_사람이_내려갈_때까지_선다():
    st = DrivingStack(route=straight_route(300.0))
    key = ("road", 1.0)
    out = _run_latch(st, key, [(2.0, 3.7, 2.0), (0.0, 3.0, 1.5), (13.9, 3.0, 1.5), (13.9, 3.0, 1.5)])
    assert out[1:] == [0.0, 0.0, 0.0], out
    assert st._cwp_latch(("other", 2.0), 13.9, 5.0, 8.0) == 5.0   # 다른 횡단보도는 따로다


def test_사람이_내려가면_래치가_풀린다():
    st = DrivingStack(route=straight_route(300.0))
    key = ("road", 1.0)
    st._cwp_latch(key, 0.0, 3.7, 2.0)
    st._cwp_hold.pop(key, None)                          # 호출부: `_on` 이 비면 지운다
    assert st._cwp_latch(key, 13.9, 6.0, 12.0) == 6.0


# ---------------------------------------------------------------- v6 수정판: 옆 차로에 서 있는 사람 (2026-09-11)
def _stand(ds, dp, ow=0.7):
    """경로기준 횡 dp 에 서 있는 사람 한 줄."""
    return (ds, dp, 0.0, 0.6, ow, 1.8, 77, 0.7, 0.6, 0.0)


def test_옆_차로에_서_있는_사람에게는_차선을_바꾸지_않는다():
    """실측 v6 수정판: 주차차 옆 **2.6~3.3m** 에 서 있는 사람(차로 안 통과 실여유 1.3m)에게
    예전 문턱(횡 2.5m)이 걸려 한 차로를 옮기기 시작했고, 적신호 감속·뛰어 건너는 사람까지
    겹쳐 오프셋이 1.30 -> 0.34 -> 3.14 로 오갔다. 사용자: "정신을 못 차리네"."""
    st = DrivingStack(route=straight_route(300.0))
    for dp in (-2.4, -2.6, -3.3, 2.6):
        assert not st._ped_blocks([_stand(23.0, dp)], 0.0), dp


def test_차로_안에_서_있는_사람은_여전히_막은_것이다():
    st = DrivingStack(route=straight_route(300.0))
    for dp in (0.0, -1.0, 1.9):
        assert st._ped_blocks([_stand(20.0, dp)], 0.0), dp
    assert not st._ped_blocks([_stand(30.0, 0.0)], 0.0)       # 25m 밖은 아직
    walking = (20.0, 0.0, 1.3, 0.6, 0.7, 1.8, 78, 0.7, 0.6, 0.0)
    assert not st._ped_blocks([walking], 0.0)                 # 걷는 사람은 우회 대상이 아니다(정지)
