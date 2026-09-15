"""미보고 신호 선감속(TL_UNSEEN) — 2026-09-09 6코스 스윕 실측.

VTD 는 신호 대부분을 60~300m 앞에서 보고하지만 130·167 은 앞범퍼 **9m**, 100 은 12m, 213 은 14m
앞에서야 tl_id 가 온다(같은 신호는 판마다 같은 거리). 코스 B 는 167 을 37km/h 로 9m 앞에서 적색으로
처음 보고 1.9m 지나쳤다(항목7 -6). D 는 같은 신호를 13km/h 로 와서 무사했다.
지도의 도색 정지선(방위 있음) 중 신호 DB 점과 짝이 되는 것을 경로 위에서 골라 두고, 그 선이 앞
TL_ANT_LOOK 안인데 보고된 신호가 그 선을 가리키지 않으면 보고 지점(9m)에서 RED_STOP 이 세울 수
있는 속도까지 미리 계획감속한다.
"""
import math
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from behavior import Behavior                                          # noqa: E402
from drive import DrivingStack, straight_route                         # noqa: E402
from vtd_io import State, TL_UNSET                                     # noqa: E402


def _ego(v=0.0):
    s = State(); s.speed = v; s.tl_state = TL_UNSET; s.tl_id = 0; s.t = 0.0; s.heading = 0.0
    return s


# ---------------------------------------------------------------- drive: 경로 위 신호 정지선 고르기
def test_경로_위_내_방향_신호_정지선만_고른다():
    tl = {"7": [100.0, 1.5], "9": [200.0, -4.0]}                       # 200 은 도색선과 짝이 안 됨(거리 20m)
    lines = [
        [100.0, 0.0, 0.0],              # 신호7 정지선, 내 방향(+x)          -> 채택
        [100.0, 3.3, 0.0],              # 같은 선, 옆 차로 도색            -> 채택(같은 s)
        [150.0, 0.0, math.pi / 2],      # 교차로 다른 갈래(직각)             -> 제외
        [160.0, 0.0, math.pi],          # 반대 방향 차로의 선                -> 제외
        [220.0, 0.0, 0.0],              # 신호 없는 양보선(DB 점과 20m)     -> 제외
        [100.0, 9.0, 0.0],              # 횡 9m — 경로 밖                    -> 제외
    ]
    st = DrivingStack(route=straight_route(300.0), tl_stops_extra=tl, stoplines_all=lines)
    assert [round(s, 1) for s, _x, _y in st._tl_lines] == [100.0, 100.0], st._tl_lines


def test_보고되지_않은_앞_정지선까지_거리를_준다():
    tl = {"7": [100.0, 1.5]}
    st = DrivingStack(route=straight_route(300.0), tl_stops_extra=tl,
                      stoplines_all=[[100.0, 0.0, 0.0]])
    assert st._tl_unseen_dist(50.0, None) == 50.0                       # 50m 앞, 아직 모름
    assert st._tl_unseen_dist(30.0, None) is None                       # 70m — 아직 멀다(TL_ANT_LOOK 60)
    assert st._tl_unseen_dist(50.0, 52.0) is None                       # 보고된 신호의 선이 그 선이다
    assert st._tl_unseen_dist(50.0, 45.0) is None                       # 그보다 앞선 선(도색 early) 도 안다
    assert st._tl_unseen_dist(50.0, 120.0) == 50.0                      # 보고된 건 더 먼 신호 — 이 선은 모른다
    assert st._tl_unseen_dist(101.0, None) is None                      # 지났다
    st2 = DrivingStack(route=straight_route(300.0))                     # 데이터 없으면 꺼짐
    assert st2._tl_lines == [] and st2._tl_unseen_dist(50.0, None) is None


# ---------------------------------------------------------------- behavior: 캡
def test_보고_지점에서_RED_STOP_이_세울_수_있는_속도까지_미리_줄인다():
    b = Behavior()
    b.speed_limit = 13.9                                                # 50km/h 도로(기본 8.33 이면 캡이 안 보인다)
    rep_axle = b.tl_report_d + b.front_overhang                         # 9 + 3.81 = 12.8m(뒷축)
    v_rep = b._stop_target_speed(rep_axle - b.stop_margin)              # RED_STOP 이 그 순간 허용할 속도
    # 코스 B: 13.4m/s 로 접근. 보고 지점(12.8m)에서는 정확히 v_rep
    v, _o, _t, r = b.plan(_ego(v=13.4), 13.9, tl_unseen_dist=rep_axle)
    assert r == "TL_UNSEEN" and abs(v - v_rep) < 1e-6, (v, v_rep, r)
    # 40m 앞: 계획감속(a_plan)으로 v_rep 에 이르는 속도 — 13.4 보다 낮아야 하고 v_rep 보다 높다
    v40, _o, _t, r40 = b.plan(_ego(v=13.4), 13.9, tl_unseen_dist=40.0)
    assert v_rep < v40 < 13.4 and r40 == "TL_UNSEEN", (v40, r40)
    assert abs(v40 - math.sqrt(v_rep ** 2 + 2 * b.a_plan * (40.0 - rep_axle))) < 1e-6
    # 보고 지점을 지나도 계속 모르면 v_rep 로 기어간다(그 선이 신호 선이 아닐 수도 있다 — 서지는 않는다)
    v5, _o, _t, _r = b.plan(_ego(v=13.4), 13.9, tl_unseen_dist=5.0)
    assert abs(v5 - v_rep) < 1e-6 and v5 > 3.0, v5
    # None 이면 예전과 같다
    v0, _o, _t, r0 = b.plan(_ego(v=13.4), 13.9)
    assert r0 == "LANE_KEEP" and abs(v0 - 13.4) < 1e-6 or v0 >= 13.4 - 1e-6, (v0, r0)


def test_실제_코스_B_경로에서_신호167_정지선을_찾는다():
    """routes/ 실데이터: B 경로 위 신호 정지선에 167 의 도색선(1302.8,566.24) 이 들어 있어야 한다."""
    import json
    here = os.path.dirname(os.path.abspath(__file__))
    R = os.path.join(here, "..", "routes")
    route = json.load(open(os.path.join(R, "HL_FMA_NEW_B.json"), encoding="utf-8"))["ego_route"]
    pts = [(float(p["x"]), float(p["y"])) if isinstance(p, dict) else (float(p[0]), float(p[1])) for p in route]
    tl = json.load(open(os.path.join(R, "tl_map_livinglab.json"), encoding="utf-8"))
    lines = json.load(open(os.path.join(R, "stoplines_all.json"), encoding="utf-8"))["stoplines"]
    st = DrivingStack(route=pts, tl_stops_extra=tl, stoplines_all=lines)
    hit = [(s, x, y) for s, x, y in st._tl_lines if math.hypot(x - 1302.8, y - 566.24) < 0.5]
    assert hit, "신호167 정지선이 경로 위 목록에 없다"
    # 코스 B 의 신호는 9개 — 선은 차로 수만큼 겹칠 수 있으니 서로 다른 s 로 묶어 세면 그 근처여야 한다
    groups = []
    for s, _x, _y in st._tl_lines:
        if not groups or s - groups[-1] > 5.0:
            groups.append(s)
    assert 6 <= len(groups) <= 14, (len(groups), groups)


# ---------------------------------------------------------------- drive: 신호가 **없을 때** 도색선 대체값을 넘기지 않는다
def test_신호_미보고면_도색선_대체거리를_보고된_신호로_치지_않는다():
    """재주행 B 2026-09-09 항목7 -6 재발의 원인.

    tl_id=0 인 프레임에서 `_tl_stop_dist` 는 `_tl_stop_painted` 로 앞 도색선 거리를 돌려준다.
    그걸 `_tl_unseen_dist` 에 그대로 넘기면 '그 선은 이미 보고된 신호의 선'이 돼 TL_UNSEEN 이
    **한 번도** 안 걸린다(B 전 구간 cap 0회 · 167 을 또 9m 앞에서 36km/h 로 봄).
    """
    s = _ego(v=10.0)
    s.tl_id = 0
    assert DrivingStack._reported_tl_dist(s, 30.0) is None                # 신호 없음 -> 대체값 버림
    s.tl_id = -1
    assert DrivingStack._reported_tl_dist(s, 30.0) is None
    s.tl_id = 167
    assert DrivingStack._reported_tl_dist(s, 30.0) == 30.0                # 보고된 신호의 선은 그대로
    assert DrivingStack._reported_tl_dist(s, None) is None


def test_실주행_경로에서_신호_없을_때_선감속_거리가_나온다():
    """코스 B 신호 167 앞 50m: VTD 는 아직 tl_id=0 인데 `_tl_lines` 의 167 선이 앞에 있다.
    step() 이 넘기는 값과 같은 식으로 계산했을 때 거리가 나와야 한다(None 이면 고치기 전 동작)."""
    tl = {"7": [100.0, 1.5]}
    st = DrivingStack(route=straight_route(300.0), tl_stops_extra=tl,
                      stoplines_all=[[100.0, 0.0, 0.0]])
    s = _ego(v=10.0)
    s.x, s.y, s.tl_id = 50.0, 0.0, 0
    painted, _passed = st._tl_stop_dist(s)                                 # 대체 탐색은 50m 를 준다
    assert painted is not None and abs(painted - 50.0) < 1e-6
    assert st._tl_unseen_dist(50.0, painted) is None                       # 그대로 넘기면 죽는다(옛 버그)
    assert st._tl_unseen_dist(50.0, st._reported_tl_dist(s, painted)) == 50.0
