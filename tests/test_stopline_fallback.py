"""모르는 신호 id 일 때 **도색 정지선**으로 메우는지.

왜 필요한가(실측 2026-09-04): 로그 48개에 나온 신호 60종 중 **80·82** 가 맵의
controller 에 없다(tl80 은 2895 프레임). 예전 `_tl_stop_dist` 는 그때 (None, False) 를
줬고, 그러면 적신호가 `RED_STOP_BLIND` -> **그 자리 정지**다. 교차로 한복판에 선다.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from drive import DrivingStack                                      # noqa: E402

HERE = os.path.join(os.path.dirname(__file__), "..")


class S:
    def __init__(self, x, y, heading, tl_id):
        self.x, self.y, self.heading, self.tl_id = x, y, heading, tl_id


def _at(line, back, side=0.0):
    """정지선에서 진행방향으로 back[m] 뒤, 옆으로 side[m] 인 지점의 상태."""
    x, y, h = line
    return S(x - back * math.cos(h) - side * math.sin(h),
             y - back * math.sin(h) + side * math.cos(h), h, 12345)


LINE = (100.0, 50.0, 0.7)


def _stack(**kw):
    return DrivingStack(**kw)


def test_없으면_예전그대로():
    d = _stack()
    assert d._tl_stop_dist(_at(LINE, 10.0)) == (None, False)


def test_앞의_도색선까지_거리를_준다():
    d = _stack(stoplines_all=[list(LINE)])
    dist, passed = d._tl_stop_dist(_at(LINE, 12.0))
    assert passed is False
    assert abs(dist - 12.0) < 1e-6


def test_가장_가까운_것을_고른다():
    far = (LINE[0] + 20 * math.cos(LINE[2]), LINE[1] + 20 * math.sin(LINE[2]), LINE[2])
    d = _stack(stoplines_all=[list(far), list(LINE)])
    dist, _ = d._tl_stop_dist(_at(LINE, 8.0))
    assert abs(dist - 8.0) < 1e-6


def test_이미_지난_선이면_지났다고_한다():
    """★핵심. (None, False) 로 주면 behavior 가 '모르는 신호'로 보고 교차로 안에 선다."""
    d = _stack(stoplines_all=[list(LINE)])
    assert d._tl_stop_dist(_at(LINE, -5.0)) == (None, True)


def test_너무_멀리_지났으면_모른다고_한다():
    d = _stack(stoplines_all=[list(LINE)])
    assert d._tl_stop_dist(_at(LINE, -(DrivingStack.TLP_BACK + 5.0))) == (None, False)


def test_반대방향_선은_무시():
    rev = (LINE[0], LINE[1], LINE[2] + math.pi)
    d = _stack(stoplines_all=[list(rev)])
    assert d._tl_stop_dist(_at(LINE, 10.0)) == (None, False)


def test_옆_차로_선은_무시():
    d = _stack(stoplines_all=[list(LINE)])
    assert d._tl_stop_dist(_at(LINE, 10.0, side=DrivingStack.TLP_LAT + 2.0)) == (None, False)


def test_아는_신호는_폴백을_안_탄다():
    """DB 에 있는 신호는 예전 경로 그대로여야 한다 — 도색선이 있어도 무시."""
    d = _stack(stoplines_all=[list(LINE)], tl_stops_extra={7: [LINE[0], LINE[1]]})
    st = _at(LINE, 10.0)
    st.tl_id = 7
    dist, _ = d._tl_stop_dist(st)
    assert abs(dist - 10.0) < 1e-6


def test_실제_DB_가_읽히고_고아정지선을_포함한다():
    a = json.load(open(os.path.join(HERE, "routes", "stoplines_all.json"),
                      encoding="utf-8"))["stoplines"]
    b = json.load(open(os.path.join(HERE, "routes", "stoplines.json"),
                      encoding="utf-8"))["stoplines"]
    assert len(a) == 710, len(a)
    assert set(map(tuple, b)) <= set(map(tuple, a)), "고아 정지선은 전체의 부분집합이어야"


def test_로그에_나온_신호가_DB_에_있나():
    """옛날엔 tl 80·82 가 없었다. 없는 건 이제 폴백이 받지만, **개수가 늘면** 알아야 한다."""
    tl = json.load(open(os.path.join(HERE, "routes", "tl_map_livinglab.json"),
                        encoding="utf-8"))
    assert len(tl) == 214, len(tl)
    assert "80" not in tl and "82" not in tl, "맵에 생겼으면 폴백 주석을 고칠 것"


# ---------------- 고아 정지선은 '양보 표시가 붙은 것' 만 (2026-09-05, 안 A)
def test_고아_정지선은_양보표시_또는_보호구역_안만_남는다():
    """★2026-09-05 에 106 -> 2 개로 줄인 계약.

    근거(법):
      · **정지선(노면표시 530) 자체에는 정지의무가 없다.** 대법원 전원합의체가
        일시정지 표지(227·521)와 구분했다 — 530 은 "정지를 해야 할 경우 정지해야 할
        **지점**을 표시하는 것으로서 그 표시 자체에 의하여 정지의무가 있음을
        표시하는 것은 아니다".
      · **이 맵에 일시정지 노면표시·표지는 0개다.** 양보(Rm_Give_Way) 23개뿐이다.
      · 대회 채점 15개 항목에 '정지선 미준수' 가 없다(7·9 는 신호등 전용,
        12 는 '횡단보도 **위** 정지' 감점).
    비용: 코스당 20~160초, 주최측 사전테스트 경로 2번에서도 40초.

    ⚠️ 보행자 보호는 이것과 **무관하게** 계속 돈다 —
       제27조① = `YIELD_PED`(보호구역 무관) · 제27조⑦ = crosswalks 의 `zone`.
    """
    sl = json.load(open(os.path.join(HERE, "routes", "stoplines.json"),
                       encoding="utf-8"))["stoplines"]
    # ★2026-09-05 저녁: **보호구역 안**의 고아 정지선을 되살렸다(사용자 결정 "저건 해야
    #   하는 거잖아"). 2(양보 표시) + 보호구역 안 = 28. '보호구역 안' 은 횡단보도 zone 과
    #   같은 정의(build_lane_plan.in_zone + ZONE_CW_REACH)다.
    # ★2026-09-06: 보호구역 모델이 **방향별 + 표시 없는 팔 40m**(zone_extent 주석)로 바뀌며
    #   28 -> 43. 늘어난 15개는 전부 보호구역 교차로에 붙은 표시 없는 팔 위의 것이다.
    #   실주행 6코스 경로 위에 새로 걸리는 건 H 의 1개뿐(경로 3m 안 기준).
    # ★2026-09-06 (2): 보호구역 교차로 판정을 '표시 창이 닿는 교차로'로 좁히고(junction 89 제외)
    #   팔은 90m 로 늘리며 43 -> 34. 실주행 경로 위 변화는 H 1개 빠짐뿐.
    assert len(sl) == 34, f"{len(sl)}개 — 양보 표시도 보호구역도 아닌 정지선이 들어왔거나 빠졌다"


def test_보행자_보호는_따로_돈다():
    """고아 정지선을 줄여도 제27조①·⑦ 은 건드리지 않았다는 가드."""
    src = open(os.path.join(HERE, "src", "behavior.py"), encoding="utf-8").read()
    assert "YIELD_PED" in src, "제27조① 보행자 양보가 사라졌다"
    drv = open(os.path.join(HERE, "src", "drive.py"), encoding="utf-8").read()
    assert 'c["zone"]' in drv, "제27조⑦ 보호구역 횡단보도 판정이 사라졌다"
