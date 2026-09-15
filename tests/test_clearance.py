"""차체 실여유(SAT) 유닛테스트.

이 값이 틀리면 채점기가 없는 충돌을 만들거나(오탐) 진짜 충돌을 놓친다.
실제로 v5 연료통을 밟고도 '충돌 아님'으로 통과시킨 적이 있어서 넣은 지표다.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from run_logger import body_clearance, EGO_HALF_W, EGO_FRONT   # noqa: E402
from vtd_io import State, Obj                                   # noqa: E402

CAN = dict(L=0.15, W=0.46, H=0.61)
CAR = dict(L=4.39, W=1.80, H=1.40)


def place(fx, fy, dh=0.0, **dim):
    """ego 좌표계 (fx, fy) 에 dh[rad] 만큼 돌아간 물체를 놓는다."""
    s = State()                       # ego 는 원점, heading 0
    return body_clearance(s, Obj(1, fx, fy, 0.0, dh, 0.0, dim["L"], dim["W"], dim["H"]))


def test_멀리_있으면_거리만큼_양수():
    assert abs(place(10.0, 0.0, **CAN) - (10.0 - EGO_FRONT - CAN["L"] / 2)) < 0.05


def test_차체_아래면_음수():
    assert place(1.4, 0.0, **CAN) < 0


def test_비켜서_지나가면_양수():
    assert place(1.4, -1.62, **CAN) > 0.3


def test_나란히_선_차는_반폭_합만큼_필요():
    need = EGO_HALF_W + CAR["W"] / 2                 # 1.843
    assert place(1.4, -(need - 0.1), **CAR) < 0      # 조금 모자라면 접촉
    assert place(1.4, -(need + 0.1), **CAR) > 0      # 조금 넉넉하면 통과


def test_비스듬한_차는_모서리가_튀어나온다():
    """차선변경 중인 차는 정렬됐을 때보다 더 넓은 자리를 쓴다."""
    straight = place(1.4, -2.5, 0.0, **CAR)
    tilted = place(1.4, -2.5, math.radians(20), **CAR)
    assert tilted < straight


def test_대각선으로_떨어진_차는_접촉이_아니다():
    """축정렬 근사만 쓰면 '앞으로도 겹치고 옆으로도 겹친다'고 오판하기 쉬운 배치."""
    assert place(6.0, -2.6, math.radians(30), **CAR) > 0


def test_뒤에_있는_차도_본다():
    assert place(-8.0, 0.0, **CAR) > 0
    assert place(-1.0, 0.0, **CAR) < 0


def test_경로기준_외접크기가_기울어진_차의_횡폭을_키운다():
    """안전망은 물체를 '경로와 나란한 상자'로 재는데, 굽은 길에선 그게 과소평가다.

    2026-08-16 EV_LEADBRAKE 실측: 곡선 위 정지차를 계산 여유 +0.38m 로 '통과 가능'
    이라 보고 지나가려다 실제 차체 여유 -0.06m 로 파고들어 **접촉 2515프레임**.
    물체가 경로에 14° 틀어져 있었고 그 각도가 만든 횡폭 차이가 정확히 0.5m 였다.
    """
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
    from drive import route_extent

    class O:
        pass
    o = O(); o.length, o.width = 4.4, 1.9

    o.heading = 0.0
    assert route_extent(o, 0.0) == (4.4, 1.9)              # 나란하면 그대로

    o.heading = math.radians(14)
    lon, lat = route_extent(o, 0.0)
    assert 2.85 < lat < 2.95, lat                          # 횡 반폭 0.95 -> 1.45
    # 그 0.5m 가 '통과 가능(+0.38)' 을 '접촉(-0.06)' 으로 뒤집는다
    half_ego, fy = 0.94, 2.34
    assert fy - half_ego - 1.9 / 2 > 0.35                  # 옛 계산: 지나가도 된다
    assert fy - half_ego - lat / 2 < 0.0                   # 새 계산: 닿는다

    o.heading = math.radians(90)
    lon, lat = route_extent(o, 0.0)                        # 직각이면 길이/폭이 뒤바뀐다
    assert abs(lon - 1.9) < 1e-9 and abs(lat - 4.4) < 1e-9

    o.heading = math.radians(-14)                          # 반대로 틀어져도 같다
    assert route_extent(o, 0.0) == route_extent_at(o, 14)


def test_옆으로_빠지는_중엔_지금자리로_세우지_않는다():
    """실측 2026-08-26 EV_COMBO_PASSPED: 보행자를 피해 우측 -6.34m 로 비껴 나가다
    **-5.41m 에서** 안전망이 완전정지를 줬다. 그 판정은 물체를 경로축 외접상자로 보고
    종방향 간격을 무시한다 — 똑바로 갈 때는 맞지만 비스듬히 빠지는 중엔 틀린다.
    **상자 대 상자 실여유는 +2.93m** 로 닿지도 않았는데 굳었고, 후진이 없어 260초 미완주.

    실측값(로그 rw·dh 에서 복원): 경로기준 ds 7.47 · d +2.22 · 자차 요 -25.3° ·
    원본 4.4x1.8 · 경로기준 방위 -14.3° · 남은 횡이동 -0.93m.
    이 값으로 SAT 를 계산하면 2.95 가 나온다 — 로그의 2.93 과 일치한다."""
    import math as _m
    from behavior import Behavior
    b = Behavior()
    yaw = _m.radians(-25.3)
    # (ds, d, spd, 경로축길이, 경로축폭, 높이, id, 원본폭, 원본길이, 경로기준방위)
    car = (7.47, 2.22, 0.0, 4.66, 2.78, 1.4, 1, 1.8, 4.4, _m.radians(-14.3))
    assert b._sweep_ok(car, yaw, -0.93, b.body_gap_min) is True, "안 닿는데 세운다"
    # ★기동 중이 아니면(남은 횡이동 없음) 옛 동작 그대로 = 정지
    assert b._sweep_ok(car, yaw, 0.0, b.body_gap_min) is False, "가만히 있는데 서행 허용"
    # ★**지금 이미 닿을 만큼 가까우면** 절대 허용 안 한다
    close = (4.9, 0.9, 0.0, 4.66, 2.78, 1.4, 2, 1.8, 4.4, 0.0)
    assert b._sweep_ok(close, 0.0, -2.0, b.body_gap_min) is False, "닿기 직전인데 기어간다"
    # ★★다 빠져도 옆을 못 지나가면 허용 안 한다 — 옛 NARROW_CREEP 재발 방지.
    #   내 차로 한복판의 차 앞에서 0.5m 만 비켜서는 절대 못 지난다.
    ahead = (8.0, 0.0, 0.0, 4.4, 1.8, 1.4, 3, 1.8, 4.4, 0.0)
    assert b._sweep_ok(ahead, 0.0, -0.5, b.body_gap_min) is False, "다 빠져도 막히는데 기어간다"
    # ★같은 차라도 **한 차로를 통째로** 비키면 지나갈 수 있다
    assert b._sweep_ok(ahead, 0.0, -3.2, b.body_gap_min) is True, "한 차로 비켰는데 세운다"


def route_extent_at(o, deg):
    from drive import route_extent
    saved = o.heading
    o.heading = math.radians(deg)
    out = route_extent(o, 0.0)
    o.heading = saved
    return out


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn(); print(f"  ✅ {name}")
            except AssertionError as e:
                fails += 1; print(f"  ❌ {name}: {e}")
    print("실패 없음" if not fails else f"{fails}건 실패")
    sys.exit(1 if fails else 0)
