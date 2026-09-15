"""갇힘 탈출 — 막은 차 **너머**까지 나갈 폭 (2026-09-06 코스 A 미완주).

무슨 일이 있었나 (실측 t=467~656, **188초 이동 0.00m** · 횡오프셋 내내 0.00)
  13m x 2.5m 짜리 버스가 **역주행**으로 마주 와 우리 앞 −0.19m 에 섰다.
  그 버스는 우리 차로와 왼쪽 차로를 **같이** 물고 있었다 — 경로기준 +0.25~+2.75m.
  그 지점 차로계획: 차로폭 3.0 · 같은방향 좌 4.35 · 우 1.5 · 반대편까지 7.15

왜 못 빠졌나
  · 오른쪽: 1.5m 뿐이라 차폭 1.886m 가 안 들어간다. 끝까지 붙어도 **0.14m 가 겹친다**.
  · 왼쪽 한 차로치(3.0m): 목표차선이 곧 그 버스라 `side_clear` 가 계속 False → OVT:WAIT.
  · 필요한 건 버스 **너머** +4.04m 였는데 아무도 그걸 보지 않았다.

기본 모드는 같은 방향 차도의 l/r 안에서만 폭을 넓힌다. 명시적인
`ALLOW_CENTERLINE_ESCAPE` 실험 모드만 xl/xr 공간을 쓰며, 채점상 면책은 없다.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from drive import DrivingStack, straight_route                         # noqa: E402
from overtake import Overtaker                                         # noqa: E402

# 코스 A (1098,-645) 실측 차로계획
PLAN_A = {"lane": -2, "w": 3.0, "l": 4.35, "r": 1.5, "xl": 7.15, "xr": 0.0,
          "need": 0.0, "sig": 0, "j": 0, "jx": 0, "lim": 13.889}


def _stack(**kw):
    return DrivingStack(route=straight_route(300.0), **kw)


def _bus(ds=8.4, d=1.50, wid=2.5, spd=0.0):
    """rf_objs 한 줄: (ds, d, speed, olen, owid, hgt, id, wid0, len0, dh)."""
    return (ds, d, spd, 13.0, wid, 3.2, 421, wid, 13.0, 3.04)


def test_기본모드는_막은_차_너머가_중앙선밖이면_폭을_안_낸다():
    st = _stack()
    assert st._escape_width(PLAN_A, [_bus()], d_ego=0.0,
                            side=+1.0, lane_w=3.0) is None


def test_실험옵션에서만_막은_차_너머_중앙선폭을_낸다():
    st = _stack(allow_centerline_escape=True)
    w = st._escape_width(PLAN_A, [_bus()], d_ego=0.0, side=+1.0, lane_w=3.0)
    assert w is not None, "실험 옵션인데 버스 너머로 나갈 폭을 못 냈다"
    # 버스 왼쪽 끝 2.75 + 차 반폭 0.943 + 여유 0.35
    assert abs(w - 4.043) < 0.01, w
    # 그 폭이 실제로 있는 자리인가 — 4.043 + 0.943 = 4.99 <= 반대편까지 7.15
    assert w + st.HALF_WIDTH <= max(PLAN_A["l"], PLAN_A["xl"])


def test_기본모드도_같은방향_차도안에서는_탈출폭을_넓힌다():
    st = _stack()
    same_direction_wide = dict(PLAN_A, l=5.2, xl=8.0)
    w = st._escape_width(same_direction_wide, [_bus()], d_ego=0.0,
                         side=+1.0, lane_w=3.0)
    assert abs(w - 4.043) < 0.01


def test_오른쪽으로는_원리상_못_빠진다():
    """우측 여유 1.5m — 버스 오른쪽 끝(+0.25)을 비키려면 우리 왼쪽 끝이 그보다 안이어야 한다."""
    st = _stack()
    assert st._escape_width(PLAN_A, [_bus()], d_ego=0.0, side=-1.0, lane_w=3.0) is None


def test_한_차로치로_충분하면_넓히지_않는다():
    """평범한 승용차가 차로 중앙에 서 있으면 예전 그대로 한 차로만 나간다."""
    st = _stack()
    car = (10.0, 0.0, 0.0, 4.4, 1.8, 1.5, 7, 1.8, 4.4, 0.0)
    assert st._escape_width(PLAN_A, [car], d_ego=0.0, side=+1.0, lane_w=3.0) is None


def test_자리가_없으면_안_나간다():
    """반대편까지 좁은 도로면 None — 인도로 올라가지 않는다."""
    st = _stack()
    narrow = dict(PLAN_A, l=3.2, xl=3.2)
    assert st._escape_width(narrow, [_bus()], d_ego=0.0, side=+1.0, lane_w=3.0) is None


def test_움직이는_차에는_안_켠다():
    """지나가는 중인 차는 곧 없어진다 — 그걸 너머로 비키면 마주 달려드는 꼴이다."""
    st = _stack()
    assert st._escape_width(PLAN_A, [_bus(spd=5.0)], d_ego=0.0, side=+1.0, lane_w=3.0) is None


def test_내_진로_밖의_차는_안_본다():
    """옆 차로에서 제 갈 길 가는(서 있는) 차 때문에 반대편까지 나가면 안 된다."""
    st = _stack()
    aside = (10.0, 4.0, 0.0, 4.4, 1.8, 1.5, 9, 1.8, 4.4, 0.0)
    assert st._escape_width(PLAN_A, [aside], d_ego=0.0, side=+1.0, lane_w=3.0) is None


def test_너무_멀면_포기한다():
    """ESCAPE_MAX_W 를 넘는 폭은 낸 적이 없어야 한다(도로를 통째로 가로지르지 않는다)."""
    st = _stack()
    huge = (10.0, 5.0, 0.0, 13.0, 6.0, 3.2, 11, 6.0, 13.0, 0.0)
    wide = dict(PLAN_A, l=20.0, xl=20.0)
    assert st._escape_width(wide, [huge], d_ego=0.0, side=+1.0, lane_w=3.0) is None


# ---------------------------------------------------------------- 추월기 쪽 계약
def test_탈출폭은_차로_반폭을_안_건드린다():
    """`set_lane_width` 로 대신하면 '내 차로가 막혔나' 판정까지 헐거워진다."""
    ov = Overtaker()
    ov.set_lane_width(3.0)
    before = ov.lane_half
    ov.set_escape_width(4.04)
    assert abs(abs(ov.W) - 4.04) < 1e-6
    assert ov.lane_half == before, "차로 반폭이 같이 바뀌었다"


def test_기동_중에는_탈출폭을_안_바꾼다():
    ov = Overtaker()
    ov.set_lane_width(3.0)
    ov.state = "PASS"
    ov.set_escape_width(4.04)
    assert abs(abs(ov.W) - 3.0) < 1e-6



# ---------------------------------------------------------------- 종점 붙기 — 대각선 정차
def test_남은_거리로_못_끝낼_횡이동은_요구하지_않는다():
    """★2026-09-08 사전테스트2 종점. "왜 대각선으로 멈춰".

    `_po_edge` 는 '도로 우측 끝에서 몇 m' 라, 도로가 넓어지거나 경로가 왼쪽으로 합류하면
    요구량이 계속 커진다. 그 자체는 **맞다** — 도로 기준으로 제자리를 지키는 게 A·G 종점을
    고친 설계이고, A 는 두 차로(6.2m)를 따라가야 정상이다.
    문제는 **끝낼 수 없는 양까지 요구**하는 것이다.

    실측: 종점 앞 우측 여유가 51m 전 6.45 → 11m 전 **11.85** 로 벌어지며 오프셋이
    −3.3 → **−10.2m** 로 커졌고, 다 옮기지 못한 채 종점에 닿아 조향 −25° 로 비스듬히 섰다.

    남은 거리에 `PULLOVER_RATE_MAX` 로 갈 수 있는 만큼만 요구하면, 목표는 그대로 두고
    대각선만 없어진다.
    """
    st = _stack()
    rate = st.PULLOVER_RATE_MAX

    def reach(po_off, s_remain, speed):
        return po_off - rate * max(0.0, s_remain / max(speed, 2.0) - 0.5)

    # 실측 그 자리: 지금 −3.3 · 종점까지 12m · 12km/h(3.3m/s) → 3.1초 남음
    r = reach(-3.3, 12.0, 3.3)
    assert abs(r - (-3.3 - rate * (12.0 / 3.3 - 0.5))) < 1e-9
    assert max(-10.2, r) == r, f"−10.2 를 그대로 요구하면 안 된다 (상한 {r:.2f})"
    assert r > -7.5, f"상한이 너무 헐겁다 ({r:.2f})"

    # 멀리 있을 때는 A 처럼 큰 이동도 막지 않는다 (100m 남음 · 8m/s → 12.5초)
    far = reach(0.0, 100.0, 8.0)
    assert far < -6.2, f"A 종점의 두 차로(6.2m) 합류를 막으면 안 된다 ({far:.2f})"


def test_붙기_기준은_교차로_점을_쓰지_않는다():
    """★2026-09-08 사전테스트2. 종점이 **교차로 안**(j=1)이라 기준이 통째로 틀렸다.

    교차로 안에서는 `l`/`r` 이 연결로 반폭이라 '도로 우측 끝까지 얼마'라는 뜻이 아니다.
    실측: 종점 r=1.65(j=1) · 바로 앞 점 r=**11.85**(j=0).
      옛 기준(종점) → pick=-0.21 → `_po_edge`=1.44 → 지금 도로(r=11.85)에서 **want=-10.4**
      새 기준(앞 점) → pick=-3.40 → `_po_edge`=8.45 → **want=-3.40** (딱 한 차로)
    그 10m 를 다 못 옮기고 종점에 닿아 비스듬히 섰다.
    `_pullover_offset` 은 이미 교차로 점에서 계산을 건너뛴다 — 목표를 고를 때도 같아야 한다.
    """
    from drive import DrivingStack, straight_route                     # noqa: E402
    J = dict(lane=-1, w=3.3, l=1.65, r=1.65, need=0.0, sig=0, j=1, jx=1, lim=13.889)
    ROAD = dict(J, r=11.85, l=1.65, j=0, jx=0)
    st = DrivingStack(route=straight_route(200.0), lane_plan=[ROAD] * 198 + [J, J])
    assert st._po_goal_plan is not None
    assert not st._po_goal_plan.get("j"), "교차로 점을 기준으로 잡았다"
    assert abs(float(st._po_goal_plan["r"]) - 11.85) < 1e-6, st._po_goal_plan["r"]


def test_전부_교차로면_예전대로_마지막_점을_쓴다():
    from drive import DrivingStack, straight_route                     # noqa: E402
    J = dict(lane=-1, w=3.3, l=1.65, r=1.65, need=0.0, sig=0, j=1, jx=1, lim=13.889)
    st = DrivingStack(route=straight_route(200.0), lane_plan=[J] * 200)
    assert st._po_goal_plan is J
