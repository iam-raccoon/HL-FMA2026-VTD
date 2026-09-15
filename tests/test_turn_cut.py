"""회전 중 접촉 가드(TURN_CUT) — (1456,939) 무신호 좌회전에서 3번 스친 자리.

무엇이 문제였나
  `_cpa_conflict` 는 두 차가 **모두 직진**한다고 보고 최근접을 잡는다. 그래서 방위차가
  작은 차(=나란히 가는 차)를 `cpa_parallel_deg`(12°)로 통째로 건너뛴다. 똑바로 갈 때는
  맞다 — 옆 차로 차마다 급제동하지 않으려면 필요한 예외다.
  하지만 **내가 도는 중이면 그 차를 내가 가로지른다.**

실측 (연결로 3164, 길이 95m, 73° 좌회전)
  2026-08-26 코스 E 여유 -0.65m · 코스 H -0.19m · 2026-09-06 코스 E -0.19m
  2026-09-06 로그: t=146.0 에 이미 **뒤 27.6m·왼쪽 3.5m 에 52km/h** 차가 보였는데
  방위차 1° 라 CPA 가 건너뛰었고, 자차가 25° 돌아 12° 를 넘긴 t=149.9 에야 CPA_BRAKE.
  접촉은 t=150.1 — **0.2초 전**이다.

고친 뒤(같은 로그를 그대로 흘려서 잰 값)
  요레이트 원호만 쓰면  t=149.33 (0.77초 전)  — 돌기 시작해야 켜지니 늦다
  **경로를 주면      t=146.37 (3.73초 전, 경로 31m 앞)**
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from behavior import Behavior                                          # noqa: E402
from vtd_io import Obj, State                                          # noqa: E402


def _ego(v=8.0):
    s = State()
    s.speed = v
    s.heading = 0.0
    s.t = 0.0
    return s


def _arc_path(radius=25.0, v=8.0, horizon=4.0, step=0.25):
    """왼쪽으로 도는 경로 [(t, x, y, heading)] — 자차 기준."""
    out, prev = [], None
    t = 0.0
    while t < horizon - 1e-9:
        t += step
        th = v * t / radius
        x, y = radius * math.sin(th), radius * (1.0 - math.cos(th))
        h = 0.0 if prev is None else math.atan2(y - prev[1], x - prev[0])
        out.append((t, x, y, h))
        prev = (x, y)
    return out


def _straight_path(v=8.0, horizon=4.0, step=0.25):
    out, prev = [], None
    t = 0.0
    while t < horizon - 1e-9:
        t += step
        x, y = v * t, 0.0
        h = 0.0 if prev is None else math.atan2(y - prev[1], x - prev[0])
        out.append((t, x, y, h))
        prev = (x, y)
    return out


# 뒤 15m·왼쪽 3.6m 에서 52km/h 로 따라오는 직진차 (실측한 그 차)
def _overtaker(fx=-15.0, fy=3.6, v=14.4):
    return Obj(1, fx, fy, 0.0, 0.0, v, 4.57, 1.8, 1.5)


def test_좌회전_경로면_뒤에서_오는_직진차를_잡는다():
    b = Behavior()
    e = _ego(v=8.0)
    e.objects = [_overtaker()]
    d = b._turn_cut_conflict(e, path_ahead=_arc_path())
    assert d is not None, "회전 경로가 그 차를 가로지르는데 못 잡았다"
    assert 5.0 < d < 40.0, d


def test_직진_경로면_옆으로_지나가는_차에_안_걸린다():
    """★이게 없으면 옆 차로 차마다 급제동한다(2026-08-28 사용자 지적과 같은 실패)."""
    b = Behavior()
    e = _ego(v=8.0)
    e.objects = [_overtaker()]
    assert b._turn_cut_conflict(e, path_ahead=_straight_path()) is None


def test_회전해도_충분히_비껴가면_안_걸린다():
    b = Behavior()
    e = _ego(v=8.0)
    e.objects = [_overtaker(fx=-15.0, fy=9.0)]      # 두 차로 밖
    assert b._turn_cut_conflict(e, path_ahead=_arc_path()) is None


def test_사람은_여기서_안_본다():
    """보행자는 YIELD_PED 담당 — 여기서 또 보면 규칙이 겹친다."""
    b = Behavior()
    e = _ego(v=8.0)
    e.objects = [Obj(2, -15.0, 3.6, 0.0, 0.0, 3.0, 2.0, 0.6, 1.7)]   # VTD 보행자 치수
    assert b._turn_cut_conflict(e, path_ahead=_arc_path()) is None


def test_느린_물체는_안_본다():
    b = Behavior()
    e = _ego(v=8.0)
    e.objects = [_overtaker(v=1.0)]                  # cpa_min_v 아래 = 정지차·서행차
    assert b._turn_cut_conflict(e, path_ahead=_arc_path()) is None


# ---------------------------------------------------------------- 경로가 없을 때(폴백)
def test_경로가_없으면_요레이트로_대신_본다():
    b = Behavior()
    e = _ego(v=8.0)
    e.objects = [_overtaker(fx=-8.0)]
    b._yaw_rps = 8.0 / 25.0                          # R=25m 로 도는 중
    assert b._turn_cut_conflict(e) is not None


def test_차로변경_정도로는_안_켜진다():
    """차로변경은 2~3°/s 다. 문턱(7°/s) 아래면 이 가드는 통째로 꺼져 있어야 한다."""
    b = Behavior()
    e = _ego(v=8.0)
    e.objects = [_overtaker(fx=-8.0)]
    b._yaw_rps = math.radians(3.0)
    assert b._turn_cut_conflict(e) is None


# ---------------------------------------------------------------- plan() 까지
def test_plan_이_TURN_CUT_으로_속도를_깎는다():
    b = Behavior()
    e = _ego(v=8.0)
    e.objects = [_overtaker()]
    v, _off, _t, reason = b.plan(e, 13.9, path_ahead=_arc_path())
    assert reason == "TURN_CUT", reason
    assert v < 13.9, v


def test_plan_은_경로가_없으면_예전과_같다():
    """path_ahead 를 안 주면(테스트·옛 호출) 동작이 바뀌면 안 된다."""
    b = Behavior()
    e = _ego(v=8.0)
    e.objects = [_overtaker()]
    v, _off, _t, reason = b.plan(e, 13.9)
    assert reason != "TURN_CUT", reason


# ---------------------------------------------------------------- 종점 붙기 지시등 우선순위
def test_붙는_중에는_붙기가_지시등을_잡는다():
    """★2026-09-06 코스 G 항목13 −3. 실측 t=506.8~509.4 지시등이 `2,1,1,1,1,2,2,2,2,2,2`.

    예전엔 붙기가 `turn == 0` 일 때만 지시등을 잡아서, 경로에 박힌 차로변경 신호(왼쪽)와
    프레임 단위로 번갈아 켜졌다. 오른쪽이 **연속**으로 켜진 건 0.8초뿐이라
    '차로변경 선행점등 0.8초 < 3.0초' 로 −3. 붙는 중이면 경로의 차로변경은 어차피 안 한다.
    """
    from drive import DrivingStack, straight_route, TS_LEFT, TS_RIGHT   # noqa: E402
    st = DrivingStack(route=straight_route(200.0))
    st._po_on = True
    st._po_net_left = -1                       # 오른쪽으로 붙는 중
    # 경로 차로변경이 왼쪽을 요구해도(turn_beh=TS_LEFT) 붙기가 이긴다
    assert st._final_turn(turn_av=0, turn_beh=TS_LEFT, s_remain=20.0) == TS_RIGHT
    # 추월기(실제로 하는 기동)는 그대로 우선한다
    assert st._final_turn(turn_av=TS_LEFT, turn_beh=0, s_remain=20.0) == TS_LEFT


# ---------------------------------------------------------------- 붙기 지시등 방향
def test_붙는_중_지시등은_지금_가는_쪽을_따른다():
    """★2026-09-08 사전테스트1 종점(792.8,484.7). "오른쪽으로 붙는데 왼쪽 깜빡이".

    종점 앞 54m 에서 도로 우측 여유가 **4.55m -> 1.44m** 로 좁아진다. 그래서
    `squeeze`(앞에서 안쪽으로 밀림)가 6.5초 내내 참이었는데, 예전 판정은 그걸 먼저 봐서
    **오른쪽으로 붙는 동안 좌측**을 켰다. 실측 그 구간 `off` 는 0.00 -> -1.28(오른쪽)이었다.
    """
    from drive import DrivingStack                                     # noqa: E402
    # 오른쪽으로 붙는 중(횡속도 -0.7) + 앞에서 좁아짐 -> **우측**이어야 한다
    assert DrivingStack._po_direction(delta=+3.0, lat_v=-0.7, squeeze=True) == -1
    # 남은 이동이 오른쪽이면 squeeze 와 무관하게 우측
    assert DrivingStack._po_direction(delta=-1.0, lat_v=0.0, squeeze=True) == -1


def test_아무_데도_안_갈_때만_앞일을_예고한다():
    """`squeeze` 는 지금 정지 상태일 때 '곧 안쪽으로 밀린다'를 알리는 값이다."""
    from drive import DrivingStack                                     # noqa: E402
    assert DrivingStack._po_direction(delta=0.0, lat_v=0.0, squeeze=True) == 1
    assert DrivingStack._po_direction(delta=0.0, lat_v=0.0, squeeze=False) == 0
    # 왼쪽으로 가야 하면(끝나는 차로에서 안쪽으로) 그대로 좌측
    assert DrivingStack._po_direction(delta=+1.0, lat_v=+0.5, squeeze=False) == 1


def test_이미_가장자리면_밀린다고_보지_않는다():
    """★단위 섞임. 실측 사전테스트1 종점: 목표 1.443m · 앞 도로 1.44m — 밀릴 데가 없다.

    옛 식은 왼쪽에 **오프셋**(r-반폭-0.15), 오른쪽에 **가장자리 거리**(po_edge)를 놓고 비교해
    차 반폭만큼 늘 기울어 거의 언제나 참이었다(`0.347 < 1.143`).
    """
    from drive import DrivingStack                                     # noqa: E402
    assert DrivingStack._po_squeeze(r_ahead=1.44, po_edge=1.443) is False
    # 넓은 자리에서 목표를 크게 잡아 둔 뒤 도로가 좁아지면 — 그때는 진짜로 밀린다
    assert DrivingStack._po_squeeze(r_ahead=1.44, po_edge=4.28) is True
    assert DrivingStack._po_squeeze(r_ahead=None, po_edge=4.28) is False
