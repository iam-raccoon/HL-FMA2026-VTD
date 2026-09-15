"""종점 붙기 지시등 — 2026-09-09 6코스 스윕에서 4코스(A·B·G·H) 항목13 -3 의 원인 셋.

  A t=574.1  코너 출구에서 경로 차로변경 신호가 **한 프레임** L(`2,1,2`) -> 우측 선행이 끊겨 0.9초.
  B t=298.1  48km/h 로 시작점(90m)에 닿아 남은 6.5초 — 3.5초 선행을 기다리면 못 끝내니
             `_reach` 가 바로 움직였고 지시등과 이동이 같은 프레임(선행 1.2초).
  G t=440.0  off -1.25 -> -2.02 (오른쪽으로 가는 중) 인데 1초간 L.
  H t=395~398 R/L 이 10번 뒤집힘. off 0.00 -> -0.24(오른쪽) 로 가는데 L.
고친 것
  ① 시작점 PULLOVER_SIG_LEAD_S*속도 앞에서 우측을 **먼저** 켠다(`_po_presig`).
  ② 방향은 **이 프레임에 명령한 오프셋 변화**(cmd)를 먼저 본다 — 추정치(EMA 횡속도)는 늦다.
  ③ 방향 전환은 새 방향이 PO_DIR_HOLD_S 이어져야 한다(디바운스).
  ④ 행동층 좌측은 BEH_LEFT_MIN_S 이어져야 붙기 우측을 밀어낸다(한 프레임 튐 무시).
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from drive import DrivingStack, straight_route, TS_LEFT, TS_RIGHT      # noqa: E402

ROAD = dict(lane=-1, w=3.3, l=3.3, r=4.6, xl=0.0, xr=0.0, need=0.0, sig=0, j=0, jx=0, lim=13.889)


def _stack(n=400):
    return DrivingStack(route=straight_route(float(n)), lane_plan=[dict(ROAD)] * n)


# ---------------------------------------------------------------- ② 명령이 먼저다
def test_이_프레임에_오른쪽으로_명령했으면_우측이다():
    """H t=397.0: 횡속도 EMA 는 아직 0, squeeze 참 -> 예전엔 L. 명령은 오른쪽이었다."""
    assert DrivingStack._po_direction(delta=0.0, lat_v=0.0, squeeze=True, cmd=-0.2) == -1
    # G t=440.0: delta 는 작고 squeeze 참인데 오프셋은 -1.25 -> -2.02 로 가는 중
    assert DrivingStack._po_direction(delta=+0.1, lat_v=-0.1, squeeze=True, cmd=-0.1) == -1
    # 왼쪽으로 명령했으면(끝나는 차로 클램프) 좌측
    assert DrivingStack._po_direction(delta=0.0, lat_v=0.0, squeeze=False, cmd=+0.2) == 1
    # cmd 를 안 주면 예전 계약 그대로
    assert DrivingStack._po_direction(delta=0.0, lat_v=0.0, squeeze=True) == 1
    assert DrivingStack._po_direction(delta=-1.0, lat_v=0.0, squeeze=True) == -1


# ---------------------------------------------------------------- ① 선행 점등 구간
def test_시작점_앞_선행구간에서_우측을_먼저_켠다():
    """B: 13.4m/s 면 시작점 90m 앞 + 13.4*3.5 = 137m 부터 우측. 그 밖에서는 안 켠다."""
    st = _stack()
    st._pullover_offset(ROAD, 150.0, [], 0.0, 0.1, 13.4, bi=10)
    assert st._po_presig is False and st._po_on is False
    assert st._final_turn(turn_av=0, turn_beh=0, s_remain=150.0) == 0
    st._pullover_offset(ROAD, 130.0, [], 0.0, 0.1, 13.4, bi=10)
    assert st._po_presig is True and st._po_on is False
    assert st._final_turn(turn_av=0, turn_beh=0, s_remain=130.0) == TS_RIGHT
    # 천천히 오면 구간도 짧다(2m/s 하한): 90 + 2*3.5 = 97m
    st2 = _stack()
    st2._pullover_offset(ROAD, 100.0, [], 0.0, 0.1, 0.0, bi=10)
    assert st2._po_presig is False
    st2._pullover_offset(ROAD, 96.0, [], 0.0, 0.1, 0.0, bi=10)
    assert st2._po_presig is True


def test_선행구간에서_한_프레임_튀는_행동층_좌측은_무시한다():
    """A t=574.09: 코너 출구 `2,1,2`. 0.5초 이상 이어진 좌측만 붙기 우측을 밀어낸다."""
    st = _stack()
    st._po_presig = True
    st._beh_left_age = 0.1
    assert st._final_turn(turn_av=0, turn_beh=TS_LEFT, s_remain=120.0) == TS_RIGHT
    st._beh_left_age = 0.6                      # 진짜 경로 차로변경(연속) 이면 그대로 좌측
    assert st._final_turn(turn_av=0, turn_beh=TS_LEFT, s_remain=120.0) == TS_LEFT
    # 추월기는 여전히 최우선
    assert st._final_turn(turn_av=TS_LEFT, turn_beh=0, s_remain=120.0) == TS_LEFT


def test_붙는_중_방향_미정이면_우측을_유지한다():
    """예전엔 `_po_net_left == 0` 이고 종점 45m 밖이면 행동층 값으로 떨어져 우측이 끊겼다."""
    st = _stack()
    st._po_on = True
    st._po_net_left = 0
    assert st._final_turn(turn_av=0, turn_beh=0, s_remain=80.0) == TS_RIGHT
    st._beh_left_age = 0.0
    assert st._final_turn(turn_av=0, turn_beh=TS_LEFT, s_remain=80.0) == TS_RIGHT
    st._beh_left_age = 1.0
    assert st._final_turn(turn_av=0, turn_beh=TS_LEFT, s_remain=80.0) == TS_LEFT


# ---------------------------------------------------------------- ③ 디바운스
def test_방향은_PO_DIR_HOLD_S_이어져야_바뀐다():
    """H: 후보가 0.5초·0.7초씩 L 로 튀었다 — 0.8초 미만은 무시, 1초 이어지면 바뀐다."""
    st = _stack()
    seq = [-1] * 10 + [1] * 5 + [-1] * 10 + [1] * 10 + [1] * 5     # 0.1초 프레임
    st._po_direction = lambda *a, **k: seq.pop(0)
    st._po_direction_committed = lambda *a, **k: seq.pop(0)   # 시작 후엔 이쪽이 후보를 낸다
    seen = []
    s_remain = 80.0
    for _ in range(len(seq)):
        st._pullover_offset(ROAD, s_remain, [], 0.0, 0.1, 8.0, bi=10)
        seen.append(st._po_net_left)
        s_remain -= 0.8
    # 처음 -1 은 0.8초(8프레임) 뒤 채택, 0.5초짜리 L 은 무시, 1.0초 이어진 L 만 채택
    assert seen[9] == -1, seen
    assert all(x == -1 for x in seen[10:25]), seen[10:25]
    assert seen[-1] == 1, seen[-8:]
    assert sum(1 for a, b in zip(seen, seen[1:]) if a != b) == 2, seen


# ---------------------------------------------------------------- 종점 마무리 (2026-09-11 코스 H)
def test_목표가_도망가는_속도를_이동속도에_더한다():
    """코스 H 는 종점 60m 앞에서 **경로 자체가 왼쪽 차로로 옮겨 간다**(lane -2 -> -1,
    우측 여유 1.7 -> 4.7m). 그러면 '도로 오른쪽 끝' 목표가 1.08m/s 로 도망가는데 이동 속도가
    `|want-off|/t_go` 로만 잡혀 0.7m/s 에 묶였다 -> 차가 경로에 끌려 왼쪽으로 1.3m 밀렸다가
    막판에 오른쪽으로 돌아왔다. 사용자: "왼쪽으로 갔다가 오른쪽으로 가"."""
    import inspect
    from drive import DrivingStack
    src = inspect.getsource(DrivingStack._pullover_offset)
    assert "_want_v" in src and "_po_want_prev" in src
    # 목표가 1.08m/s 로 도망가면 이동속도도 그만큼은 나와야 한다(상한 1.2 안에서)
    st = DrivingStack(route=straight_route(300.0))
    st._po_want_prev, st._po_off = -1.00, -1.00
    want, dt = -1.108, 0.1
    t_go = 3.7
    _want_v = abs(want - st._po_want_prev) / dt
    rate = min(st.PULLOVER_RATE_MAX,
               max(st.PULLOVER_RATE, abs(want - st._po_off) / max(t_go - 0.8, 0.4) + _want_v))
    assert rate > 1.0, rate                      # 옛 식은 0.7 이었다


def test_붙기_구간에서는_가속하지_않는다():
    """실측 코스 H: 종점 86m 앞에 19.9km/h 로 들어와 45.9km/h 까지 가속한 뒤 마지막 30m 를
    3.8m/s^2 로 급제동했다. 붙는 중에 속도를 올릴 이유가 없다."""
    from drive import DrivingStack
    st = DrivingStack(route=straight_route(300.0))
    assert st.PULLOVER_HOLD_V == 5.6
    for speed, cap in ((5.5, 5.6), (12.0, 12.0), (0.0, 5.6)):
        assert min(13.9, max(speed, st.PULLOVER_HOLD_V)) == cap


def test_붙기는_한_방향_기동이라_왼쪽으로_되돌아가지_않는다():
    """실측 2026-09-11 사전주행2 종점 100m: 지도의 우측 여유 `r` 이 1.65 -> 8.2 -> 5.1 -> 11.85m
    로 널뛰어 목표가 좌우로 흔들렸고, 오프셋이 -1.4 -> **+1.1** -> -5.1 로 갈지자가 됐다.
    사용자: "왜이럼". 붙기는 커브로 오른쪽에 가는 일이다 — 시작한 뒤 왼쪽으로는 안 돌아간다.
    ⚠️ 도로가 좁아져 더는 오른쪽에 못 있을 때만 `here` 바닥이 왼쪽으로 밀어 준다."""
    import inspect
    from drive import DrivingStack
    src = inspect.getsource(DrivingStack._pullover_offset)
    i_mono = src.index("if self._po_started and want > self._po_off")
    i_floor = src.index("self._po_off = max(self._po_off, here)")
    assert i_mono < i_floor, "한 방향 규칙은 도로 끝 바닥보다 **먼저** 와야 한다"
    # 폭으로는 못 가른다 — 코스 A 종점은 6m 넘게 옮기는 것이 정답이라 상한을 두면 안 된다
    assert not hasattr(DrivingStack, "PULLOVER_MAX_OFF")


# ------------------------------------------------- 사전주행2 종점 재생 (2026-09-11 2차)
def _replay_pretest2():
    """실측 사전주행2 종점 130m 를 그 지도(`r` 1.6 -> 8.2 -> 5.1 -> 11.85m)로 되돌린다.

    반환: (마지막 오프셋, `_po_edge`, 프레임별 `_po_net_left`, 프레임별 오프셋)
    """
    import json
    from drive import DrivingStack
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "routes")
    route = [(p[0], p[1]) for p in json.load(open(os.path.join(base, "route_pretest_2.json")))["ego_route"]]
    plan = json.load(open(os.path.join(base, "route_pretest_2_lane.json")))["pts"]
    st = DrivingStack(route=route, lane_plan=plan)
    total, d_ego, dt = st.cum[-1], 0.0, 0.04
    nets, offs, off = [], [], 0.0
    for i, q in enumerate(plan):
        rem = total - st.cum[i]
        if rem > 130:
            continue
        speed = max(3.0, min(9.7, rem / 6.0))
        step = (st.cum[i] - st.cum[i - 1]) if i else 1.0
        for _ in range(max(1, int(step / (speed * dt)))):
            off = st._pullover_offset(q, rem, [], d_ego, dt, speed, bi=i)
            d_ego = 0.7 * d_ego + 0.3 * off          # 추종 지연
            nets.append(st._po_net_left)
            offs.append(off)
    return off, st._po_edge, nets, offs


def test_붙는_중에는_좌측_지시등이_켜지지_않는다():
    """실측 2026-09-11 사전주행2 t=114.1~115.9: 명령은 오른쪽(off -1.38 -> -2.30)인데
    **좌측이 1.9초** 켜졌다. 방향을 '원래 목표까지 남은 거리'로 쟀기 때문 — 한 방향 규칙이
    명령을 묶어 둔 사이 그 거리는 왼쪽을 가리킨다. 시작한 뒤에는 명령만 본다."""
    off, edge, nets, offs = _replay_pretest2()
    assert 1 not in nets, "붙는 중 좌측 지시등: %d 프레임" % sum(1 for n in nets if n == 1)
    assert -1 in nets, nets[:20]                     # 우측은 켜져야 한다
    # 오프셋도 한 방향(오른쪽)이어야 한다
    assert all(b <= a + 1e-6 for a, b in zip(offs, offs[1:])), "오프셋이 왼쪽으로 되돌아갔다"


def test_붙을_자리는_차선_위가_아니라_차로_중심이다():
    """목표를 고르는 순간 경로가 차로를 옮기는 중이면 `car_edge` 가 두 차로 사이다 —
    실측 6.76m(차로 중심 5.05 / 8.45m). 그대로 붙잡으면 **차선을 밟고 선다**."""
    off, edge, nets, offs = _replay_pretest2()
    r_goal, w = 11.85, 3.3                            # 사전주행2 종점(같은 방향 4차로)
    n = int(round((r_goal - w / 2.0) / w))
    centers = [r_goal - k * (r_goal - w / 2.0) / n for k in range(n + 1)]
    assert min(abs(edge - c) for c in centers) < 0.1, (edge, centers)
    assert edge < 6.5, edge                           # 옛 값 7.18 = 차선 위


def test_공식_6코스_종점은_후보가_하나뿐이라_스냅이_안_걸린다():
    """스냅은 같은 방향 차로가 3개 이상일 때만 의미가 있다. 공식 코스 종점 우측 여유는
    1.75~4.85m(차로 1~2개)라 `nn>=1` 이어도 후보가 종점 차로와 그 오른쪽 하나뿐이고,
    그 하나는 이미 `r_goal + pick` 이다 — 클램프가 이기는 경우엔 그보다 오른쪽이라 후보가 없다."""
    for r_goal, w in ((4.58, 2.96), (4.51, 3.16), (1.75, 3.5), (1.50, 3.0), (4.85, 2.89), (4.72, 3.4)):
        rr = r_goal - w / 2.0
        nn = int(round(rr / w)) if rr > 0 else 0
        assert nn <= 1, (r_goal, w, nn)


def test_서_있는_동안에는_붙기_오프셋이_안_움직인다():
    """실측 2026-09-11 사전주행2 신호15: 붙는 중 적색을 만나 15초 서 있는 사이 **명령만**
    -6.59 -> -6.80 으로 더 갔다. 차는 못 움직이니 그 차이가 빚으로 남고, 녹색에 출발하면서
    남은 6.6m 안에 비스듬히 갚는다 — 조향 -22.2°, 종점에서 도로와 20.6° 틀어져 섰다.
    옆으로 가려면 앞으로 가야 한다."""
    from drive import DrivingStack
    st = _stack()
    s_remain = 60.0
    for _ in range(30):                                # 먼저 달리면서 붙기를 시작시킨다
        st._pullover_offset(ROAD, s_remain, [], 0.0, 0.1, 8.0, bi=10)
        s_remain -= 0.8
    moving = st._po_off
    assert moving < -0.1, moving                       # 오른쪽으로 가는 중이어야 한다
    for _ in range(50):                                # 5초 정지(적색)
        off = st._pullover_offset(ROAD, s_remain, [], 0.0, 0.1, 0.0, bi=10)
    assert off == moving, (moving, off)                # 한 톨도 안 움직여야 한다
    st._pullover_offset(ROAD, s_remain, [], 0.0, 0.1, 8.0, bi=10)
    assert st._po_off < moving                         # 다시 굴러가면 이어서 간다


def test_정지_적립_금지는_공식코스_기울기를_건드리지_않는다():
    """상한 16.7°는 느슨하다 — 공식 6코스 붙기 기울기는 실측 최대 7.5°다."""
    from drive import DrivingStack
    assert DrivingStack.PULLOVER_SLOPE_MAX == 0.30
    import math
    assert math.degrees(math.atan(DrivingStack.PULLOVER_SLOPE_MAX)) > 15.0
