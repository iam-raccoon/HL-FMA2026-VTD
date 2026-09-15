"""route_frame / control / vtd_io 유닛테스트 (소켓·시뮬 없이 돌아감)."""
import math
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from route_frame import RouteFrame, offset_path      # noqa: E402
from control import LongPI, PurePursuit              # noqa: E402
from vtd_io import (DATA_SIZE, CTRL_SIZE, build_ctrl, pack_data, parse_data,  # noqa: E402
                    Obj, State, VTDLink)


def _arc_route(r=50.0, n=120):
    """반경 50m 곡선 경로 — 곡률 오염 검증용."""
    return [(r * math.sin(i * 0.02), r - r * math.cos(i * 0.02)) for i in range(n)]


def test_직선경로_투영():
    rf = RouteFrame([(i, 0.0) for i in range(50)])
    s, d, _ = rf.project(10.0, 2.0)
    assert abs(s - 10.0) < 0.1
    assert abs(d - 2.0) < 0.1, f"왼쪽(+) 규약 위반: d={d}"


def test_곡선에서_같은차선_차는_d가_0에_가깝다():
    """★ego 직교좌표 fy는 곡률에 오염된다. 경로기준 d는 그렇지 않아야 한다."""
    route = _arc_route()
    rf = RouteFrame(route)
    ahead = route[60]                                   # 경로 위(=같은 차선) 60번째 점
    _, d, _ = rf.project(ahead[0], ahead[1])
    assert abs(d) < 0.2, f"경로 위 점인데 d={d:.2f}"


def test_offset_path_는_좌측으로_민다():
    p = offset_path([(i, 0.0) for i in range(10)], 3.5)
    assert abs(p[5][1] - 3.5) < 0.05


def test_point_at_은_누적거리를_경로좌표로_바꾼다():
    rf = RouteFrame([(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)])
    assert rf.point_at(0.0) == (0.0, 0.0)
    assert rf.point_at(3.0) == (3.0, 0.0)
    x, y = rf.point_at(5.0)
    assert abs(x - 3.0) < 1e-9 and abs(y - 2.0) < 1e-9
    assert rf.point_at(7.0) == (3.0, 4.0)


def test_point_at_은_경로밖이면_none():
    rf = RouteFrame([(0.0, 0.0), (10.0, 0.0)])
    assert rf.point_at(-0.1) is None
    assert rf.point_at(10.1) is None
    assert RouteFrame([]).point_at(0.0) is None


def test_제동게인이_가속보다_크다():
    """앞차 급정지시 -1.5m/s²밖에 안 나와 추돌하던 버그(2026-08-14)."""
    pi = LongPI()
    assert pi.kp_brake > pi.kp
    a = pi.accel(v_target=2.0, v_cur=8.0, dt=0.05)
    assert a < -3.0, f"강한 제동이 나와야 함 (a={a:.2f})"


def test_purepursuit_은_좌측경로에_좌조향():
    pp = PurePursuit()
    path = [(i, 2.0) for i in range(30)]
    steer = pp.steer(0.0, 0.0, 0.0, 5.0, path)
    assert steer > 0, f"좌측 경로엔 좌조향(+) 이어야 함 ({steer})"


def test_패킷_크기_규격():
    assert DATA_SIZE == 1109 and CTRL_SIZE == 9
    assert len(build_ctrl(0.1, -1.0, 1)) == 9


def test_패킷_왕복():
    objs = [Obj(7, 10.0, -2.0, 0.0, 0.1, 5.5, 4.4, 1.8, 1.5)]
    raw = pack_data(1.0, 2.0, 0.0, 0.5, 0.0, 0.0, objs, tl_id=30, tl_state=1)
    st = parse_data(raw)
    assert abs(st.x - 1.0) < 1e-3 and st.tl_id == 30 and st.tl_state == 1
    assert len(st.objects) == 1 and st.objects[0].id == 7


def _drive(link, v, dts, step=None, t0=1000.0):
    """v[m/s] 등속 주행. dts=수신시각 간격, step=한 프레임에 실제로 전진한 시간.

    step을 주면 '위치는 일정한 sim step으로 오는데 수신 시각만 흔들리는'
    실제 상황을 재현한다(dt 지터가 그대로 속도 잡음이 되는 케이스).
    """
    st = State(); t = t0; x = 0.0
    link._update_speed(st, t)                 # 첫 프레임 = 기준점
    raws, smooth = [], []
    for dt in dts:
        t += dt; x += v * (step if step else dt)
        st = State(x=x)
        link._update_speed(st, t)
        raws.append(st.speed_raw); smooth.append(st.speed)
    return raws, smooth


def test_등속주행이면_참값으로_수렴():
    raws, sm = _drive(VTDLink(), 8.0, [0.05] * 60)
    assert abs(sm[-1] - 8.0) < 0.05, sm[-1]


def _naive(v, dts, step):
    """비교군: 창 없이 프레임마다 그냥 미분했을 때의 값."""
    return [v * step / dt for dt in dts]


def test_dt지터에서도_속도가_참값을_지킨다():
    """20Hz 고정 sim step인데 수신 시각이 ±40% 흔들리는 상황(네트워크/OS 지터).
    단순 미분이면 프레임마다 5.7~13.3m/s 로 널뛴다."""
    jitter = [0.03, 0.07, 0.035, 0.065, 0.05] * 12
    _, sm = _drive(VTDLink(), 8.0, jitter, step=0.05)
    s = sm[20:]                               # 초기 수렴구간 제외
    naive = _naive(8.0, jitter, 0.05)[20:]
    assert max(naive) - min(naive) > 5.0      # 비교군은 실제로 크게 흔들린다(전제 확인)
    assert max(s) - min(s) < 1.0, (max(s), min(s))
    assert abs(sum(s) / len(s) - 8.0) < 0.3   # 평균은 참값


def test_리스폰_점프는_감지하고_필터를_리셋():
    link = VTDLink()
    _drive(link, 8.0, [0.05] * 40)
    st = State(x=500.0, y=300.0)               # 순간이동
    link._update_speed(st, 1000.0 + 40 * 0.05 + 0.05)
    assert st.respawned and st.speed == 0.0



def test_패킷_뭉침에도_속도가_안_튄다():
    """실측 2026-08-15(v3): 위치는 sim 클럭으로 전진하는데 시간은 벽시계로 잰다.
    패킷이 뭉쳐 오면 한 프레임이 0.020s 만에 도착해 16.9m/s 로 읽혔다(참값 8.3).

    ⚠️ 뭉침은 **총 이동거리를 늘리지 않는다** — 빨리 온 만큼 다음이 늦게 온다.
       (거리까지 2배로 주면 그냥 '진짜 빨리 달리는 차'라 모델이 틀린다)
    """
    link = VTDLink()
    st = State(); t = 0.0; x = 0.0
    link._update_speed(st, t)
    peak = 0.0
    for i in range(120):
        x += 8.0 * 0.040                   # sim 은 항상 일정하게 전진
        t += 0.020 if i % 7 == 3 else (0.040 + (0.020 / 6 if i % 7 else 0))
        st = State(x=x)
        link._update_speed(st, t)
        if i > 30:
            peak = max(peak, st.speed)
    assert peak < 8.0 * 1.10, f"뭉침에 {peak:.2f} m/s 까지 튐"


def test_헤딩_스냅도_리스폰으로_잡는다():
    """실측 2026-08-15: 추월 중 리스폰이 1.59m 이동 + 헤딩 20°(250°/s) 스냅으로 왔다.
    거리 조건(2.5m)만 보던 감지기가 이걸 놓쳐 제어기 상태가 초기화되지 않았다."""
    link = VTDLink()
    st = State(); link._update_speed(st, 0.0)
    st = State(x=0.4); link._update_speed(st, 0.04)
    st = State(x=1.99, heading=math.radians(20.1))   # 1.59m 이동 + 20° 스냅
    link._update_speed(st, 0.08)
    assert st.respawned, "헤딩 스냅을 놓쳤다"


def test_정상_선회는_리스폰이_아니다():
    """실제로 낼 수 있는 요레이트(1.5 rad/s)로 계속 도는 건 오탐이면 안 된다."""
    link = VTDLink()
    st = State(); link._update_speed(st, 0.0)
    t, h = 0.0, 0.0
    for _ in range(40):
        t += 0.05; h += 1.5 * 0.05
        st = State(x=t * 8.0, heading=h)
        link._update_speed(st, t)
        assert not st.respawned, f"정상 선회를 리스폰으로 오탐(h={h:.2f})"


def test_sim이_느리면_주변차를_시계로_써서_속도를_되찾는다():
    """★VTD sim 이 실시간보다 느리게 돌면 벽시계 미분이 참속도를 크게 낮춰 읽는다.
    제어기는 "느리다"고 믿고 계속 가속한다(친구 실측 E_TR 46 -> 81km/h).
    객체는 speed 를 GT 로 주므로 **그 차가 움직인 거리 / 그 차의 속도 = 흐른 sim 시간**
    이다. 여기서는 sim 이 실시간의 1/7 로 도는 상황을 만든다:
      한 패킷당 sim 0.04초(= 14m/s x 0.04 = 0.56m 전진)인데 패킷은 0.28초마다 온다.
    보정이 없으면 0.56/0.28 = 2.0m/s 로 읽힌다."""
    link = VTDLink()
    st = State(); t = 0.0; x = 0.0
    v = 14.0
    def _objs(ox):
        return [Obj(1, ox + 20.0, 3.3, 0.0, 0.0, v, 4.4, 1.8, 1.5),
                Obj(2, ox - 25.0, -3.3, 0.0, 0.0, v, 4.4, 1.8, 1.5),
                Obj(3, ox + 60.0, 0.0, 0.0, 0.0, v, 4.4, 1.8, 1.5)]
    st.objects = _objs(0.0)
    link._update_speed(st, t)
    for _ in range(60):
        x += v * 0.04                       # sim 은 한 패킷에 0.04초어치만 간다
        t += 0.28                           # 벽시계는 그 7배가 흘렀다
        st = State(x=x); st.objects = _objs(x)
        link._update_speed(st, t)
    assert st.sim_clock, "시계를 못 쟀다"
    assert abs(st.sim_scale - 1.0 / 7.0) < 0.02, st.sim_scale
    assert abs(st.speed - v) < 0.8, f"보정 실패: {st.speed:.2f} (참값 {v})"


def test_주변차가_없으면_보정하지_않는다():
    """시계로 쓸 차가 없으면 아무것도 하지 않는다 — 예전과 똑같이 동작해야 한다."""
    link = VTDLink()
    raws, sm = _drive(link, 8.0, [0.05] * 60)
    assert abs(sm[-1] - 8.0) < 0.05, sm[-1]


def test_정상_주행에서는_시계가_1이라_아무_변화가_없다():
    """실측 검증(코스 E 실주행 15051프레임, 표본 57069): 중앙 scale 1.004.
    VTD 는 우리 랜에서 실시간으로 돈다 — 이 보정은 평소 무개입이어야 한다."""
    link = VTDLink()
    st = State(); t = 0.0; x = 0.0
    v = 10.0
    st.objects = [Obj(1, 20.0, 3.3, 0.0, 0.0, v, 4.4, 1.8, 1.5),
                  Obj(2, -25.0, -3.3, 0.0, 0.0, v, 4.4, 1.8, 1.5)]
    link._update_speed(st, t)
    for _ in range(60):
        x += v * 0.04; t += 0.04
        st = State(x=x)
        st.objects = [Obj(1, x + 20.0, 3.3, 0.0, 0.0, v, 4.4, 1.8, 1.5),
                      Obj(2, x - 25.0, -3.3, 0.0, 0.0, v, 4.4, 1.8, 1.5)]
        link._update_speed(st, t)
    assert abs(st.sim_scale - 1.0) < 0.02, st.sim_scale
    assert abs(st.speed - v) < 0.05, st.speed


def test_느린주기_정상주행은_리스폰_오탐_아님():
    # 5Hz(dt=0.2s)로 16m/s면 한 프레임 3.2m 이동 -> 거리조건만 보면 오탐
    link = VTDLink()
    st = State(); link._update_speed(st, 0.0)
    st = State(x=3.2); link._update_speed(st, 0.2)
    assert not st.respawned and st.speed > 0.0


# ---------------------------------------------------------------- 수신 끊김
# 배경(2026-08-16 EV_COMBO_CHAIN 실측): VTD 쪽 모듈이 순간 멈추자 recv 가 타임아웃을
# 던졌고 **제어기가 통째로 죽었다**(13초 만에 TimeoutError, 완주 0점).
# 대회 중 한 번 끊겼다고 판을 버릴 수는 없다 — 잠깐은 견디고, 오래가면 안전 정지.

def test_순간_수신끊김은_견딘다():
    import socket as _sk
    from vtd_io import VTDLink, DATA_SIZE

    class Flaky:
        def __init__(self, k): self.k = k
        def recv(self, n):
            if self.k > 0:
                self.k -= 1
                raise _sk.timeout()
            return b"\x00" * DATA_SIZE

    link = VTDLink()
    link.sock = Flaky(2)                       # 2회 타임아웃 후 정상 프레임
    try:
        got = link.recv_state()
    except _sk.timeout:
        raise AssertionError("타임아웃이 그대로 올라온다 — 제어기가 죽는다")
    assert got is not None, "잠깐 끊겼다고 포기하면 안 된다"


def test_오래_끊기면_None_으로_안전종료():
    import socket as _sk
    from vtd_io import VTDLink

    class Dead:
        def recv(self, n): raise _sk.timeout()

    link = VTDLink()
    link.sock = Dead()
    try:
        got = link.recv_state()
    except _sk.timeout:
        raise AssertionError("타임아웃이 그대로 올라온다 — 제어기가 죽는다")
    assert got is None, "진짜 끊겼으면 None 을 줘야 호출부가 정지시킨다"


def test_정지명령은_가속도0이_아니라_제동이다():
    """실측 2026-08-16 EV_COMBO_CHAIN: ego v=0.00 · 앞차 v=0.00 인데 간격이 70초에
    걸쳐 7.11 -> 5.80m 로 줄었다(경사 굴러감 ≈2cm/s). 그 바람에 추월 완화선(6.0m)마저
    밑돌아 영원히 못 나갔다. '멈춰 있어라'는 명령은 제동이어야 한다."""
    from control import LongPI
    pi = LongPI()
    assert pi.accel(0.0, 0.0, 0.05) < -0.5, "정지 중인데 제동을 안 건다"
    assert pi.accel(0.0, 0.1, 0.05) < -0.5, "미세하게 굴러가도 제동을 건다"
    # 정상 주행에는 영향 없어야 한다
    assert pi.accel(8.0, 8.0, 0.05) == 0.0 or abs(pi.accel(8.0, 8.0, 0.05)) < 0.5
    assert pi.accel(8.0, 2.0, 0.05) > 0.5, "가속 요구는 그대로"


def test_수신주기가_벌어지면_속도를_묶는다():
    """제어 갱신이 느려지면 그만큼 느리게 달려야 한다.

    2026-08-16 v7 실측: VTD 출력이 25Hz -> 3.5Hz 로 떨어졌다. 우리 계산은 0.8ms 라
    멀쩡했지만 제어 갱신이 0.28초에 한 번이라 50km/h 로 3.9m 를 눈감고 갔고,
    적신호를 6~10건 그냥 지났다. 이유가 시뮬 쪽이어도 감점은 우리 몫이다.
    """
    import drive as D
    st = D.DrivingStack(route=D.straight_route(300.0), base_limit=13.9)
    s = State(); s.x, s.y, s.heading, s.speed = 10.0, 0.0, 0.0, 13.9

    fast = st.step(s, 0.04, 1.0).v_cmd          # 25Hz
    for i in range(60):                          # EMA 가 느린 주기에 수렴하도록
        st.step(s, 0.28, 2.0 + i * 0.28)
    slow = st.step(s, 0.28, 30.0).v_cmd          # 3.5Hz

    assert slow < fast, (slow, fast)
    assert abs(slow - D.DrivingStack.STEP_METERS / 0.28) < 1.0, slow
    print(f"    25Hz {fast*3.6:.0f}km/h -> 3.5Hz {slow*3.6:.0f}km/h")


def test_달린_적_없으면_완주가_아니다():
    """★실측 2026-09-04 사전테스트 준비: 시나리오가 안 올라가 ego 가 (0,0) 이었는데
    제어기가 **첫 프레임에** `✅ 목표 도달 (x=0.0 y=0.0, 0s)` 을 찍고 끝냈다.
    그 경로는 시작까지 1690m·끝까지 928m 라 **끝점이 더 가까워** 최근접 인덱스가
    종점으로 잡혔기 때문이다. 대회장이었으면 아무것도 안 하고 조용히 미완주다."""
    import drive as D
    from vtd_io import State
    route = D.straight_route(300.0)
    st = D.DrivingStack(route=route, base_limit=13.9)
    #  경로 **끝** 근처에 갑자기 놓인다(= 달린 적 없음)
    s = State(); s.x, s.y, s.heading, s.speed = 299.0, 0.0, 0.0, 0.0
    cmd = st.step(s, 0.04, 1.0)
    assert not cmd.goal_reached, "달린 적 없는데 완주로 봤다"


def test_실제로_달려서_끝에_닿으면_완주다():
    import drive as D
    from vtd_io import State
    route = D.straight_route(300.0)
    st = D.DrivingStack(route=route, base_limit=13.9)
    now = 1.0
    cmd = None
    for x in range(0, 300, 2):                  # 2m 씩 실제로 굴러간다
        now += 0.1
        s = State(); s.x, s.y, s.heading, s.speed = float(x), 0.0, 0.0, 8.0
        cmd = st.step(s, 0.1, now)
    assert st._odo > D.DrivingStack.GOAL_MIN_RUN, st._odo
    assert cmd.goal_reached, "끝까지 달렸는데 완주가 아니다"


def test_리스폰_점프는_주행거리에_안_들어간다():
    import drive as D
    from vtd_io import State
    st = D.DrivingStack(route=D.straight_route(300.0), base_limit=13.9)
    s = State(); s.x = 0.0; st.step(s, 0.04, 1.0)
    s2 = State(); s2.x = 200.0; s2.respawned = True      # 순간이동
    st.step(s2, 0.04, 1.04)
    assert st._odo < 1.0, st._odo


def test_max_speed_는_지도값보다_느리게_묶는다():
    """--max-speed: 지도 제한보다 **느리게** 달리는 상한. 느린 건 위반이 아니다.

    2026-08-17 실측: 차로 안 정지물 옆을 제한 45 이상으로 지나가면 VTD 출력이
    25Hz -> 3.4Hz 로 무너지고 **끝까지 회복되지 않는다**(제어를 끊고 25초 세웠다
    재접속해도 3.4Hz. VTD 재시작 외엔 안 풀린다). 40 이하면 안 걸린다.
    그런 코스를 만나면 당일 플래그 하나로 낮춘다 — 미완주보다 26초가 싸다.
    """
    import drive as D
    route = D.straight_route(300.0)
    plan = [{"lim": 13.9, "lane": -1, "w": 3.2, "l": 0.0, "r": 0.0,
             "xl": 0.0, "xr": 0.0, "need": 0.0, "sig": 0, "j": 0}] * len(route)

    #  ★모든 상한에서 SPEED_MARGIN 만큼 미리 빼 둔다(아래 과속여유 테스트 참조).
    M = D.DrivingStack.SPEED_MARGIN
    free = D.DrivingStack(route=route, base_limit=13.9, lane_plan=plan)
    assert abs(free.base_limit - (13.9 - M)) < 1e-6, free.base_limit

    cap = D.DrivingStack(route=route, base_limit=13.9, lane_plan=plan, max_speed=11.1)
    assert abs(cap.base_limit - (11.1 - M)) < 1e-6, cap.base_limit
    assert max(cap.v_prof) <= 11.1 + 1e-6, max(cap.v_prof)
    # 지도값이 상한보다 낮은 구간(보호구역 30)은 그대로 30(-여유) 이어야 한다
    plan30 = [dict(p, lim=8.33) for p in plan]
    z = D.DrivingStack(route=route, base_limit=13.9, lane_plan=plan30, max_speed=11.1)
    assert abs(max(z.v_prof) - (8.33 - M)) < 0.2, max(z.v_prof)


def test_제한속도를_목표로_삼지_않는다():
    """★실측 2026-08-30 코스 E 753초: 50 구간 최대 **+4.68km/h**, 30 구간 +2.15,
    1km/h 초과가 4.28%/3.58%, 초과 구간 29개(합 27.2초). 안내문은 +1km/h 까지만
    봐준다[항목 1·2] — 넘으면 구간마다 경미 -3 씩, 두 항목이니 구간당 최대 -6 이다.
    프로파일이 제한값을 그대로 목표로 주면 제어기·추정지연이 그만큼을 넘긴다.
    """
    import drive as D
    M = D.DrivingStack.SPEED_MARGIN
    assert M * 3.6 >= 1.5, "관측된 초과(최대 +2.7km/h 지속)를 덮지 못한다"
    route = D.straight_route(300.0)
    plan = [{"lim": 13.9, "lane": -1, "w": 3.2, "l": 0.0, "r": 0.0,
             "xl": 0.0, "xr": 0.0, "need": 0.0, "sig": 0, "j": 0}] * len(route)
    st = D.DrivingStack(route=route, base_limit=13.9, lane_plan=plan)
    assert max(st.v_prof) <= 13.9 - M + 1e-6, max(st.v_prof)
    #  보호구역(30)도 같은 여유를 받는다 — 항목 2 는 +1 부터 경미다
    plan30 = [dict(p, lim=8.33) for p in plan]
    z = D.DrivingStack(route=route, base_limit=13.9, lane_plan=plan30)
    assert max(z.v_prof) <= 8.33 - M + 1e-6, max(z.v_prof)


def test_순환코스에서_첫_인덱스는_출발점이다():
    """실측 2026-08-19 새 코스 A — 5.3km 중 2.4km 를 통째로 건너뛴 사고의 재현.

    코스가 출발점 근처로 되돌아오면, 스폰 지점에서 rt[0] 과 뒤쪽 구간이 **똑같이
    가깝다**. 첫 프레임을 전역 최근접으로 잡으면 뒤쪽을 집어 앞부분을 안 간다
    (그러고도 '완주'로 끝난다 — 목표점에는 도달하니까). 지우지 말 것.
    """
    from drive import nearest_index
    # 0~9: 동쪽으로 감 / 10~19: 되돌아와 출발점 옆을 다시 지남
    route = [(x * 3.0, 0.0) for x in range(10)] + [(27.0 - x * 3.0, 2.5) for x in range(10)]
    i = nearest_index(route, 0.5, 1.2, None)     # 출발점과 뒤쪽 구간이 비슷하게 가깝다
    assert i == 0, f"첫 인덱스가 {i} — 뒤쪽 구간을 집었다(앞부분을 통째로 건너뛴다)"


def test_출발점에서_멀면_전역으로_찾는다():
    """녹화 경로를 중간부터 재생하는 경우까지 막으면 안 된다."""
    from drive import nearest_index
    route = [(x * 3.0, 0.0) for x in range(40)]
    assert nearest_index(route, 90.0, 0.4, None) == 30



# ── 어린이보호구역 무신호 횡단보도 [법 제27조⑦] ─────────────────────────────
# 사용자 지적 2026-08-20: "골목길 횡단보도 정지선에서는 왜 안 멈춤?"
# 우리 코드는 신호등만 봤다. 코스 A 한 판에 해당 지점이 24곳인데 한 곳도 안 섰다.

def _cw_stack(lim_kmh, signal=False):
    import drive as D
    route = [(float(i), 0.0) for i in range(0, 200)]
    plan = [{"lim": lim_kmh / 3.6, "w": 3.3, "l": 3.3, "r": 3.3, "need": 0.0,
             "xl": 0.0, "xr": 0.0, "j": 0, "road": "1", "lane": -1} for _ in route]
    cw = [{"road": "1", "s": 0.0, "x": 100.0, "y": 0.0,
           "stops": [], "signal": signal}]
    return D.DrivingStack(route=route, base_limit=13.9, lane_plan=plan, crosswalks=cw)


_T = [0.0]


def _at(stack, x, v):
    s = State(); s.x = x; s.y = 0.0; s.heading = 0.0; s.speed = v
    s.tl_id = -1; s.tl_state = 0
    _T[0] += 0.04
    s.t = _T[0]
    return stack.step(s, 0.04, _T[0])


def test_보호구역_무신호_횡단보도에는_선다():
    st = _cw_stack(30)
    c = _at(st, 75.0, 8.0)                      # 25m 앞 -> 이미 감속 계획에 들어와야
    assert c.reason == "CROSSWALK_STOP", c.reason
    assert c.v_cmd < 8.0, f"횡단보도 25m 앞인데 {c.v_cmd*3.6:.0f}km/h ({c.reason})"
    c = _at(st, 92.0, 2.0)                      # 8m 앞 = 정지 지점
    assert c.v_cmd < 0.6, f"횡단보도 앞인데 안 섬 ({c.v_cmd*3.6:.0f}km/h, {c.reason})"


def test_한번_서면_그_횡단보도는_통과시킨다():
    """안 그러면 영원히 못 간다."""
    st = _cw_stack(30)
    _at(st, 92.0, 0.0)                          # 정지 목표점(=100-8)에서 섰다
    c = _at(st, 93.0, 0.5)
    assert c.reason != "CROSSWALK_STOP", c.reason


def test_멀리서_선_것은_인정하지_않는다():
    """★실측 2026-08-20 코스 A: 24곳 중 18번만 섰다. 적신호에 서 있는 동안
       20m 앞 횡단보도까지 '섰다'로 소진되고, 붙어 있는 횡단보도도 한 번에 지워졌다.
       **그 횡단보도 앞에서** 서야 인정해야 한다."""
    st = _cw_stack(30)
    _at(st, 70.0, 0.0)                          # 22m 앞에서 (다른 이유로) 정지
    c = _at(st, 88.0, 3.0)                      # 다시 접근
    assert c.reason == "CROSSWALK_STOP", f"멀리서 선 걸로 소진됐다 ({c.reason})"


def test_정지선이_있으면_그_앞에_선다():
    """[법 제27조⑦] '정지선이 설치된 경우에는 그 정지선'.
       ⚠️ 후보 중 **내 앞이면서 횡단보도보다 가까운 것**을 써야 한다 — 그냥 제일 가까운
       걸 집으면 **건너편 정지선**을 잡는다(실측 2026-08-20 코스 A: 24곳 중 10곳)."""
    import drive as D
    route = [(float(i), 0.0) for i in range(0, 200)]
    plan = [{"lim": 30 / 3.6, "w": 3.3, "l": 3.3, "r": 3.3, "need": 0.0,
             "xl": 0.0, "xr": 0.0, "j": 0, "road": "1", "lane": -1} for _ in route]
    # 횡단보도 x=100, 정지선은 앞(x=96)과 **건너편**(x=104) 둘 다 있다
    cw = [{"road": "1", "s": 0.0, "x": 100.0, "y": 0.0,
           "stops": [[104.0, 0.0], [96.0, 0.0]], "signal": False}]
    st = D.DrivingStack(route=route, base_limit=13.9, lane_plan=plan, crosswalks=cw)
    c = _at(st, 91.2, 1.0)          # 정지선(96)까지 4.8m = CW_GAP_LINE -> 여기서 0
    assert c.v_cmd < 0.6, f"앞쪽 정지선(96m) 앞에서 서야 함 ({c.v_cmd*3.6:.1f}km/h, {c.reason})"


def test_보호구역이_아니면_안_선다():
    """무신호 횡단보도라도 보호구역 밖이면 의무 정지가 아니다 — 보행자가 있을 때만."""
    st = _cw_stack(50)
    c = _at(st, 92.0, 10.0)
    assert c.reason != "CROSSWALK_STOP", c.reason
    assert c.v_cmd > 6.0, c.v_cmd


def test_신호_있는_횡단보도는_이_규칙_대상이_아니다():
    """신호가 있으면 신호를 따르면 된다. 두 번 세우면 시간만 버린다."""
    st = _cw_stack(30, signal=True)
    c = _at(st, 92.0, 8.0)
    assert c.reason != "CROSSWALK_STOP", c.reason


def test_양보용_횡단보도는_보호구역_밖에서도_찾는다():
    """일시정지 의무와 달리, 이미 필요한 양보 정지의 위치 보정에는 일반 횡단보도도 쓴다."""
    st = _cw_stack(50)
    s = State(); s.x = 80.0; s.y = 0.0; s.heading = 0.0; s.speed = 5.0
    key, cw = st._crosswalk_ahead(s, 50 / 3.6, s_ego=80.0, zone_only=False)
    assert key is not None and cw is not None
    assert abs((cw[0] - cw[1]) - 12.0) < 1e-6, cw

    # 의무정지를 이미 마쳤다는 표식이 양보 위치 보호까지 꺼서는 안 된다.
    st._cw_done.add(key)
    key_after_stop, _cw = st._crosswalk_ahead(
        s, 50 / 3.6, s_ego=80.0, zone_only=False)
    assert key_after_stop == key

    # 기존 보호구역 의무정지 검색은 넓어지면 안 된다.
    key_mandatory, _cw = st._crosswalk_ahead(s, 50 / 3.6, s_ego=80.0)
    assert key_mandatory is None


def test_주행스택이_횡단보도_앞_목표를_양보규칙에_넘긴다():
    """지도 검색과 Behavior 인자가 실제 step 경로에서 끊기지 않았는지 확인한다."""
    st = _cw_stack(50)                           # 일반(비보호구역) 횡단보도 x=100
    st._junction_ahead = lambda _bi: 12.0
    st._upcoming_turn = lambda _bi: 1
    st.beh._oncoming_clear = lambda *_args, **_kwargs: False

    c = _at(st, 88.0, 5.7)                      # 횡단보도 안전 정지점 x=92까지 4m
    assert c.reason == "YIELD_CROSS", c.reason
    expected = st.beh._stop_target_speed(4.0)
    assert abs(c.v_cmd - expected) < 1e-6, (c.v_cmd, expected)


def test_급커브에서만_전방주시를_줄인다():
    """실측 2026-08-27 공식 WP9: 지시 오프셋 0 인데 경로에서 최대 2.70m 벗어나고
    `|d_ego| > 0.5m` 가 프레임의 8.1%. 차로폭 2.40m 굽은 구간에서 **차선을 5.2초 물었다.**
    v1~v7 은 같은 조건에서 최대 0.22m · 0.0% — 문제는 추종이 아니라 **급커브**다.

    ★게이트는 **공식 직선 코스가 원리상 안 걸리도록** 잡았다:
      10m 방위변화 최대  v1 4.8° · v7 4.8°  vs  WP9 117.8°(11.2% 가 20° 초과)"""
    import json, math, os, sys
    R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "routes")
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
    from scenario import Scenario
    from drive import DrivingStack

    def curves(base):
        st = DrivingStack(scenario=Scenario.load(f"{R}/{base}.json"), base_limit=8.33,
                          lane_plan=json.load(open(f"{R}/{base}_lane.json"))["pts"])
        return [st._local_curve(i) for i in range(len(st.route))], st

    cv1, st1 = curves("planned_from_xml_v1")
    cw9, st9 = curves("planned_wp9")
    assert max(cv1) < st1.TURN_CURVE_DEG, f"공식 직선 코스가 게이트에 걸린다 (최대 {max(cv1):.1f}°)"
    assert max(cw9) > 90.0, f"WP9 급커브를 못 본다 (최대 {max(cw9):.1f}°)"
    hit = sum(1 for c in cw9 if c > st9.TURN_CURVE_DEG) / len(cw9)
    assert 0.03 < hit < 0.30, f"WP9 발동 비율이 이상하다 ({hit*100:.1f}%)"


def test_회전_지시등은_회전이_끝날_때까지_켠다():
    """실측 2026-08-28 코스 A: 회전 12개 중 **11개가 1.2~2.3초 일찍** 꺼졌다.
    `_upcoming_turn` 은 35m 앞을 보므로 회전에 깊이 들어가면 남은 각도가 문턱(25°)
    아래로 떨어져 0 이 된다 — **핸들은 아직 꺾여 있는데** 꺼진다.
    사용자가 화면에서 먼저 봤다: "steer -24.4° 인데 sig 가 꺼져 있다".
    시행령 별표2 는 "그 행위가 **끝날 때까지**" 신호하라고 한다.

    ⚠️ 경로 곡률로 래치를 풀면 부족하다 — **경로는 이미 직선인데 차가 아직 돈다**
       (전방주시 지연). 실측: 곡률 기준은 12개 중 11개가 여전히 0.8~1.4초 일찍 꺼졌다.
       **자차 회전율**로 풀어야 12/12 가 끝까지 켜진다.

    여기서는 경로를 그대로 따라가는 합성 주행으로 **큰 회전마다 그 구간 내내 켜져
    있는가**를 본다(실주행 되먹임 결과는 커밋 메시지에).
    """
    import json, math, os, sys
    HERE = os.path.dirname(os.path.abspath(__file__))
    R = os.path.join(HERE, "..", "routes")
    sys.path.insert(0, os.path.join(HERE, "..", "src"))
    from scenario import Scenario
    from drive import DrivingStack
    from vtd_io import State

    lane = json.load(open(f"{R}/HL_FMA_NEW_A_lane.json", encoding="utf-8"))["pts"]
    st = DrivingStack(scenario=Scenario.load(f"{R}/HL_FMA_NEW_A.json"), base_limit=8.33,
                      lane_plan=lane)
    route = st.route
    V, HZ, STEP = 8.0, 25.0, 2
    log = []                                   # (방위, 지시등)
    t = 0.0
    for i in range(STEP, len(route) - STEP, STEP):
        s = State()
        s.x, s.y = route[i]
        a, b = route[i - STEP], route[i + STEP]
        s.heading = math.atan2(b[1] - a[1], b[0] - a[0])
        s.speed, s.t = V, t
        cmd = st.step(s, STEP / HZ, t)
        log.append((s.heading, cmd.turn, bool((lane[i] or {}).get("j"))))
        t += STEP / HZ

    # 큰 회전 구간(누적 40° 이상)을 찾아 그 안에서 지시등이 켜져 있던 비율을 본다
    def d(i):
        return math.degrees((log[i][0] - log[i - 1][0] + math.pi) % (2 * math.pi) - math.pi)
    segs, cur = [], None
    for i in range(1, len(log)):
        if abs(d(i)) > 0.5:                    # 스텝당 0.5° = 초당 6°
            cur = [i, i] if cur is None else [cur[0], i]
        elif cur:
            segs.append(cur); cur = None
    if cur:
        segs.append(cur)
    bad = []
    for a, b in segs:
        tot = sum(d(k) for k in range(a + 1, b + 1))
        if abs(tot) < 40 or b - a < 4:
            continue
        # ★**교차로를 지나는 회전만** 본다. 교차로가 없으면 그냥 굽은 길이고,
        #   거기서 켜면 그게 오점등이다(사용자가 두 번 지적한 바로 그 건).
        if not any(log[k][2] for k in range(a, b + 1)):
            continue
        want = 2 if tot < 0 else 1
        on = sum(1 for k in range(a, b + 1) if log[k][1] == want)
        if on < 0.8 * (b - a + 1):
            bad.append((round(tot), f"{on}/{b - a + 1}"))
    assert segs, "합성 주행에서 회전 구간을 못 찾았다 — 시험 전제가 깨졌다"
    assert not bad, f"회전 내내 안 켜진 곳: {bad}"




def test_수신_백로그는_최신_프레임만_쓴다():
    """VTD 가 멎었다 몰아 보내면(모듈 히컵) 묵은 프레임 순서대로 소화하는 동안
    과거 위치로 조향한다. 4프레임(160ms) 넘게 밀렸으면 최신만 쓴다."""
    from vtd_io import VTDLink, pack_data, DATA_SIZE

    class FakeSock:
        def __init__(self, chunks):
            self.chunks = list(chunks)
        def recv(self, n):
            return self.chunks.pop(0) if self.chunks else b""

    frames = b"".join(pack_data(float(x), 0.0, 0.0, 0.0, 0.0, 0.0, [])
                      for x in (10, 11, 12, 13, 14))
    lk = VTDLink()
    lk.sock = FakeSock([frames])
    s1 = lk.recv_state()
    assert s1 is not None and s1.x == 14.0, s1.x     # 5개 중 최신만
    assert len(lk._buf) == 0
    assert lk.recv_state() is None                   # 소켓 닫힘


def test_백로그_2프레임은_안_버린다():
    """정상 지터(TCP 코얼레싱 1~2프레임)는 건드리지 않는다 — 제어율이 반토막난다."""
    from vtd_io import VTDLink, pack_data

    class FakeSock:
        def __init__(self, chunks):
            self.chunks = list(chunks)
        def recv(self, n):
            return self.chunks.pop(0) if self.chunks else b""

    frames = b"".join(pack_data(float(x), 0.0, 0.0, 0.0, 0.0, 0.0, [])
                      for x in (20, 21))
    lk = VTDLink()
    lk.sock = FakeSock([frames])
    assert lk.recv_state().x == 20.0                 # 순서대로
    assert lk.recv_state().x == 21.0


def test_드레인_후_리스폰_오탐이_없다():
    """버린 프레임만큼 위치가 점프해도, dt 도 같이 길어져 JUMP_V 를 못 넘는다."""
    from vtd_io import VTDLink, pack_data
    import time as _time

    class FakeSock:
        def __init__(self, chunks):
            self.chunks = list(chunks)
        def recv(self, n):
            return self.chunks.pop(0) if self.chunks else b""

    lk = VTDLink()
    f0 = pack_data(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, [])
    lk.sock = FakeSock([f0])
    lk.recv_state()
    _time.sleep(0.25)                                # 250ms 멎었다가
    burst = b"".join(pack_data(0.7 * k, 0.0, 0.0, 0.0, 0.0, 0.0, [])
                     for k in range(1, 7))           # 6프레임 몰아서 (총 4.2m)
    lk.sock = FakeSock([burst])
    s2 = lk.recv_state()
    assert abs(s2.x - 4.2) < 1e-4 and not s2.respawned, (s2.x, s2.respawned)

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
