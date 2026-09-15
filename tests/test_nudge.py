"""소형 인레인 장애물 횡회피(nudge) 유닛테스트.

배경(2026-08-15): 공식 v5 의 Fuelcan01 이 경로 중심선에서 4cm 지점에 있는데
차폭 절반이 0.94m 라 **그대로 밟고 지나갔다**. 사람(H>=1.2)은 정지, 차(최대변>=1.2)는
추월 FSM 이 맡는데 그 사이 크기가 아무 규칙에도 안 걸렸던 것.
강의 [70:57] 이 "연료통 ... 회피하려고 차선 이동을 해서 간 거 같고요" 라고 확인해준다.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from drive import DrivingStack, straight_route      # noqa: E402

FUELCAN = dict(L=0.15, W=0.46, H=0.61)


def obj(ds, d, L=0.15, W=0.46, H=0.61, spd=0.0, oid=9):
    return (ds, d, spd, L, W, H, oid)


def stack(**kw):
    return DrivingStack(route=straight_route(400.0), **kw)


def settle(st, objs, d_ego=0.0, n=120, dt=0.05, plan=None, ped=False):
    """오프셋이 수렴할 때까지 돌리고 최종값을 돌려준다."""
    off = 0.0
    for _ in range(n):
        off = st._nudge_offset(objs, d_ego, dt, plan if plan is not None else P(), ped)
    return off


def test_경로중앙_연료통은_차폭만큼_비킨다():
    st = stack()
    off = settle(st, [obj(10.0, 0.04)])
    # 필요 여유 = 반차폭0.943 + 물체반폭0.23 + 여유0.25 = 1.42m
    assert abs(off) > 1.0, off
    assert abs(off - 0.04) >= 0.943 + 0.23 + 0.20, off      # 차체가 물체를 안 스친다


def test_이미_충분히_떨어진_물체엔_안_비킨다():
    st = stack()
    off = settle(st, [obj(10.0, 3.0)])                       # 옆차선 갓길
    assert off == 0.0, off


def test_낮은_노면물체는_무시():
    st = stack()
    off = settle(st, [obj(10.0, 0.0, H=0.1)])                # 맨홀 등
    assert off == 0.0, off


def test_자동차는_nudge가_아니라_추월FSM_담당():
    st = stack()
    off = settle(st, [obj(10.0, 0.0, L=4.4, W=1.8, H=1.5)])
    assert off == 0.0, off


def test_지나가면_경로로_복귀():
    st = stack()
    settle(st, [obj(10.0, 0.04)])
    assert settle(st, [obj(-8.0, 0.04)]) == 0.0              # 뒤로 보냈다


def test_목표가_도망가지_않는다():
    """d 는 ego 기준 상대값이라, 비킨 만큼 d_ego 를 반영하지 않으면 목표가 계속 밀린다."""
    st = stack()
    off = settle(st, [obj(10.0, 0.04)])
    # 이제 ego 가 실제로 off 만큼 비켜 있다고 보고, 물체는 ego 기준 반대편으로 보인다
    d_rel = 0.04 - off
    off2 = st._nudge_offset([obj(10.0, d_rel)], off, 0.05, P())
    assert abs(off2 - off) < 0.05, (off, off2)               # 같은 자리를 유지해야 한다


def test_오프셋은_한번에_안_튄다():
    st = stack()
    first = st._nudge_offset([obj(10.0, 0.04)], 0.0, 0.05, P())
    assert abs(first) <= DrivingStack.NUDGE_RATE * 0.05 + 1e-9, first


def test_상한을_넘지_않는다():
    st = stack()
    off = settle(st, [obj(10.0, 0.0, L=1.19, W=1.19, H=0.8)])
    assert abs(off) <= DrivingStack.NUDGE_MAX + 1e-9, off



def test_사람은_비켜가지_않고_선다():
    """실측 2026-08-15 EV_PED: DummyPerson(L0.23 W0.40 H1.40)이 '작은 장애물'로
    분류돼 **서야 할 사람을 옆으로 비켜 지나가고 있었다**. 사람은 behavior 가 세운다."""
    st = stack()
    off = settle(st, [obj(10.0, 0.0, L=0.23, W=0.40, H=1.40)])
    assert off == 0.0, off


def test_휠체어는_사물이_아니라_취약대상으로_우회한다():
    """H=0.92라 옛 H>=1.2 식에서는 사물이었고, behavior의 YIELD_PED와 모순됐다."""
    st = stack()
    wheelchair = [(10.0, 0.0, 0.0, 1.01, 0.62, 0.92, 31, 0.62, 1.01)]
    assert settle(st, wheelchair, ped=False) == 0.0       # 먼저 정지·양보
    assert abs(settle(st, wheelchair, ped=True)) > 1.0   # 허가 후에는 넓게 우회


def test_자전거도_원본치수로_취약대상_우회한다():
    """경로축 외접상자는 비스듬한 자전거 폭을 키우므로 원본 W/L을 사용해야 한다."""
    st = stack()
    # 경로축 L/W=0.8/2.0, 원본 W/L=0.65/1.90
    bicycle = [(10.0, 0.0, 0.0, 0.8, 2.0, 1.10, 32, 0.65, 1.90)]
    assert st._rf_is_vru(bicycle[0])
    assert settle(st, bicycle, ped=False) == 0.0
    assert abs(settle(st, bicycle, ped=True)) > 1.0


def test_휠체어가_있는_자리는_사람용_여유로_막는다():
    st = stack()
    wheelchair = [(10.0, 3.0, 0.0, 1.01, 0.62, 0.92, 31, 0.62, 1.01)]
    # off=1.0이면 실여유 약 0.75m: 차량용 0.35m는 넘지만 VRU용 1.2m에는 부족하다.
    assert not st._lat_free(1.0, wheelchair, d_ego=0.0)


# ---- 추월/회피 방향을 지도로 고르기 ----
def P(l=5.0, r=9.0, xl=0.0, xr=0.0, w=3.3, need=0.0, lane=-2):
    return dict(lane=lane, w=w, l=l, r=r, xl=xl, xr=xr, need=need)


def test_좌측여유_없으면_우측으로_추월():
    """이 코스는 우리가 맨 왼쪽 차선인 구간이 많다(좌여유 중앙값 1.6m).
    좌측으로 나가면 중앙선을 넘어 리스폰(감점)이 난다 — 실측 2026-08-15."""
    st = stack()
    assert st._side_for(P(l=1.5, r=9.0), need=4.44) < 0


def test_같은방향_오른쪽차로로_장애물회피가_가능하다():
    st = stack()
    off = settle(st, [obj(10.0, 0.0)], plan=P(l=1.0, r=5.0))
    assert off < -1.0


def test_좌측여유_있으면_통상대로_좌측():
    st = stack()
    assert st._side_for(P(l=5.0, r=9.0), need=4.44) > 0


def test_같은방향_왼쪽차로로_장애물회피가_가능하다():
    st = stack()
    off = settle(st, [obj(10.0, 0.0)], plan=P(l=5.0, r=1.0))
    assert off > 1.0


def test_일반대로에서는_반대차선으로_안_넘어간다():
    """같은 방향 차로가 안 되면 그냥 못 비키는 것 — 중앙선이 실선이면 xl/xr=0."""
    st = stack()
    assert st._side_for(P(l=1.0, r=1.0, xl=0.0, xr=0.0), need=4.44) == 0.0


def test_점선이어도_그냥_막힌_정도로는_안_넘는다():
    """주최측 공식 답변상 점선도 중앙선 침범이며 장애물 회피 면책이 없다."""
    st = stack()
    assert st._side_for(P(l=1.0, r=1.0, xl=6.0, xr=0.0), need=4.44) == 0.0


def test_오래_갇혀도_기본모드는_점선_반대차선을_쓰지_않는다():
    """공식 답변상 장애물·점선 예외가 없으므로 desperate도 기본 정책을 못 푼다."""
    st = stack()
    assert st._side_for(P(l=1.0, r=1.0, xl=6.0, xr=0.0),
                        need=4.44, desperate=True) == 0.0


def test_모르는_구간은_예전대로_넛지한다():
    """★2026-09-08. plan=None 은 실주행에 없는 경우다(모의 전용). 예전 동작을 그대로 둔다.

    친구 커밋은 여기서 넛지도 막았는데, 그러면 오프라인 회귀가 통째로 죽는다.
    중앙선 정책은 차로계획이 **있을 때** `xl/xr` 잠금으로 지킨다.
    """
    st = stack()
    for _ in range(120):
        off = st._nudge_offset([obj(10.0, 0.0)], 0.0, 0.05, plan=None)
    assert abs(off) > 0.5, off

def test_모르는_구간에_들어가도_진행중_횡회피는_급복귀하지_않는다():
    """새 목표는 만들지 않되 기존 nudge는 변화율 제한으로 안전하게 복귀한다."""
    st = stack()
    st._nudge = 2.0
    off = st._nudge_offset([], d_ego=2.0, dt=0.1, plan=None)
    assert 0.0 < off < 2.0
    assert abs(off - 2.0) <= st.NUDGE_RATE * 0.1 + 1e-9


def test_명시적_옵션에서만_점선_반대차선을_탈출로_쓴다():
    import contextlib
    import io
    logbuf = io.StringIO()
    with contextlib.redirect_stdout(logbuf):
        st = stack(allow_centerline_escape=True)
        assert st._side_for(P(l=1.0, r=1.0, xl=6.0, xr=0.0),
                            need=4.44, desperate=True) > 0
    log = logbuf.getvalue()
    assert "평가상 중앙선 침범" in log and "면책되지 않는" in log


def test_환경변수도_명시적으로_켜야_중앙선_탈출이_열린다():
    old = os.environ.get("ALLOW_CENTERLINE_ESCAPE")
    try:
        os.environ["ALLOW_CENTERLINE_ESCAPE"] = "1"
        st = stack()
        assert st.allow_centerline_escape is True
        assert st._side_for(P(l=1.0, r=1.0, xl=6.0), 4.44, desperate=True) > 0
    finally:
        if old is None:
            os.environ.pop("ALLOW_CENTERLINE_ESCAPE", None)
        else:
            os.environ["ALLOW_CENTERLINE_ESCAPE"] = old


def test_추월_기동중에는_방향을_안_바꾼다():
    """나가 있는 도중에 방향이 뒤집히면 장애물을 가로질러 되돌아온다.

    ★2026-09-05 PR #3 이후 계약이 바뀌었다: FOLLOW/WAIT 에서도 **한 번 요청으로는 안
      바뀐다.** 같은 요청이 SIDE_HOLD(0.8초) 이어져야 바뀐다 — 정지 중 깜빡이가
      L/R 로 떨리던 원인이 매 프레임 재선택이었다(tests/test_overtake_side_hold.py).
      이 테스트는 그 전 계약("한 번이면 바뀐다")을 박아 둔 것이라 PR #3 때 같이 고쳐야
      했는데, 검증 결과를 `grep '[0-9]+ passed'` 로 뽑아 **'1 failed' 를 놓쳤다.**
    """
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.5, side=+1.0)
    ov.state = "PASS"; ov.W = 3.5
    for k in range(40):                     # 1.6초 내내 반대쪽을 요청해도
        ov.set_side(-1.0, k * 0.04)
    assert ov.W == 3.5                      # 기동 중엔 절대 안 바뀐다
    ov.state = "FOLLOW"
    ov.set_side(-1.0, 10.0)
    assert ov.W == 3.5                      # 한 번으로는 안 바뀐다(떨림 방지)
    t = 10.0
    while t < 10.0 + ov.SIDE_HOLD + 0.1:
        t += 0.04
        ov.set_side(-1.0, t)
    assert ov.W == -3.5                     # 이어지면 바뀐다



def test_차로_한복판_정지차는_내가_옆에_나가_있어도_막는_차다():
    """블로커 판정은 **차로 중심 기준**이어야 한다. 내 현재 위치 기준이 아니라.

    실측 2026-08-16 EV_LEADBRAKE: 앞선 회피로 차로 중심에서 왼쪽 3.35m 에 나가 있었고
    정지차는 경로 절대 d=+0.00(차로 정중앙)이었다. 상대 기준으로 보면 3.35m 옆이라
    '안 막는다'가 되고, 그대로 차로 중심으로 복귀하다 **들이받았다**
    (접촉 4376프레임, 200초 정지).
    """
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.5, side=+1.0)
    # (id, fx, fy, spd, len, wid, hgt) — fy 는 **내 위치 기준** 상대 횡거리
    stopped = [(9, 12.0, -3.35, 0.0, 4.4, 1.8, 1.5)]     # 내가 왼쪽 3.35m 에 있을 때

    assert ov._own_lane_block(stopped, d_ego=0.0) is None      # 옆 차선이면 안 막는다
    blk = ov._own_lane_block(stopped, d_ego=3.35)              # 실제로는 차로 한복판
    assert blk is not None and blk[2] == 9, blk


def test_목표차선_판정도_차로_중심_기준이다():
    """내가 이미 왼쪽에 나가 있어도, 목표차선은 '차로 중심에서 왼쪽 한 칸' 이다."""
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.5, side=+1.0)
    # 차로 중심 기준 +3.5m(=목표차선 한복판)에 선 차. 내가 +3.5m 에 나가 있으면 fy=0.
    car = [(4, 10.0, 0.0, 0.0, 4.4, 1.8, 1.5)]
    assert ov._target_lane_clear(car, 8.0, d_ego=0.0) is True    # 내 자리면 목표차선 밖
    assert ov._target_lane_clear(car, 8.0, d_ego=3.5) is False   # 목표차선이 막혔다


def test_비스듬히_선_차의_폭을_본다():
    """실측 2026-08-26 코스 H: 원본 L5.2 W1.9 짜리가 약 30° 기울어 서 있어 경로축 폭이
    **4.4m**(부풀린 게 아니라 실제 가로폭)였다. 중심이 목표차선 밖(3.6m)이라 '비었다'로
    보고 좌측 추월을 커밋했는데, 몸통이 목표차선을 덮고 있어 차체 안전망이 0 으로 잡았다.
    -> `OVT:PASS` 인 채 **203초 교착**. 중심만 보면 안 되고 **폭**을 같이 봐야 한다."""
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.3, side=+1.0)
    # 중심은 목표차선(+3.3) 에서 3.6m 떨어져 있지만 폭 4.4m 라 몸통이 걸친다
    tilted = [(1, 8.7, -0.3, 0.0, 5.2, 4.4, 1.5)]
    assert ov.side_clear(tilted, 0.0, 0.0, +1.0) is False, "몸통이 걸치는데 비었다고 본다"
    # 폭이 실제로 좁으면(정렬된 차) 목표차선은 비어 있다
    aligned = [(1, 8.7, -0.3, 0.0, 5.2, 1.9, 1.5)]
    assert ov.side_clear(aligned, 0.0, 0.0, +1.0) is True, "정렬된 차인데 막혔다고 본다"


def test_추월대상이_자기_추월차선을_막으면_안된다():
    """실측 2026-08-26 공식 회귀: EV_ONCOMING·REARPASS·BOTHBLOCK·COMBO_PASSPED **4판**이
    같은 자리(568,-390)에서 `OVT:WAIT` 로 굳어 미완주했다. 원인은 목표차선 판정에서
    **폭을 빼면서 여유 0.5m 를 그대로 둔 이중계산**이다. 0.5 는 원래 '폭을 모르니
    이만큼 봐준다'는 대체값이었다.

        차로폭 3.16m · 장애물 fy=-0.2 · 폭 1.8
        |(-0.2) - (-3.16)| - 1.8/2 = 2.06   vs   1.75 + 0.5 = 2.25   -> '막힘'

    **19cm 차이로, 추월하려는 그 차 자신이 자기 추월차선을 막았다.** 차로폭이 3.5m 였던
    기존 시험은 여유가 35cm 라 안 걸렸다 — 좁은 차로에서만 터진다."""
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.16, side=-1.0)
    car = [(1, 15.0, -0.2, 0.0, 4.4, 1.8, 1.5)]      # 내 차로 한복판의 정지차
    assert ov.side_clear(car, 0.0, 0.0, -1.0) is True, "추월 대상이 목표차선을 막는다고 본다"
    assert ov.side_clear(car, 0.0, 0.0, +1.0) is True, "반대쪽도 비어 있다"


def test_차로_반폭은_지도값을_따른다():
    """실측 2026-08-26 EV_BOTHBLOCK: 260초 정지·미완주. `lane_half` 가 **1.75m 고정**이라
    3.16m 차로에서 목표차선을 15cm 넓게 봤고, 그 15cm 때문에 **내 차로의 정지차가
    옆 차선을 막는 것으로** 판정됐다.

        좌측 목표차선 중심 W=+3.16 · 내 차로 정지차 dpath=+0.30 · 경로축 폭 2.4
          고정 1.75 -> 안쪽 경계 1.41 : 정지차 왼쪽 끝 1.50 이 넘는다 -> '막힘'(오판)
          실제 1.58 -> 안쪽 경계 1.58 : 안 넘는다 -> 비어 있다(정답)

    지도(`차로계획.w`)가 폭을 알고 있으니 상수를 쓰지 않는다."""
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.5, side=+1.0)
    assert ov.lane_half == 1.75, "생성자 기본값이 바뀌었다 — 이 시험의 전제가 깨진다"
    ov.set_lane_width(3.16)
    assert abs(ov.lane_half - 1.58) < 1e-9, f"지도 폭을 안 따른다 ({ov.lane_half})"
    # 내 차로 한복판에 비스듬히 선 차(경로축 폭 2.4). 좌측 차로는 비어 있어야 한다.
    tilted = [(1, 15.0, 0.30, 0.0, 4.4, 2.4, 1.5)]
    assert ov.side_clear(tilted, 0.0, 0.0, +1.0) is True, "내 차로의 차가 옆 차선을 막는다고 본다"
    # ⚠️ 그래도 **정말 걸치는** 차는 막아야 한다(코스 H 의 4.4m 짜리는 훨씬 더 나와 있다).
    reaching = [(1, 8.7, -0.3, 0.0, 5.2, 4.4, 1.5)]
    assert ov.side_clear(reaching, 0.0, 0.0, +1.0) is False, "몸통이 걸치는데 비었다고 본다"
    # 기동 중에는 안 바뀐다 — 나가 있는 도중에 기준이 흔들리면 안 된다.
    ov.state = "PASS"
    ov.set_lane_width(2.5)
    assert abs(ov.lane_half - 1.58) < 1e-9, "기동 중에 반폭이 바뀌었다"


def test_옆차로_정지차를_내차로_막힘으로_보지_않는다():
    """실측 2026-08-26 EV_BOTHBLOCK(로그의 rw·dh 로 확인, 각도 추측 아님): 260초 미완주.

        **옆 차로** 정지차 d=-3.16 · 경로축 폭 2.78(dh -14°) · lane_half 1.58
          3.16 - 2.78/2 = 1.77  <  1.58 + 0.3 = 1.88   -> '내 차로를 막는다'(오판)

    폭을 빼면서 여유 0.3 을 그대로 둔 **이중계산**이다 — `_target_lane_clear` 와 똑같은
    병인데 형제 함수를 놓쳤다. 그 차 뒤에 선 채(OBSTACLE_STOP) 앞지르기도 못 했다:
    `min_pass_gap` 이 그 차까지의 6.9m 를 막았기 때문이다. 좌측은 내내 비어 있었다."""
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.16, side=+1.0)
    ov.set_lane_width(3.16)                                   # lane_half 1.58
    aside = [(1, 6.9, -3.16, 0.0, 4.71, 2.78, 1.4)]           # 옆 차로에 비스듬히 선 차
    assert ov._own_lane_block(aside, d_ego=0.0) is None, "옆 차로 차가 내 차로를 막는다고 본다"
    # ★그래도 **내 차로**의 차는 막아야 한다.
    mine = [(2, 15.0, 0.30, 0.0, 4.4, 1.85, 1.4)]
    assert ov._own_lane_block(mine, d_ego=0.0) is not None, "내 차로의 정지차를 놓친다"
    # ★비스듬해서 몸통이 걸쳐 들어오는 차도 막아야 한다(코스 H).
    tilted = [(3, 12.0, -2.2, 0.0, 5.9, 4.4, 1.5)]
    assert ov._own_lane_block(tilted, d_ego=0.0) is not None, "걸쳐 들어온 차를 놓친다"
    # ★내가 옆으로 나가 있어도 **차로 중심 기준**이다(EV_LEADBRAKE 접촉 재발 방지).
    center = [(4, 10.0, -3.35, 0.0, 4.4, 1.8, 1.5)]           # 경로 절대 d = 0.00
    assert ov._own_lane_block(center, d_ego=3.35) is not None, "차로 한복판의 차를 놓친다"


def test_사람은_추월대상이_아니다():
    """실측 2026-08-26 코스 A (1391,-184): **교차로 안 신호 횡단보도 위**에 선 보행자를
    추월 FSM 이 '앞을 막은 차'로 잡았다. 20초 갇힘 탈출이 `allow_start` 를 뚫자
    8.7m 앞에서 우측 우회를 커밋했고, 옆으로 2.47m 를 벌어야 하는데 남은 전진거리가
    2.4m 뿐이라 0.59m 에서 굳었다 — **180초 정지 · 접촉 57프레임 · 미완주**.

    새 나간 이유: 사람 제외 조건이 `max(길이,폭) < 1.2` 인데 **VTD 보행자는
    2.0 x 0.6 x 1.7** 이라 길이에서 탈락한다. behavior 는 원본 치수+높이로 제대로
    사람으로 보는데 FSM 만 못 봤다 — 같은 물체를 두 모듈이 다르게 부르고 있었다."""
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.16, side=+1.0)
    ov.set_lane_width(3.16)
    # (id, ds, d, spd, 경로축길이, 경로축폭, 높이, 원본폭, 원본길이)
    person = [(2, 8.7, 0.02, 0.0, 2.09, 0.66, 1.7, 0.6, 2.0)]
    assert ov._own_lane_block(person, d_ego=0.0) is None, "사람을 추월 대상으로 잡는다"
    # 비스듬히 선 사람도(경로축 폭이 2.1m 까지 부푼다) 마찬가지다
    tilted = [(2, 8.7, 0.02, 0.0, 0.7, 2.11, 1.7, 0.6, 2.0)]
    assert ov._own_lane_block(tilted, d_ego=0.0) is None, "비스듬한 사람을 차로 본다"
    # ★차는 그대로 추월 대상이다
    car = [(3, 8.7, 0.02, 0.0, 4.4, 1.85, 1.4, 1.8, 4.4)]
    assert ov._own_lane_block(car, d_ego=0.0) is not None, "정지차를 놓친다"
    # ★키 낮은 물건(콘·연료통)은 원래대로 nudge 담당 — 여기서 안 잡는다
    cone = [(4, 8.7, 0.02, 0.0, 0.6, 0.6, 0.6, 0.6, 0.6)]
    assert ov._own_lane_block(cone, d_ego=0.0) is None, "콘을 추월 대상으로 잡는다"
    # ★원본 치수가 없는 옛 호출(7칸)도 죽지 않는다
    assert ov._own_lane_block([(5, 8.7, 0.02, 0.0, 4.4, 1.85, 1.4)], d_ego=0.0) is not None


def test_side_clear_는_양쪽을_따로_답한다():
    """실측 2026-08-25 코스 E: 좌측이 **한순간** 막힌 걸 보고 우측으로 뒤집은 뒤
    250초를 섰다. 우측은 정지차 4대가 벽이었고 좌측은 곧 비었는데 못 돌아갔다.
    예전엔 바깥이 `target_clear`(**선택된** 쪽의 결과)만 봐서, 우측을 고른 상태에서
    우측이 막히면 '좌측을 배제하라'는 결론이 매 프레임 되풀이되는 순환이었다.
    -> 고르기 전에 좌·우를 각각 물어볼 수 있어야 한다."""
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.03, side=+1.0)
    # 우측 차로(차로중심 -3.03m)에 정지차 4대가 벽처럼. 좌측은 비었다.
    wall = [(i, fx, -3.0, 0.0, 4.6, 1.8, 1.5)
            for i, fx in enumerate((-11.8, -3.4, 2.3, 8.8), start=10)]
    assert ov.side_clear(wall, 0.0, 0.0, +1.0) is True,  "빈 좌측을 막혔다고 본다"
    assert ov.side_clear(wall, 0.0, 0.0, -1.0) is False, "막힌 우측을 비었다고 본다"
    # ⚠️ 물어본 뒤에도 원래 방향(W)이 그대로여야 한다 — 조회가 상태를 바꾸면 안 된다.
    assert ov.W > 0, f"side_clear 가 W 를 바꿔놨다 ({ov.W})"


def test_차선배치_모르는곳에선_앞지르기_시작_안함():
    """교차로(lane_room=null)에서 나가면 VTD가 정상차선으로 되돌려 리스폰(감점).
    도교법 제22조도 교차로 앞지르기를 금지한다 — 실측 2026-08-15 s≈175/210."""
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.5, side=+1.0)
    ov.state = "WAIT"; ov.block_id = 7
    blocker = (7, 20.0, 0.0, 0.0, 4.4, 1.8, 1.5)     # 우리 차선 20m 앞 정지차

    class E: speed = 0.0
    for _ in range(5):
        ov.plan(E(), [blocker], now=100.0, allow_start=False)
    assert ov.state == "WAIT", ov.state                # 금지 구간 -> 계속 대기
    ov.plan(E(), [blocker], now=100.0, allow_start=True)
    assert ov.state == "WAIT", ov.state                # 허용돼도 지시등 3초 선행(PASS_SIG_LEAD) 뒤에
    ov.plan(E(), [blocker], now=100.0 + ov.PASS_SIG_LEAD + 0.05, allow_start=True)
    assert ov.state == "PASS", ov.state                # 그 뒤엔 나간다


def test_기동중이면_금지구간이어도_중단_안함():
    """반대차선에 나가 있는 상태로 멈추는 게 더 위험하다."""
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.5, side=+1.0)
    ov.state = "PASS"; ov.block_id = 7; ov.offset = 3.5

    class E: speed = 5.0
    off, _, st, _ = ov.plan(E(), [(7, 5.0, 0.0, 0.0, 4.4, 1.8, 1.5)], now=100.0, allow_start=False)
    assert st == "PASS" and abs(off - 3.5) < 1e-6





# ---- 무한정지 탈출 금지 조건 ----
def test_앞에_사람이_있으면_탈출하지_않는다():
    """실측 2026-08-15 EV_PED: 보행자 앞 2.02m 에 제대로 섰는데, 그 지점 신호가 적색이라
    라벨이 RED_STOP 이었고 `reason != YIELD_PED` 가드를 통과해 145초에 걸쳐 기어가 닿았다."""
    st = stack()
    ped = (6.0, 0.03, 0.0, 0.23, 0.40, 1.40, 5)
    assert st._escape_blocked([ped])


def test_앞이_비었으면_탈출한다():
    st = stack()
    assert not st._escape_blocked([])
    far = (30.0, 0.0, 0.0, 4.4, 1.8, 1.5, 5)          # 너무 멀다
    aside = (6.0, 3.5, 0.0, 4.4, 1.8, 1.5, 6)         # 옆으로 비켜 있다
    assert not st._escape_blocked([far, aside])


def test_앞차가_막고_있어도_탈출하지_않는다():
    st = stack()
    car = (8.0, 0.0, 0.0, 4.4, 1.8, 1.5, 7)
    assert st._escape_blocked([car])



def test_경로에_박힌_차선변경도_깜빡이():
    """녹화 경로는 주최측 차량이 실제로 차선을 바꾼 궤적이다(s≈95~125 에서 한 차로 왼쪽).
    우리는 그 선을 그냥 따라가서 오프셋이 0 이라, 지시등 로직이 못 알아채고 안 켰다."""
    from vtd_io import TS_LEFT, TS_RIGHT
    rt = straight_route(400.0)
    plan = [P(need=0.0) for _ in rt]
    for i in range(40, 60):
        plan[i]["sig"] = 1                      # 이 구간은 좌측으로 차선변경 중
    st = DrivingStack(route=rt, lane_plan=plan)

    class S:
        x = 50.0; y = 0.0; heading = 0.0; speed = 8.0
        objects = []; tl_id = 0; tl_state = 0; respawned = False
    cmd = st.step(S(), 0.05, 100.0)
    assert cmd.turn == TS_LEFT, f"깜빡이 안 켬 (turn={cmd.turn})"



def test_못_나가는_동안엔_깜빡이_끈다():
    """양쪽이 다 막혀 하염없이 서 있는데 좌측 깜빡이만 켜고 있던 것(2026-08-15 실측 지적).
    도교법상 진로변경 신호는 '하려는 시점 3초/30m 전'이지 대기 내내가 아니다."""
    from overtake import Overtaker, TS_OFF, TS_LEFT
    ov = Overtaker(lane_width=3.5, side=+1.0)
    ov.state = "WAIT"; ov.block_id = 7

    class E: speed = 0.0
    blocker = (7, 20.0, 0.0, 0.0, 4.4, 1.8, 1.5)          # 내 차선 앞 정지차
    side_block = (8, 15.0, 3.5, 0.0, 4.4, 1.8, 1.5)       # 목표차선도 막힘
    _, turn, _, _ = ov.plan(E(), [blocker, side_block], now=100.0)
    assert turn == TS_OFF, f"못 가는데 깜빡이 켬 ({turn})"

    ov.state = "WAIT"                                      # 목표차선이 비면 켠다
    _, turn2, _, _ = ov.plan(E(), [blocker], now=100.0)
    assert turn2 == TS_LEFT, f"나갈 수 있는데 깜빡이 안 켬 ({turn2})"




def test_사람_우회_중_오프셋이_진동하지_않는다():
    """실측 2026-08-15 EV_PED: 판정에 ego 기준 횡거리를 써서, 비킨 만큼 사람이 창 밖으로
    나가 우회가 풀렸다 -> 오프셋이 3.3 -> 0.85 -> 3.3 으로 진동했고 그 골짜기에서
    사람과의 실여유가 0.42m 까지 좁아졌다.

    ⚠️ **이 테스트는 그 버그를 잡지 못한다.** 수정 전 코드로도 통과한다(확인함) —
       합성 환경에서 진동이 재현되지 않았다. 실제 근거는 VTD 실주행 로그뿐이다
       (수정 후 실여유 0.42m -> 2.13m, 오프셋 3.3 유지). 회귀 감시용 약한 그물로만 둔다.
    """
    from vtd_io import State, Obj
    rt = straight_route(400.0, step=1.0)                # +x 방향 직선, y=0
    st = DrivingStack(route=rt, lane_plan=[P() for _ in rt])

    PED_X, PED_Y = 120.0, 0.0                           # 경로 정중앙에 서 있는 사람
    x, y, t = 60.0, 0.0, 100.0
    offs = []
    for _ in range(400):
        s_ = State(x=x, y=y, heading=0.0)
        s_.speed = 6.0
        s_.objects = [Obj(9, PED_X, PED_Y, 0.0, 0.0, 0.0, 0.23, 0.40, 1.40)]
        cmd = st.step(s_, 0.05, t)
        # 횡은 명령 오프셋을 그대로 따라간다고 가정(추종 지연 없음)
        y += max(-0.1, min(0.1, cmd.lane_offset - y))
        x += max(0.0, cmd.v_cmd) * 0.05
        t += 0.05
        if 0 < PED_X - x < 25.0:                        # 사람이 앞에 있는 동안만 본다
            offs.append(abs(cmd.lane_offset))
    assert offs, "사람 접근 구간이 안 잡힘"
    peak = max(offs)
    assert peak > 1.5, f"우회를 아예 안 함 (peak={peak:.2f})"
    # 일단 벌린 뒤로는 다시 좁아지면 안 된다
    after = offs[offs.index(peak):]
    assert min(after) > peak - 0.6, f"오프셋이 진동함 (peak {peak:.2f} -> {min(after):.2f})"




# ---------------------------------------------------------------- 교차로 안
# 배경(2026-08-16 EV_BOTHBLOCK 실측): 교차로 안에서 앞 13.6m 정지차 뒤에 갇혔는데
# **오른쪽 3.17m 에 같은 방향 빈 차로가 있었다.** 그때 차로계획이 교차로를 통째로
# None(모름)으로 비워둬서 비켜갈 후보로조차 안 봤다. plan_route 가 도로·차로를
# 같이 넘겨주게 되어 이제는 교차로 안도 채워진다 — 대신 **앞지르기 금지는 유지**해야 한다
# (도교법 제22조). 구분: 금지되는 건 '앞지르기'지 '같은 방향 진로변경'이 아니다.

def PJ(**kw):
    """교차로 안 차로계획(j=1)."""
    d = P(**kw)
    d["j"] = 1
    return d


def test_교차로_안에서도_옆차로가_있으면_비킬_방향을_고른다():
    st = stack()
    assert st._side_for(PJ(l=0.5, r=4.65), need=1.42) < 0      # 우측 같은방향 차로로


def test_교차로_안에서는_반대차선을_안_넘는다():
    st = stack()
    # 같은 방향 여유가 없고 xl 만 있는 상황이라도, 교차로 안이면 xl 은 0 이어야 정상.
    # (build_lane_plan 이 교차로 점의 xl/xr 을 0 으로 준다 — 여기선 그 계약을 고정한다)
    assert st._side_for(PJ(l=0.5, r=0.5, xl=0.0, xr=0.0), need=1.42) == 0.0


def test_교차로_안에서는_앞지르기를_시작하지_않는다():
    """plan 이 채워졌다는 이유로 교차로 앞지르기가 열리면 안 된다."""
    import inspect
    src = inspect.getsource(DrivingStack.step)
    assert 'plan.get("j")' in src, "교차로 판정이 j 플래그를 안 본다"
    assert "plan is None or in_junction" in src, "plan is None 만 보면 교차로가 안 걸러진다"


def test_완전히_막혀_오래_갇혀도_기본모드는_불충분한_폭으로_안_나간다():
    st = stack()
    tight = PJ(l=1.0, r=2.0, xl=0.0, xr=0.0)          # 어느 쪽도 need 를 못 채움
    assert st._side_for(tight, need=4.44) == 0.0
    # ★2026-09-08: 갇히면 **같은 방향 차도 안에서** 차 한 대 폭(2.0m)까지는 나간다.
    #   중앙선(xl/xr)은 그대로 잠겨 있다 — 주최측 답변은 거기에만 걸린다.
    assert st._side_for(tight, need=4.44, desperate=True) == -1.0
    # 차 한 대 폭도 없으면 여전히 안 나간다
    narrow = PJ(l=1.0, r=1.36, xl=0.0, xr=0.0)
    assert st._side_for(narrow, need=4.44, desperate=True) == 0.0


def test_실험옵션은_종전의_절박한_넓은쪽_선택을_복원한다():
    st = stack(allow_centerline_escape=True)
    tight = PJ(l=1.0, r=2.0, xl=0.0, xr=0.0)
    assert st._side_for(tight, need=4.44, desperate=True) < 0


def test_갇혀도_나갈_공간_자체가_없으면_안_움직인다():
    st = stack()
    none_ = PJ(l=0.2, r=0.3, xl=0.0, xr=0.0)          # 벽에 붙어 있음
    assert st._side_for(none_, need=4.44, desperate=True) == 0.0



def test_고른_방향이_막혔으면_반대쪽을_고른다():
    """지도는 '차선이 있다'만 안다. 좌우 여유가 같으면 좌측을 고르는데,
    하필 그 좌측이 막혀 있으면 462초를 서 있게 된다(EV_BOTHBLOCK 실측)."""
    st = stack()
    both = PJ(l=4.65, r=4.65)
    assert st._side_for(both, need=4.04) > 0                      # 평소엔 좌측
    assert st._side_for(both, need=4.04, exclude=+1.0) < 0        # 좌측 제외하면 우측


def test_양쪽_다_제외하면_0():
    st = stack()
    both = PJ(l=4.65, r=4.65)
    assert st._side_for(both, need=4.04, exclude=+1.0) < 0
    # 우측도 제외되면(둘 다 막힘) 나갈 데 없음 -> 0
    assert st._side_for(PJ(l=0.2, r=4.65), need=4.04, exclude=-1.0) == 0.0


def test_가까이_붙어_갇히면_추월_최소간격을_완화한다():
    """실측 2026-08-16 EV_COMBO_CHAIN: 앞 장애물이 7.11m 인데 min_pass_gap 이 9.0m 라
    '가까우면 차선변경 금지' 규칙에 걸려 8232프레임(464초) 갇혔다. 후진은 못 한다."""
    import types
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.3)
    ego = types.SimpleNamespace(speed=0.0)
    # 7.5m 앞 정지차 1대, 목표차선은 비어 있음
    objs = [(1, 7.5, 0.0, 0.0, 4.4, 1.8, 1.5)]
    t = 1000.0
    ov.plan(ego, objs, now=t)                       # FOLLOW -> WAIT
    assert ov.state == "WAIT", ov.state
    ov.plan(ego, objs, now=t + 1.0)
    assert ov.state == "WAIT", "갇힌 직후엔 아직 안 나간다(9m 규칙)"
    ov.plan(ego, objs, now=t + ov.stuck_relax + 3.0)
    ov.plan(ego, objs, now=t + ov.stuck_relax + 3.0 + ov.PASS_SIG_LEAD + 0.05)   # 지시등 3초 선행
    assert ov.state == "PASS", f"오래 갇혔으면 나가야 한다 (state={ov.state})"


def test_달리는_중엔_최소간격을_안_낮춘다():
    import types
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.3)
    objs = [(1, 7.5, 0.0, 0.0, 4.4, 1.8, 1.5)]
    t = 2000.0
    ov.plan(types.SimpleNamespace(speed=0.0), objs, now=t)
    for k in range(1, 40):                          # 계속 굴러가는 중이면 완화 없음
        ov.plan(types.SimpleNamespace(speed=3.0), objs, now=t + k)
    assert ov.state == "WAIT", f"주행 중엔 7.5m 에서 틀면 안 된다 (state={ov.state})"


def test_적신호에_서도_추월을_취소하지_않는다():
    """실측 2026-08-17 공식 v3 (7판 중 3판 미완주, 그 정지차를 만나면 100% 실패).

        t=23.7  OVT:PASS   off 3.50   cap_by=RED_STOP   <- 적신호에 선다
        t=27.7  OVT:RETURN                              <- 4초 지나 추월 취소
        t=28.0  off 0.00 -> 차로 중심 = 그 차가 서 있는 자리
        t=34~   재출발해서 그대로 박고 1026초 미완주

    세운 건 안전망이 아니라 **신호**였다. '멈췄으니 추월을 포기'가 그대로 충돌 명령이
    된다. 차로 중심이 아직 막혀 있으면 서 있어도 나가 있는 상태를 유지해야 한다.
    """
    import types
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.3)
    blocked = [(1, 7.5, 0.0, 0.0, 4.4, 1.8, 1.5)]      # 차로 중심 7.5m 앞 정지차
    stopped = types.SimpleNamespace(speed=0.0)
    t = 5000.0
    ov.plan(stopped, blocked, now=t)
    ov.plan(stopped, blocked, now=t + 1)
    ov.plan(stopped, blocked, now=t + ov.stuck_relax + 3)
    t += ov.PASS_SIG_LEAD + 0.05                                # 지시등 3초 선행
    ov.plan(stopped, blocked, now=t + ov.stuck_relax + 3)
    assert ov.state == "PASS", ov.state
    ov.plan(stopped, blocked, now=t + ov.stuck_relax + 3.1)   # 다음 프레임에 오프셋이 열린다
    assert abs(ov.offset) > 1.0, f"추월 오프셋이 안 열렸다 ({ov.offset})"
    # 적신호에 abort_sec 를 한참 넘겨 서 있어도 유지해야 한다
    for k in range(1, 30):
        ov.plan(stopped, blocked, now=t + ov.stuck_relax + 3 + k)
    assert ov.state == "PASS", f"적신호에 섰다고 추월이 취소됐다 (state={ov.state})"
    assert abs(ov.offset) > 1.0, f"오프셋이 무너졌다 ({ov.offset})"


def test_이전_PASS_잔여타이머가_다음_PASS를_즉시_취소하면_안된다():
    """실측 2026-08-16 EV_COMBO_CHAIN: PASS 로 나가자마자 RETURN -> 8초 쿨다운 ->
    다시 대기 -> PASS -> 즉시 취소... 무한 루프. _pass_stop_since 가 이전 시도의
    값을 그대로 들고 있어서였다. 정지차 뒤에서 서 있다 나가는 게 정상 상황이다."""
    import types
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.3)
    objs = [(1, 7.5, 0.0, 0.0, 4.4, 1.8, 1.5)]
    stopped = types.SimpleNamespace(speed=0.0)
    t = 3000.0
    ov.plan(stopped, objs, now=t)                      # -> WAIT
    ov.plan(stopped, objs, now=t + 1)
    ov.plan(stopped, objs, now=t + ov.stuck_relax + 3) # 조건 충족 -> 지시등
    t += ov.PASS_SIG_LEAD + 0.05                       # 3초 선행
    ov.plan(stopped, objs, now=t + ov.stuck_relax + 3) # -> PASS
    assert ov.state == "PASS", ov.state
    # 서 있는 채로 abort_sec 를 넘기면 중단(정상).
    #  ★단 **차로가 비었을 때만** 이다. 막는 차가 아직 차로 중심에 있으면 복귀가 곧
    #    충돌이라 유지한다(아래 test_적신호에_서도_추월을_취소하지_않는다).
    cleared = [(1, 7.5, 3.4, 0.0, 4.4, 1.8, 1.5)]      # 그 차가 차로 밖으로 비켰다
    for k in range(1, 12):
        ov.plan(stopped, cleared, now=t + ov.stuck_relax + 3 + k)
    assert ov.state in ("RETURN", "FOLLOW", "WAIT"), ov.state
    # ★다시 나갈 수 있어야 한다 — 잔여 타이머로 첫 프레임에 죽으면 안 된다
    t2 = t + 200.0
    for k in range(0, 40):
        ov.plan(stopped, objs, now=t2 + k)
        if ov.state == "PASS":
            break
    assert ov.state == "PASS", f"다시 나가지 못한다 (state={ov.state})"
    ov.plan(stopped, objs, now=t2 + k + 0.05)
    assert ov.state == "PASS", "PASS 첫 프레임에 잔여 타이머로 취소됐다"


def test_완화해도_닿을_거리에서는_안_나간다():
    """★완화가 충돌을 만들면 안 된다. 실측 2026-08-16 EV_COMBO_CHAIN: 하한을
    pass_clear(6.0m)로 낮췄더니 fx=6.11m 에서 나가 clr=-0.01 로 접촉, 390초간 물렸다.
    fx 는 **뒷축~상대차 중심** 이라 실제 범퍼 간격 = fx - 반길이 - 앞오버행(3.808)."""
    import types
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.3)
    stopped = types.SimpleNamespace(speed=0.0)
    olen = 4.4
    touch = ov.front_overhang + olen / 2.0            # = 6.0m, 실제 간격 0
    objs = [(1, touch + 0.1, 0.0, 0.0, olen, 1.8, 1.5)]
    t = 5000.0
    for k in range(0, 40):
        ov.plan(stopped, objs, now=t + k)
    assert ov.state != "PASS", f"닿을 거리인데 나갔다 (fx={touch+0.1:.2f}, state={ov.state})"
    # 여유 1m 를 확보한 거리에서는 나간다
    ov2 = Overtaker(lane_width=3.3)
    objs2 = [(1, ov2.front_overhang + olen / 2.0 + ov2.pass_margin + 0.1, 0.0, 0.0, olen, 1.8, 1.5)]
    for k in range(0, 40):
        ov2.plan(stopped, objs2, now=t + k)
        if ov2.state == "PASS":
            break
    assert ov2.state == "PASS", f"여유가 있는데도 안 나간다 (state={ov2.state})"


def test_비킬_자리에_다른_물체가_있으면_그리로_가지_않는다():
    """실측 2026-08-18 EV_COMBO_PASSPED — 1027초 영구정지의 **성질**을 재현한다.

    현장 값 그대로는 못 만든다. 회피가 시작된 t≈42 시점의 **경로기준** 물체 좌표를
    로그에 안 남겼기 때문이다(`objs` 는 ego 기준). 그래서 같은 구조를 합성했다:

        보행자  d_path = +1.5m  -> 회피 대상(필요 2.64m 안)  -> 우측으로 피한다
        정지차  d_path = -3.18m -> 차량이라 nudge 는 건너뛴다
        차로폭 3.19 · 좌여유 1.57 · 우여유 7.93 · 중앙선 실선(xl=xr=0)

    우여유가 넉넉하니 `want = -3.19`(차로 하나) 가 나오는데, **거기 정지차가 있다.**
    현장에서 나온 -3.17 과 사실상 같은 값이고, 정지차는 -3.18 이었다.
    behavior 는 정당하게 NARROW_BLOCK 을 걸고 nudge 는 계속 그리로 가라 해서 교착이 된다.

    ★요구사항: 비키라고 낸 오프셋이 **아는 물체와 겹치면 안 된다.**
      겹칠 수밖에 없으면 0(안 비킴)이어야 한다 — 그때는 서는 게 맞다.
    ⚠️ 이 테스트는 아직 **실패한다.** 고치려고 네 번 시도해 네 번 다 악화시켰다
      (README '미해결 ①'). 고칠 때 이 테스트로 확인할 것.
    """
    st = stack()
    plan = P(l=1.57, r=7.93, xl=0.0, xr=0.0, w=3.19)
    ped = (12.0, 1.50, 0.0, 0.20, 0.40, 1.40, 201)       # 회피 대상(사람)
    car = (14.9, -3.18, 0.0, 4.40, 1.80, 1.40, 301)      # 우측 정지차
    off = settle(st, [ped, car], plan=plan, ped=True)
    gap = abs(off - (-3.18)) - 0.943 - 0.90              # 내 반폭 + 차 반폭
    assert off == 0.0 or gap >= 0.30, (
        f"정지차(-3.18m) 자리로 비켰다: off={off:.2f}, 여유={gap:+.2f}m")


# ---- 교차로 꼬리물기 금지 [도교법 제25조⑤] ----
def _junction_plan(n=400, j0=50, j1=70, w=3.3, real=True):
    """경로점 j0~j1 이 교차로인 차로계획.

    `real=False` 면 **갈래 2개짜리 모퉁이**(jx=0) — xodr 이 junction 으로 표시하지만
    가로지를 교통이 없는 굽은 길이다. 이 맵은 junction 94개 중 41개가 그렇다.
    """
    return [dict(lane=-1, w=w, l=5.0, r=5.0, xl=0.0, xr=0.0, need=0.0, sig=0,
                 lim=13.889, j=1 if j0 <= i < j1 else 0,
                 jx=1 if (real and j0 <= i < j1) else 0) for i in range(n)]


def _state(x, v, objs):
    from vtd_io import State, Obj
    s = State()
    s.x, s.y, s.heading, s.speed = x, 0.0, 0.0, v
    s.objects = [Obj(i + 1, ox, oy, 0.0, 0.0, osp, 4.6, 1.8, 1.5)
                 for i, (ox, oy, osp) in enumerate(objs)]
    return s


def _captured_overtake_gate(st, state, now=100.0):
    seen = {}

    def fake_plan(ego, objs, **kw):
        seen.update(kw)
        return 0.0, 0, "FOLLOW", None

    st.overtaker.plan = fake_plan
    st._stalled_since = 1.0                         # stuck_long=True
    st.step(state, 0.04, now)
    return seen["allow_start"], seen["hold"]


def test_plan_none이면_예전대로_기본_방향을_준다():
    """★2026-09-08. 차로계획이 없으면 중앙선 위치를 모른다 — 그래서 막고 싶어진다.

    그런데 **실주행에서는 여기로 오지 않는다.** `drive.sh` 가 언제나 차로계획을 넘긴다.
    여기로 오는 건 오프라인 모의뿐이고, 모의에는 차로계획이 아예 없어 모든 판이 여기를 탄다.
    막으니 회귀 4판이 미완주가 됐고, '갇혔을 때만' 으로 절충해도 20초 뒤에야 풀려
    40초짜리 판은 못 끝냈다. 유일한 안전망을 잃는 것보다 예전 동작이 낫다.
    중앙선 정책은 `xl/xr` 잠금이 지킨다 — 차로계획이 **있을 때** 작동한다.
    """
    st = stack()
    assert st._side_for(None, need=4.44) == st.default_side
    assert st._side_for(None, need=4.44, desperate=True) == st.default_side

def test_교차로에서_오래_갇혀도_새_추월을_시작하지_않는다():
    st = DrivingStack(route=straight_route(400.0), lane_plan=_junction_plan())
    allow, hold = _captured_overtake_gate(st, _state(60.0, 0.0, []))
    assert allow is False and hold is True


def test_빠져나올_자리가_없으면_교차로에_안_들어간다():
    """실측 2026-08-25 코스 H: 녹색에 진입했는데 5m 앞에 정지차가 있어 **교차로 한복판에
    153초** 섰다. 교차 교통을 통째로 막았고, 교차로 안이라 앞지르기도 금지(제22조)라
    스스로 못 풀었다. 제25조⑤ — 빠져나올 자리가 없으면 들어가면 안 된다."""
    st = DrivingStack(route=straight_route(400.0), lane_plan=_junction_plan())
    # 교차로는 x=50~70. 자차 x=40 -> 진입까지 10m. 교차로 출구(70) 바로 뒤 x=73 에 정지차.
    s = _state(40.0, 8.33, [(73.0, 0.0, 0.0)])
    cmd = st.step(s, 0.04, 1.0)
    assert cmd.reason == "JUNCTION_JAM", f"꼬리물기인데 그냥 들어간다 ({cmd.reason})"
    assert cmd.v_cmd < 8.33, f"감속을 안 한다 ({cmd.v_cmd * 3.6:.1f}km/h)"
    # ⚠️ 10m 앞에서 14km/h 는 정상이다 — 정지 프로필은 거리에 비례한다.
    #    **진입 직전에 0 이 되는지**가 진짜 조건이다.
    st2 = DrivingStack(route=straight_route(400.0), lane_plan=_junction_plan())
    near = _state(48.5, 2.0, [(73.0, 0.0, 0.0)])     # 교차로(50) 1.5m 앞
    c2 = st2.step(near, 0.04, 1.0)
    assert c2.v_cmd < 0.6, f"교차로 코앞인데 안 선다 ({c2.v_cmd * 3.6:.1f}km/h, {c2.reason})"


def test_출구_너머가_비어_있으면_그냥_들어간다():
    """너무 넓게 잡으면 멀쩡한 교차로마다 선다 — 자차 길이+여유만큼만 본다."""
    st = DrivingStack(route=straight_route(400.0), lane_plan=_junction_plan())
    # 정지차가 출구(70)에서 자차길이(4.85)+여유(2.0) 보다 더 멀리(x=80) 있으면 통과 가능
    s = _state(40.0, 8.33, [(80.0, 0.0, 0.0)])
    cmd = st.step(s, 0.04, 1.0)
    assert cmd.reason != "JUNCTION_JAM", f"빠져나갈 수 있는데 막는다 ({cmd.reason})"


def test_긴_교차로에서_출구가_비면_들어간다():
    """실측 2026-08-26 코스 E: 그 교차로가 **길이 49.6m** 였다(H 는 18.1m).
    검사 구간을 `진입~출구+여유` 로 잡으면 **56m** 가 되고, 주변교통 50대면 그 안에
    늘 누군가 서 있어 **영영 진입 못 한다**(294초 정지). 법이 묻는 건
    '출구 너머에 내가 설 자리가 있나'지 '56m 가 다 비었나'가 아니다."""
    st = DrivingStack(route=straight_route(400.0), lane_plan=_junction_plan(j0=50, j1=100))
    # 교차로 x=50~100(50m). 정지차는 교차로 **한복판**(x=70) — 출구와 30m 떨어져 있다.
    s = _state(45.0, 8.33, [(70.0, 0.0, 0.0)])
    cmd = st.step(s, 0.04, 1.0)
    assert cmd.reason != "JUNCTION_JAM", f"출구가 비었는데 막는다 ({cmd.reason})"


def test_긴_교차로도_출구가_막히면_안_들어간다():
    """좁혔다고 느슨해지면 안 된다 — 출구 바로 뒤가 막히면 여전히 진입 금지."""
    st = DrivingStack(route=straight_route(400.0), lane_plan=_junction_plan(j0=50, j1=100))
    s = _state(45.0, 8.33, [(103.0, 0.0, 0.0)])       # 출구(100) 3m 뒤
    cmd = st.step(s, 0.04, 1.0)
    assert cmd.reason == "JUNCTION_JAM", f"출구가 막혔는데 들어간다 ({cmd.reason})"


def test_움직이는_차는_꼬리물기로_안_본다():
    """앞차가 굴러가고 있으면 곧 비워준다. 서 있는 차만 본다."""
    st = DrivingStack(route=straight_route(400.0), lane_plan=_junction_plan())
    s = _state(40.0, 8.33, [(73.0, 0.0, 6.0)])       # 6m/s 로 진행 중
    cmd = st.step(s, 0.04, 1.0)
    assert cmd.reason != "JUNCTION_JAM", f"움직이는 차에 막힌다 ({cmd.reason})"


def test_모퉁이는_교차로가_아니다():
    """★2026-08-30 사용자 지적 코스 E (785,571): "길 자체가 그냥 휘어진 도로다.
    좌회전이 아니라 그냥 길이 굽은 도로".

    xodr 은 두 도로가 만나 꺾이는 **모퉁이**도 junction 으로 표시한다. 이 맵은
    junction 94개 중 **41개(44%)가 갈래 2개**뿐이고, 코스 A 는 j=1 점 699개 중
    **259개(37%)** 가 그런 모퉁이다. junction 91 = incomingRoad [2813,2814] 둘뿐,
    road 3099 는 52.7m 에 93° 꺾이는 굽은 길이다.

    그걸 교차로로 세면 커브에서 **회전 지시등**이 켜지고 `route_turn>0` 이 되면서
    **대향차 양보(YIELD_CROSS)** 까지 걸려 26.9->0.0km/h 로 섰다.
    -> `jx`(갈래 3개 이상)만 교차로로 본다.
    """
    # ① 꼬리물기: 모퉁이면 출구가 막혀 있어도 진입을 막지 않는다
    st = DrivingStack(route=straight_route(400.0),
                      lane_plan=_junction_plan(real=False))
    cmd = st.step(_state(40.0, 8.33, [(73.0, 0.0, 0.0)]), 0.04, 1.0)
    assert cmd.reason != "JUNCTION_JAM", f"모퉁이에 꼬리물기를 걸었다 ({cmd.reason})"

    # ② 회전 지시등·양보: 모퉁이에서는 _upcoming_turn 이 0 이어야 한다
    import math as _m
    n = 120
    rt = []
    for i in range(n):                      # 35m 안에 60° 꺾이는 굽은 길
        a = _m.radians(min(60.0, i * 1.2))
        rt.append((sum(_m.cos(_m.radians(min(60.0, k * 1.2))) * 1.5 for k in range(i)),
                   sum(_m.sin(_m.radians(min(60.0, k * 1.2))) * 1.5 for k in range(i))))
        del a
    corner = DrivingStack(route=rt, lane_plan=_junction_plan(n=n, j0=0, j1=n, real=False))
    real = DrivingStack(route=rt, lane_plan=_junction_plan(n=n, j0=0, j1=n, real=True))
    assert corner._upcoming_turn(2) == 0, "모퉁이를 회전으로 봤다 — 커브 오점등"
    assert real._upcoming_turn(2) != 0, "진짜 교차로인데 회전으로 안 봤다"




def _turn_lead(name):
    """그 코스의 회전 지시등 선행거리[m] 목록. 회전 시작점 = 국소 10m 방위변화 8° 돌파."""
    import json as _j, os as _o, math
    R = _o.path.join(_o.path.dirname(_o.path.abspath(__file__)), "..", "routes")
    rt = [tuple(p[:2]) for p in _j.load(open(f"{R}/{name}.json", encoding="utf-8"))["ego_route"]]
    lp = _j.load(open(f"{R}/{name}_lane.json", encoding="utf-8"))["pts"]
    st = DrivingStack(route=rt, lane_plan=lp)
    cum = st.cum

    def hd(k):
        a = rt[max(0, k - 3)]; b = rt[min(len(rt) - 1, k + 3)]
        return math.atan2(b[1] - a[1], b[0] - a[0])

    on = [st._signal_turn(i) for i in range(len(rt))]
    runs = []
    for i, v in enumerate(on):
        if not v:
            continue
        if runs and i - runs[-1][1] <= 3:
            runs[-1][1] = i
        else:
            runs.append([i, i])
    out = []
    for a, b in runs:
        if cum[a] < 5.0:
            continue                       # 경로가 회전 안에서 시작 — 선행을 줄 자리가 없다
        st_i = b
        for k in range(a, b + 1):
            j = k
            while j < len(rt) - 1 and cum[j] - cum[k] < 10.0:
                j += 1
            if abs(math.degrees((hd(j) - hd(k) + math.pi) % (2*math.pi) - math.pi)) > 8.0:
                st_i = k
                break
        out.append(cum[st_i] - cum[a])
    return out


def test_회전_지시등은_30m_앞에서_켠다():
    """[법 제38조① · 시행령 별표2] 회전하려는 지점 30m 이상 전에 신호해야 한다.

    실측 2026-08-30(고치기 전): 전 코스 회전 65곳의 선행 **중앙값 21.0m**, 30m 를
    넘는 곳이 하나도 없었다(최소 0.0m). 사용자 지적 "깜빡이 늦게 킴".
    원인은 판단(`_upcoming_turn`, 35m 창)으로 점등까지 하고 있었던 것 — 코너가
    급할수록 창이 코너를 물어야 문턱을 넘으므로 선행이 저절로 짧아진다.
    """
    leads = []
    for c in ("HL_FMA_NEW_A", "HL_FMA_NEW_D", "HL_FMA_NEW_E", "HL_FMA_NEW_H"):
        leads += _turn_lead(c)
    assert leads, "회전을 하나도 못 찾았다"
    leads.sort()
    mid = leads[len(leads) // 2]
    assert mid >= 30.0, f"선행 중앙값 {mid:.1f}m < 30m"
    # 연달아 붙은 회전(앞 회전이 안 끝났는데 다음이 시작)은 30m 를 못 줄 수 있다.
    short = [x for x in leads if x < 30.0]
    assert len(short) <= len(leads) * 0.15, f"30m 미만이 {len(short)}/{len(leads)}곳"


def test_점등용_판정은_판단용보다_커브에_둔감하다():
    """`_signal_turn` 은 22m 창, `_upcoming_turn` 은 35m 창 — 같은 25° 문턱이면
    짧은 창이 **더 급한 코너만** 잡는다. '커브에서 깜빡이 켠다' 재발 방지."""
    assert DrivingStack.TURN_SIG_WIN < DrivingStack.TURN_SIGNAL_AHEAD
    assert DrivingStack.TURN_SIG_LEAD >= 30.0


def _turn_queue_stack(sign=-1):
    """x=60 에서 90도 회전(sign=-1 우회전 / +1 좌회전). x=58~64 만 실제 교차로로 표시한다."""
    route = [(float(x), 0.0) for x in range(61)]
    route += [(60.0, sign * float(y)) for y in range(1, 81)]
    plan = [dict(lane=-1, w=3.3, l=5.0, r=5.0, xl=0.0, xr=0.0,
                 need=0.0, sig=0, lim=13.889, j=1 if 58 <= i <= 64 else 0,
                 jx=1 if 58 <= i <= 64 else 0)
            for i in range(len(route))]
    return DrivingStack(route=route, lane_plan=plan)


def _right_turn_queue_stack():
    """x=60 에서 남쪽으로 90도 우회전. x=58~64 만 실제 교차로로 표시한다."""
    return _turn_queue_stack(-1)


def _queue_run(st, secs, blocker_moving_s=0.0, x_ego=21.0, x_blk=41.0, t0=100.0):
    """자차는 선 채로, 앞차는 `x_blk` 에 선 채로 `secs` 초를 돌린다.

    `blocker_moving_s > 0` 이면 그 시간 동안 앞차가 **달려와서** 그 자리에 선다.
    추월기가 그 차를 `was_moving`(달리다 멈춘 차 = 신호 대기일 수 있다)으로
    분류하게 만드는 게 목적이다 — 처음부터 서 있던 차와 구분되는지 보려면
    이 구분을 실제로 만들어 줘야 한다.

    돌려주는 각 행은 `(경과초, ov_state, lane_offset, turn)` 이다. 상태만 보면
    부족하다 — `WAIT` 는 오프셋 0 이라 차는 안 움직이는데 **지시등이 거짓말**을
    하고, 반대로 게이트가 풀린 직후에도 선행점등 3초 동안은 아직 안 나간다.
    """
    from vtd_io import State, Obj

    ego = State(x=x_ego, y=0.0, heading=0.0, speed=0.0)
    out, dt, t, v = [], 0.05, t0, 4.0
    for _ in range(int(secs / dt)):
        el = t - t0
        if el < blocker_moving_s:                       # 달려오는 중
            bx, bs = x_blk - v * (blocker_moving_s - el), v
        else:                                           # 그 자리에 섰다
            bx, bs = x_blk, 0.0
        ego.objects = [Obj(7, bx, 0.0, 0.0, 0.0, bs, 4.4, 1.8, 1.5)]
        c = st.step(ego, dt, t)
        out.append((round(el, 2), c.ov_state, c.lane_offset, c.turn))
        t += dt
    return out


def test_우회전_앞_정지차_때문에_옆차로로_나가지_않는다():
    """우회전 대기 차량을 고정 장애물로 보고 추월했다가 다시 들어오는 회귀 방지.

    교차로는 39m 앞이라 기존 25m 금지창 밖이지만, 추월 왕복 기동을 시작하기에는
    늦은 위치다.
    """
    from vtd_io import State, Obj, TS_RIGHT

    st = _right_turn_queue_stack()
    assert st._junction_ahead(21) > st.JUNC_NO_PASS       # 옛 25m 게이트로는 허용
    assert st._turn_within(21, st.OVERTAKE_TURN_GUARD) < 0

    ego = State(x=21.0, y=0.0, heading=0.0, speed=0.0)
    ego.objects = [Obj(7, 41.0, 0.0, 0.0, 0.0, 0.0, 4.4, 1.8, 1.5)]
    cmds = [st.step(ego, 0.05, 100.0 + k * 0.05) for k in range(8)]
    for c in cmds:
        assert c.ov_state != "PASS", f"우회전 앞에서 추월을 시작함(off={c.lane_offset:.2f})"
        assert abs(c.lane_offset) < 0.05, c.lane_offset

    # ★★**`WAIT` 에도 들어가면 안 된다.** `WAIT` 의 지시등은 추월할 쪽을 켠다
    #   (`overtake.set_side` 주석) — 우회전하려는 참에 **좌측**이 켜진다(항목13).
    #   `allow_start=False` 만으로는 이걸 못 막는다. `ov_hold` 로 막아야 한다.
    #   실측(고치기 전): 두 번째 프레임부터 계속 좌측이었다.
    assert all(c.ov_state == "FOLLOW" for c in cmds), [c.ov_state for c in cmds]
    assert all(c.turn == TS_RIGHT for c in cmds[1:]), [c.turn for c in cmds]

    # 앞차가 출발해 관측에서 사라져도 옆차선에서 복귀하는 동작 자체가 없어야 한다.
    ego.objects = []
    offsets = [st.step(ego, 0.05, 100.4 + k * 0.05).lane_offset for k in range(4)]
    assert max(map(abs, offsets)) < 0.05, offsets


def test_우회전_앞이어도_오래_갇히면_빠져나간다():
    """★우회전 금지창을 `stuck_long` 으로도 안 풀면 **미완주가 확정된다.**

    재현(고치기 전): 20m 앞 정지차 · 우회전 39m 앞 → **300초를 돌려도 `WAIT` ·
    오프셋 0.00** 으로 영영 못 나갔다. 그 60m 창이 경로에서 차지하는 비율은
    코스 A 21.7%(926m) · G 26.3%(903m) · B 16.6% · D 16.0% · H 9.7% · E 5.0% 다.
    거기에 죽은 NPC 가 하나 서면 0점이다.

    교차로 앞지르기 금지(제22조)는 법으로 더 센 금지인데도 `stuck_long` 이면 풀어 준다 —
    이것만 그보다 엄격할 이유가 없다.
    """
    from vtd_io import State, Obj

    st = _right_turn_queue_stack()
    ego = State(x=21.0, y=0.0, heading=0.0, speed=0.0)
    ego.objects = [Obj(7, 41.0, 0.0, 0.0, 0.0, 0.0, 4.4, 1.8, 1.5)]
    t = 100.0
    seen = set()
    # JUNCTION_WAIT(20초)를 넘겨 갇힌 뒤에도 계속 돌린다
    for _ in range(int(90.0 / 0.05)):
        c = st.step(ego, 0.05, t)
        t += 0.05
        seen.add(c.ov_state)
    assert "PASS" in seen, f"오래 갇혔는데도 못 나갔다 (거친 상태: {sorted(seen)})"


def test_좌회전_앞_정지차_때문에도_옆차로로_나가지_않는다():
    """★게이트가 **우회전만** 보고 있었다 — 좌회전 대기열은 그대로 뚫려 있었다.

    좌회전 대기는 신호 한 주기를 통째로 기다리므로 우회전보다 오히려 **더 길게**
    서 있다. 오인 추월이 나기 더 쉬운 쪽인데 무방비였다.
    실측 2026-09-08(고치기 전) — 우회전과 똑같은 배치를 좌회전으로 뒤집었을 때:
      우회전: `FOLLOW` 유지 · 오프셋 0.00
      좌회전: `WAIT` -> **3.15초에 `PASS`** · 오프셋 **3.30m**(한 차로 통째)

    창을 좌회전까지 넓히는 비용은 작다 — 60m 창이 경로에서 차지하는 비율
    (실측 2026-09-08, `_turn_within` 으로 전 코스 적분):
      우회전 A 20.9% · B 16.5% · D 15.6% · E 4.9% · G 26.0% · H 9.4%
      좌회전 A  2.1% · B  3.4% · D  3.8% · E 7.1% · G  0.0% · H 10.3%
    좌회전을 넣어도 창은 대부분 +2~4%p, 최대 +10%p 다.

    ⚠️ `_turn_within` 은 **자차 경로가 회전할 때만** 0 이 아니다. 자차가 직진인데
       옆에 좌회전 전용차로가 있는 경우는 애초에 안 걸린다 — 그건 막는 게 아니다.
    """
    from vtd_io import TS_LEFT

    st = _turn_queue_stack(+1)
    assert st._junction_ahead(21) > st.JUNC_NO_PASS       # 옛 25m 게이트로는 허용
    assert st._turn_within(21, st.OVERTAKE_TURN_GUARD) > 0

    out = _queue_run(st, 15.0)              # 갇힘 탈출 문턱에는 못 미치는 길이
    for el, state, off, _t in out:
        assert state == "FOLLOW", f"t={el}s 좌회전 앞에서 {state} (off={off:.2f})"
        assert abs(off) < 0.05, f"t={el}s off={off:.2f}"
    # 우회전 쪽과 같은 이유로 지시등도 확인한다 — `WAIT` 로 새면 추월할 쪽이 켜진다.
    assert all(r[3] == TS_LEFT for r in out[1:]), [r[3] for r in out[:10]]


def test_신호_대기_앞차는_20초_갇힘만으로는_추월하지_않는다():
    """★★갇힘 탈출(20초)이 **신호 대기차와 죽은 차를 구분하지 못했다.**

    실측 2026-09-08(고치기 전), 우회전 39m 앞 · 20m 앞 차량으로 45초:
      처음부터 정지(죽은 NPC)   : 20.1초 `WAIT` -> 23.2초 `PASS`
      **달리다 멈춘 차(신호 대기)**: 20.1초 `WAIT` -> 23.2초 `PASS`  ← **완전히 동일**
    즉 이 게이트가 애초에 막으려던 동작이 20초 뒤엔 그대로 나온다.
    측정된 신호 한 주기는 **36초**다(behavior.py, 2026-09-06 코스 E 신호108) —
    20초로는 한 주기도 못 기다린다.

    추월기는 이미 `was_moving` 으로 둘을 구분하고 있다. 그 구분을 게이트 해제
    문턱에 그대로 쓴다: 달리다 멈춘 차는 `TURN_GUARD_RELAX_MOVED`(신호 한 주기를
    넘기는 길이), 원래 정지물은 종전대로 `JUNCTION_WAIT`.
    """
    st = _turn_queue_stack(-1)
    out = _queue_run(st, 40.0, blocker_moving_s=1.0)
    bad = [r for r in out if r[1] != "FOLLOW" or abs(r[2]) > 0.05]
    assert not bad, f"신호 대기 앞차를 {bad[0][0]}초에 추월하려 한다 ({bad[0][1]}, off={bad[0][2]:.2f})"


def test_신호_한주기를_넘겨도_안_가면_그때는_빠져나간다():
    """★두 단으로 나누되 **긴 쪽도 반드시 유한**해야 한다.

    달리다 멈춘 차라도 VTD NPC 는 죽는다. '영영 안 푼다'로 되돌리면 그 자리에서
    미완주(0점)가 확정된다 — 앞지르기 감점보다 훨씬 비싸다.
    """
    st = _turn_queue_stack(-1)
    out = _queue_run(st, 75.0, blocker_moving_s=1.0)
    seen = {r[1] for r in out}
    assert "PASS" in seen, f"한 주기를 훨씬 넘겨도 못 나갔다 (거친 상태: {sorted(seen)})"
    first = next(r[0] for r in out if r[1] == "PASS")
    # 측정된 신호 한 주기(36초, behavior.py 2026-09-06 코스 E 신호108)를 넘긴 뒤여야 한다.
    # `> JUNCTION_WAIT` 로는 부족하다 — 고치기 전 값(20.1초)도 그 문턱을 넘어 통과했다.
    assert first > 36.0, f"{first}초 — 신호 한 주기도 안 기다리고 나갔다"


def test_처음부터_선_차는_짧은_문턱으로_그대로_풀린다():
    """두 단으로 나누면서 **죽은 차 쪽 탈출이 느려지면 안 된다.**

    회전 60m 창은 코스 G 에서 경로의 26.0% 다. 거기 죽은 NPC 가 하나 서면 0점이라,
    이 경로만은 종전 20초를 그대로 지켜야 한다.
    (`test_우회전_앞이어도_오래_갇히면_빠져나간다` 는 90초를 봐서 문턱이 늘어나도
     통과한다 — 그래서 짧은 쪽을 따로 못 박는다.)
    """
    st = _turn_queue_stack(-1)
    out = _queue_run(st, 30.0)              # 20초는 넘고 신호 한 주기에는 못 미친다
    seen = {r[1] for r in out}
    assert "PASS" in seen, f"처음부터 선 차인데 30초에도 못 나갔다 ({sorted(seen)})"


def _approach_stack(turn_i=200, after=80):
    """긴 직선 뒤에 90도 우회전. 자차가 **실제로 전진하는** 재현에 쓴다."""
    route = [(float(x), 0.0) for x in range(turn_i + 1)]
    route += [(float(turn_i), -float(y)) for y in range(1, after + 1)]
    plan = [dict(lane=-1, w=3.3, l=5.0, r=5.0, xl=0.0, xr=0.0,
                 need=0.0, sig=0, lim=13.889,
                 j=1 if turn_i - 2 <= i <= turn_i + 4 else 0,
                 jx=1 if turn_i - 2 <= i <= turn_i + 4 else 0)
            for i in range(len(route))]
    return DrivingStack(route=route, lane_plan=plan), turn_i


def _approach_run(st, turn_i, start_gap, blk_gap=20.0, stop_at=30.0, secs=120.0):
    """회전 `start_gap` m 앞에서 출발해 `stop_at` m 남을 때까지 전진시킨다.

    자차를 세워 둔 하네스로는 '나갔다 돌아오는 왕복'을 못 본다 — 실제로 다가가야
    복귀가 회전 전에 끝나는지가 보인다. 횡추종은 이상적(명령 오프셋 = 실제 위치)으로
    둔다. 여기서 보려는 건 조향 추종 품질이 아니라 **언제 나가고 언제 돌아오나** 다.
    """
    from vtd_io import State, Obj

    ego = State(x=float(turn_i - start_gap), y=0.0, heading=0.0, speed=8.0)
    blk_x, t, dt, log = ego.x + blk_gap, 100.0, 0.05, []
    for _ in range(int(secs / dt)):
        ego.objects = [Obj(7, blk_x, 0.0, 0.0, 0.0, 0.0, 4.4, 1.8, 1.5)]
        c = st.step(ego, dt, t)
        log.append((round(t - 100.0, 2), turn_i - ego.x, c.ov_state, c.lane_offset))
        ego.x += max(c.v_cmd, 0.0) * dt
        ego.y = c.lane_offset                       # 이상적 횡추종
        ego.speed = max(c.v_cmd, 0.0)
        t += dt
        if turn_i - ego.x <= stop_at:
            break
    return log


def test_회전_게이트는_창_상수보다_멀리_닿는다():
    """★★'창 밖에서 시작한 추월이 회전 직전에 왕복한다'는 **재현되지 않았다.**

    의심한 것: 게이트 창이 60m 이므로 61~80m 지점에서 시작하면 왕복 기동(20~30m)이
    회전 직전에 끝나 "나갔다 -> 돌아왔다 -> 우회전"이 그대로 난다.

    실측 2026-09-08(자차가 실제로 전진하는 재현, 앞차는 처음부터 정지 = 게이트에
    가장 불리한 쪽) — 그렇게 되지 않는 이유는 **게이트가 창 상수보다 멀리 닿기**
    때문이다. `_turn_within` 은 회전 **정점**이 아니라 회전이 **시작되는 지점**까지의
    거리를 재므로, 창이 60m 여도 정점 기준 **83m 앞**부터 걸린다:
      정점 60·80·83m 앞 출발 -> 즉시 추월 없음. 유일한 탈출은 갇힘 해제(t=25.15초)이고
                                그건 위 두 단 문턱이 다루는 경로다.
      정점 90m 앞 출발       -> t=3.1초에 `PASS`, **복귀 완료 시 회전까지 63.1m** 남음.
                                그 뒤 63m 를 달린 다음에야 회전 차로로 붙는다 —
                                '회전 대기열 오인'이 아니라 평범한 추월이다.

    그래서 코드는 고치지 않는다. 대신 **그 여유를 여기서 못 박는다** — `_turn_within`
    의 창 의미가 바뀌면 게이트는 조용히 60m 로 줄고 60~83m 띠가 그대로 열린다.
    """
    st, turn_i = _approach_stack()
    reach = next(r for r in range(150, 40, -1)
                 if st._turn_within(turn_i - r, st.OVERTAKE_TURN_GUARD) != 0)
    assert reach >= st.OVERTAKE_TURN_GUARD + 20.0, \
        f"게이트가 정점 {reach}m 앞부터만 걸린다 (창 {st.OVERTAKE_TURN_GUARD:.0f}m + 기동 여유 부족)"

    # 게이트 밖에서 시작한 추월은 회전까지 넉넉히 남기고 복귀를 끝낸다.
    st, turn_i = _approach_stack()
    log = _approach_run(st, turn_i, start_gap=reach + 7.0)
    started = next((r for r in log if r[2] == "PASS"), None)
    assert started and started[0] < 10.0, f"게이트 밖인데 바로 안 나갔다 ({started})"
    back = next((r for i, r in enumerate(log)
                 if i and log[i - 1][2] in ("PASS", "RETURN") and r[2] == "FOLLOW"), None)
    assert back is not None, "복귀가 끝나지 않았다"
    assert back[1] >= 40.0, f"복귀 완료 시 회전까지 {back[1]:.1f}m — 회전 직전 왕복이다"


def _lc_stack():
    from drive import DrivingStack, straight_route
    return DrivingStack(route=straight_route(400.0))


def _P(sig=0, w=3.3, l=7.0, r=7.0, j=0):
    return dict(lane=-2, w=w, l=l, r=r, xl=0.0, xr=0.0, need=0.0, sig=sig, j=j, jx=0)


def test_차선변경_중_목표차로_뒤차에_양보한다():
    """[법 제19조③] 들어가려는 방향에서 오는 차의 통행에 장애를 주면 안 된다.
    사용자 지적 2026-08-30 코스 E (1249,-108): sig=L 켜고 뒤를 안 보고 들어갔다."""
    st = _lc_stack()
    # 좌측 차로(경로기준 +3.3m) 뒤 12m 에 나보다 6m/s 빠른 차
    objs = [(-12.0, 3.3, 14.0, 4.4, 1.8, 1.5, 5)]
    v = st._lc_rear_yield(_P(sig=+1), objs, d_ego=0.0, speed=8.0)
    assert v is not None and v <= 8.0 - st.LC_REAR_MARGIN, v
    assert v >= st.LC_REAR_MIN_V


def test_뒤차가_안_따라오면_그냥_간다():
    st = _lc_stack()
    objs = [(-12.0, 3.3, 7.0, 4.4, 1.8, 1.5, 5)]      # 나보다 느리다
    assert st._lc_rear_yield(_P(sig=+1), objs, d_ego=0.0, speed=8.0) is None


def test_반대쪽_차로_뒤차는_상관없다():
    st = _lc_stack()
    objs = [(-12.0, -3.3, 14.0, 4.4, 1.8, 1.5, 5)]    # 우측 차로인데 좌로 간다
    assert st._lc_rear_yield(_P(sig=+1), objs, d_ego=0.0, speed=8.0) is None


def test_앞차는_이_판정_대상이_아니다():
    st = _lc_stack()
    objs = [(+12.0, 3.3, 14.0, 4.4, 1.8, 1.5, 5)]
    assert st._lc_rear_yield(_P(sig=+1), objs, d_ego=0.0, speed=8.0) is None


def test_아직_먼_뒤차에는_안_줄인다():
    st = _lc_stack()
    objs = [(-33.0, 3.3, 9.0, 4.4, 1.8, 1.5, 5)]      # 접근 1m/s -> TTC 33s
    assert st._lc_rear_yield(_P(sig=+1), objs, d_ego=0.0, speed=8.0) is None


def test_차선변경_지시가_없으면_안_본다():
    st = _lc_stack()
    objs = [(-12.0, 3.3, 14.0, 4.4, 1.8, 1.5, 5)]
    assert st._lc_rear_yield(_P(sig=0), objs, d_ego=0.0, speed=8.0) is None


def test_교차로_안에서는_이_규칙을_안_쓴다():
    """교차로 안의 횡이동은 회전이지 진로변경이 아니다."""
    st = _lc_stack()
    objs = [(-12.0, 3.3, 14.0, 4.4, 1.8, 1.5, 5)]
    assert st._lc_rear_yield(_P(sig=+1, j=1), objs, d_ego=0.0, speed=8.0) is None


def test_정체된_목표차로_뒤차에는_안_걸린다():
    """★친구의 게이트 버전(9fa9bd4)은 `rear_gap < 8m` 만 봐서, 목표 차로가 정체면
    뒤차도 정지 -> 간격 고정 -> 우리도 정지 -> 교착이었다. **우리 버전은 접근속도
    0.3m/s 를 요구**하므로 원래 그 문제가 없다. 되돌린 뒤에도 그런지 못박는다."""
    st = _lc_stack()
    stopped = [(-5.0, 3.3, 0.0, 4.4, 1.8, 1.5, 9)]      # 좌측 뒤 5m 에 정지차
    assert st._lc_rear_yield(_P(sig=+1), stopped, d_ego=0.0, speed=0.0) is None
    assert st._lc_rear_yield(_P(sig=+1), stopped, d_ego=0.0, speed=8.0) is None


def test_양보는_감속뿐이고_절대_가속하지_않는다():
    st = _lc_stack()
    objs = [(-12.0, 3.3, 30.0, 4.4, 1.8, 1.5, 5)]     # 아주 빠른 뒤차
    v = st._lc_rear_yield(_P(sig=+1), objs, d_ego=0.0, speed=8.0)
    #  뒤차가 아무리 빨라도 상한은 **내 속도 아래**여야 한다(안 그러면 양보가 안 된다)
    assert v is not None and v <= 8.0 - st.LC_REAR_MARGIN, v




def test_PASS_객체가_한프레임_누락돼도_즉시_복귀하지_않는다():
    """객체 패킷의 순간 누락을 '다 지나감'으로 읽으면 장애물을 가로질러 복귀한다."""
    import types
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.3)
    ov.state, ov.offset, ov.block_id = "PASS", 3.3, 7
    ego_ = types.SimpleNamespace(speed=2.0)

    ov.plan(ego_, [], now=10.0, d_ego=3.3)
    ov.plan(ego_, [], now=10.2, d_ego=3.3)
    assert ov.state == "PASS"
    # 다시 보이면 누락 타이머가 초기화된다.
    seen = [(7, 2.0, -3.3, 0.0, 4.4, 1.8, 1.5)]
    ov.plan(ego_, seen, now=10.3, d_ego=3.3)
    ov.plan(ego_, [], now=10.4, d_ego=3.3)
    assert ov.state == "PASS"
    ov.plan(ego_, [], now=10.81, d_ego=3.3)
    assert ov.state == "RETURN"


def test_리셋은_관측_이력도_버린다():
    """리스폰이면 세상이 바뀐 것이다 — still_since 의 옛 시각이 남으면
    '충분히 오래 멈춘 차' 판정이 즉시 참이 된다."""
    from overtake import Overtaker
    ov = Overtaker(lane_width=3.3)
    ov.was_moving.add(9)
    ov.still_since[9] = 1.0
    ov._wait_since = 1.0
    ov.reset()
    assert not ov.was_moving and not ov.still_since
    assert ov._wait_since is None and ov.target_clear




def test_제한속도는_교차로에서_증발하지_않는다():
    """30 구간 사이 교차로 구멍(lim=None)에서 프로파일이 base(50)로 풀리면
    보호구역 한복판을 39km/h 로 지나는 수가 있다(합성 실측 2026-08-31).
    직전 제한을 이어 붙인다. 첫 유효값 이전 구간은 그대로 base 다."""
    route = straight_route(100.0)
    lp = ([dict(w=3.3, l=5.0, r=5.0, xl=0, xr=0, need=0, sig=0, j=0, jx=0, lim=8.33)] * 40
          + [None] * 20
          + [dict(w=3.3, l=5.0, r=5.0, xl=0, xr=0, need=0, sig=0, j=0, jx=0, lim=8.33)] * 40)
    st = DrivingStack(route=route, lane_plan=lp, base_limit=13.9)
    assert max(st.v_prof[40:60]) <= 8.33 + 0.05, max(st.v_prof[40:60]) * 3.6
    # 앞쪽이 None 이면(첫 표시 전) base 로 달려도 된다 — 과잉 제한 금지
    lp2 = ([None] * 30
           + [dict(w=3.3, l=5.0, r=5.0, xl=0, xr=0, need=0, sig=0, j=0, jx=0, lim=13.9)] * 70)
    st2 = DrivingStack(route=route, lane_plan=lp2, base_limit=13.9)
    assert max(st2.v_prof[:20]) > 10.0




def test_지시등_선행은_속도에_비례한다():
    """[대회 안내문 · 평가항목 13] "차로 변경 시 3초 내 미점등: 경미(-3)".
    법(30m)과 대회(3초)를 **둘 다** 만족해야 한다 — 50km/h 면 3초가 41.7m 다."""
    d = DrivingStack.__new__(DrivingStack)
    for kmh in (30, 50, 60):
        v = kmh / 3.6
        lead = DrivingStack._sig_lead(d, v)
        assert lead >= 30.0, (kmh, lead)                 # 법
        assert lead / v >= 3.0, (kmh, lead / v)          # 대회
    assert DrivingStack._sig_lead(d, 0.0) == DrivingStack.TURN_SIG_LEAD
    assert DrivingStack._sig_lead(d, 40.0) <= DrivingStack.TURN_SIG_LEAD_MAX


def test_차로계획_지시등도_30m와_3초를_같이_만족한다():
    """경로에 박힌 차선변경은 build_lane_plan 이 선행을 깐다. 그 값도 둘 다 넘어야 한다."""
    import json as _j, os as _o, math as _m, glob as _g
    R = _o.path.join(_o.path.dirname(_o.path.abspath(__file__)), "..", "routes")
    bad = []
    for rf in sorted(_g.glob(_o.path.join(R, "*.json"))):
        if rf.endswith("_lane.json"):
            continue
        lf = rf[:-5] + "_lane.json"
        if not _o.path.exists(lf):
            continue
        j = _j.load(open(rf, encoding="utf-8"))
        lm = j.get("ego_lanes")
        if not lm or len(lm[0]) < 5:
            continue
        rt, lp = j["ego_route"], _j.load(open(lf, encoding="utf-8"))["pts"]
        cum = [0.0]
        for i in range(1, len(rt)):
            cum.append(cum[-1] + _m.hypot(rt[i][0]-rt[i-1][0], rt[i][1]-rt[i-1][1]))
        runs = []
        for i, m in enumerate(lm):
            if not m[4]:
                continue
            if runs and i - runs[-1][1] <= 2:
                runs[-1][1] = i
            else:
                runs.append([i, i])
        for a, _b in runs:
            k = a
            while k > 0 and lp[k-1] and lp[k-1].get("sig"):
                k -= 1
            lead = cum[a] - cum[k]
            lim = (lp[a] or {}).get("lim") or 8.33
            if lead < 30.0 or lead / lim < 3.0:
                bad.append(f"{_o.path.basename(rf)[:-5]} s={cum[a]:.0f} {lead:.1f}m/{lead/lim:.2f}s")
    assert not bad, "; ".join(bad[:5])


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
