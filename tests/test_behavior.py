"""behavior 유닛테스트 — 실제로 겪은 버그를 그대로 회귀 테스트로 박아둔다.

실행: python3 -m pytest tests -q      (또는 python3 tests/test_behavior.py)
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from behavior import Behavior                      # noqa: E402
from vtd_io import (State, Obj, TL_RED, TL_YELLOW, TL_GREEN,     # noqa: E402
                    TL_GREEN_LEFT, TL_UNSET)


def ego(v=8.33, tl_state=TL_GREEN, tl_id=0, t=0.0):
    s = State()
    s.speed = v
    s.tl_state = tl_state
    s.tl_id = tl_id
    s.t = t
    return s


def hold_stop(b, mk, secs=None, step=0.1, t0=1.0, until=None):
    """**완전정지를 `stop_hold_s` 만큼 유지**시킨다.

    한 프레임 `speed<문턱` 으로는 '섰다'가 되지 않는다 — 안내문이 0.5초 이상을
    요구하고(항목 7·9), 우리는 그보다 엄하게 `stop_hold_s` 를 센다. 실측 코스 E
    신호 200 이 1km/h 이하 0.40초라 걸렸다.
    `mk(t)` 는 그 시각의 State 를 만드는 함수. `until()` 이 참이 되면 **거기서 멈춘다**
    — 한 프레임 더 돌리면 `_rtor_go` 같은 래치가 걸려 그 뒤 판정을 못 본다.
    """
    secs = (b.stop_hold_s + 0.2) if secs is None else secs
    out = None
    n = int(secs / step) + 1
    for k in range(n):
        out = mk(t0 + k * step)
        if until is not None and until():
            break
    return out


# rf_objs 한 항목 = (ds, d, speed, length, width, height, id)
def obj(ds, d, speed=0.0, L=4.4, W=1.8, H=1.5, oid=2):
    return (ds, d, speed, L, W, H, oid)


def test_먼_정지물은_감속_전까지_OBSTACLE_STOP_이라_안_부른다():
    """★`reason` 이 **실제로 속도를 쥔 규칙**과 달라선 안 된다. 예전엔 차로 안 정지물이
    보이기만 하면 거리와 무관하게 `OBSTACLE_STOP` 을 덮어썼다 — 45m 앞이라 아직 감속할
    필요도 없는 프레임까지 그렇게 찍혀서, CSV 로 "왜 느리지" 를 역추적할 때 헷갈린다.
    (`cap_by` 는 이미 이긴 제약을 정확히 냈는데 `reason` 만 거짓말을 했다.)"""
    b = Behavior()
    v, _o, _t, r = b.plan(ego(v=8.0), 8.33, rf_objs=[obj(45.0, 0.0, 0.0)])
    assert r != "OBSTACLE_STOP", r
    assert abs(v - 8.33) < 1e-6, v


def test_감속이_시작되면_OBSTACLE_STOP_이_맞다():
    """반대쪽 — 실제로 상한을 쥐기 시작하면 그렇게 불러야 한다.
    ⚠️ 더 가까워지면 차체 안전망(`NARROW_BLOCK`)이 이긴다. 이 라벨이 사는 구간은
       '감속은 시작했지만 아직 안전망 거리는 아닌' 곳이다."""
    b = Behavior()
    v, _o, _t, r = b.plan(ego(v=8.0), 8.33, rf_objs=[obj(25.0, 0.0, 0.0)])
    assert r == "OBSTACLE_STOP", r
    assert v < 8.33, v


def test_라벨은_cap_by_와_어긋나지_않는다():
    """`cap_by` 는 원래 **이긴 제약**을 정확히 냈는데 `reason` 만 거짓말을 했다.
    그게 이 수정의 이유다 — 둘이 같은 이야기를 해야 로그를 믿을 수 있다."""
    b = Behavior()
    #  멀 때: 아무도 안 잡았으므로 둘 다 장애물 이름이 아니다
    _v, _o, _t, r = b.plan(ego(v=8.0), 8.33, rf_objs=[obj(45.0, 0.0, 0.0)])
    assert r != "OBSTACLE_STOP" and b.cap_by != "OBSTACLE_STOP", (r, b.cap_by)
    #  잡기 시작하면 둘 다 장애물 이름이다
    b2 = Behavior()
    _v, _o, _t, r2 = b2.plan(ego(v=8.0), 8.33, rf_objs=[obj(25.0, 0.0, 0.0)])
    assert r2 == "OBSTACLE_STOP" and b2.cap_by == "OBSTACLE_STOP", (r2, b2.cap_by)


def test_라바콘은_보행자가_아니다():
    """L0.15 W0.46 H0.61 기름통을 보행자로 보고 영구정지하던 버그(2026-08-14)."""
    b = Behavior()
    v, *_ = b.plan(ego(), 8.33, rf_objs=[obj(5.0, 0.0, 0.0, 0.15, 0.46, 0.61)])
    assert v > 5.0, f"낮은 소형 정지물에 멈추면 안 됨 (v={v})"


def test_보행자에는_선다():
    b = Behavior()
    v, *_ = b.plan(ego(), 8.33, rf_objs=[obj(8.0, 0.0, 1.0, 0.6, 0.6, 1.75)])
    assert v < 3.0, f"보행자 앞에서는 감속/정지해야 함 (v={v})"


def test_정지_위치는_범퍼_기준_정지선_2m_이내다():
    """[대회 안내문 2026-08-27 · 평가항목 7] 범퍼 기준 정지선 2.0m 미만 0.5초 이상
    정지가 '정상'이고, 2m 이상이면 경미(-3), 선을 넘으면 중대(-6).

    실측 2026-08-30 코스 E: stop_margin=6.0 일 때 범퍼~정지선이 2.18~2.31m 로
    **신호 정지 10회 전부 초과**했다(정지 정확도 자체는 ±0.07m 로 훌륭했다).
    """
    b = Behavior()
    bumper = b.stop_margin - b.front_overhang
    assert 0.5 < bumper < 2.0, f"범퍼~정지선 {bumper:.2f}m (기준 0~2m)"


def test_적신호_정지선_모르면_선다():
    """정지선 미매핑 신호에서 서행 통과하면 그냥 신호위반."""
    b = Behavior()
    v, _, _, reason = b.plan(ego(tl_state=TL_RED, tl_id=99), 8.33, tl_stop_dist=None)
    assert v == 0.0 and reason == "RED_STOP_BLIND"


def test_상태미상_신호는_감속한다():
    """tl_id는 있는데 state=UNSET인 신호(tl80 실측) — 30km/h로 지나가면 안 됨."""
    b = Behavior()
    v, _, _, reason = b.plan(ego(tl_state=TL_UNSET, tl_id=80), 8.33, tl_stop_dist=20.0)
    assert v <= b.unknown_slow_v and reason == "TL_UNKNOWN_SLOW"


def test_정지선을_모르는_상태미상_신호는_그냥_간다():
    """정지선 DB 에 없는 tl_id 는 감속하지 않는다.

    2026-08-16 9경유지 실측: VTD 가 `tl_id=80`(항상 state=0)을 두 번 합쳐 52초 보냈다.
    그 번호는 정지선 214개 DB 에 없고 맵에 controller 80 도 없다 -> d_line 이 계속
    None 이라, 아무것도 없는 110m 를 13.5km/h 로 기었다("혼자 브레이크 밟는" 현상).
    설 정지선을 모르면 기어봐야 설 수도 없다.
    """
    b = Behavior()
    v, _, _, reason = b.plan(ego(v=13.9, tl_state=TL_UNSET, tl_id=80),
                             13.9, tl_stop_dist=None)
    assert reason != "TL_UNKNOWN_SLOW", reason
    assert v > b.unknown_slow_v, v


def test_정지선을_지나친_상태미상_신호도_안_기어간다():
    """정지선이 뒤에 있으면 drive.py 가 None 을 준다 — 그것도 감속 대상이 아니다."""
    b = Behavior()
    v, _, _, reason = b.plan(ego(v=13.9, tl_state=TL_UNSET, tl_id=80),
                             13.9, tl_stop_dist=None)
    assert v > b.unknown_slow_v and reason != "TL_UNKNOWN_SLOW"


def test_황색_멈출수_있으면_정지():
    b = Behavior()
    v, _, _, reason = b.plan(ego(v=8.33, tl_state=TL_YELLOW), 8.33, tl_stop_dist=60.0)
    assert reason == "YELLOW_STOP"


def test_황색_못멈추면_통과():
    b = Behavior()
    _, _, _, reason = b.plan(ego(v=8.33, tl_state=TL_YELLOW), 8.33, tl_stop_dist=6.0)
    assert reason == "YELLOW_GO"


def test_옆차선_걸친차_옆은_서행():
    """d=2.5m 차 옆을 30km/h로 지나가 실여유 0.6m였던 버그(2026-08-14)."""
    b = Behavior()
    v, _, _, reason = b.plan(ego(), 8.33, rf_objs=[obj(8.0, 2.5, 3.0)])
    assert v <= 6.0 and reason == "SIDE_CAUTION", f"(v={v}, {reason})"


def test_앞차_정지시_간격을_넉넉히():
    """급정거 후 추월할 공간이 남아야 한다 -> 정지 목표간격 15m."""
    b = Behavior()
    assert b.obstacle_gap >= 12.0



def test_통로가_좁으면_차선판정과_무관하게_막는다():
    """실측 2026-08-15 EV_CUTIN: rel = fy - lane_offset 로 오프셋을 두 번 빼는 바람에
    1.62m 옆의 차가 3.24m 밖으로 보여 모든 구속이 사라졌고 23km/h로 옆구리를 스쳤다.
    차선 부기가 어긋나도 '내 차체가 지나갈 통로' 검사만은 살아있어야 한다."""
    b = Behavior()
    far = b.plan(ego(), 8.33, lane_offset=1.62,
                 rf_objs=[obj(12.0, -1.62, speed=0.0, L=4.39, W=1.80, H=1.4)])[0]
    near = b.plan(ego(), 8.33, lane_offset=1.62,
                  rf_objs=[obj(3.0, -1.62, speed=0.0, L=4.39, W=1.80, H=1.4)])[0]
    assert far < 8.33 * 0.8, f"12m 앞 좁은 통로인데 {far*3.6:.1f}km/h"
    assert near < 1.0, f"3m 앞 좁은 통로인데 {near*3.6:.1f}km/h"


def test_다_지나친_물체에는_영구정지하지_않는다():
    """실측 2026-08-25 코스 D(주변교통 50대): 옆으로 가로지른 차와 스친 뒤 이륜차가
    **뒷축 옆 fx=-0.1m · 옆 1.4m** 에 와서 멈췄다. 앞범퍼는 이미 3.9m 지나쳐 있었는데
    `gap<0` 이라 NARROW_BLOCK 이 걸렸고 `_stop_target_speed(-4.9)`=0 —
    **1597프레임(64초) 완전 정지**(x·y 이동량 0.000m)로 주행이 죽었다.
    상대도 0 · 자차도 0 이라 기다려도 안 풀린다. 세워서 피할 수 없는 것에는 서면 안 된다."""
    b = Behavior()
    stuck = obj(-0.1, -1.39, speed=0.0, L=2.04, W=0.6, H=1.7)
    v, *_ = b.plan(ego(v=0.0), 8.33, rf_objs=[stuck])
    assert v > 0.1, f"다 지나친 옆 물체에 영구정지 (v={v*3.6:.1f}km/h)"


def test_막힌_물체의_뒷모서리_앞에_선다():
    """실측 2026-08-25 코스 G: 옆으로 추월하던 차가 급정거해 앞우측 6.9m 에 섰다.
    자차는 2.8km/h·여유 3.09m 로 **설 수 있었는데**, 정지 목표가 '중심 - 3.81 - 1.0'
    이라 다시 5.0km/h 로 가속해 파고들었다(뒷모서리를 1.3m 지나친 목표였다).
    결과 clr -0.40 으로 물린 뒤 **503초간 이동량 0.00m**.
    ★뒷모서리 기준으로 서면 fx~7.0m 라 추월 FSM 의 최소 간격(~7.0m)도 열린다."""
    b = Behavior()
    e = ego(v=4.75)
    # 폭 3.2 = 비스듬히 선 차의 경로축 외접폭(원본 1.8m). 이래야 gap<0 이 된다.
    v, *_ = b.plan(e, 8.33, rf_objs=[obj(6.9, -2.5, speed=0.0, L=4.6, W=3.2, H=1.5)])
    assert v < 0.1, f"뒷모서리(3.1m 앞)를 지나치는 목표로 기어간다 (v={v*3.6:.1f}km/h)"


def test_종방향으로_물고_있으면_여전히_막는다():
    """위 수정이 EV_CUTIN 을 풀어주면 안 된다. 중심 fx=3.0 은 범퍼(3.81)보다 뒤지만
    길이 4.39m 라 **앞끝이 5.2m** — 우리를 종방향으로 물고 있다. 전진하면 옆구리를 긁는다."""
    b = Behavior()
    v, *_ = b.plan(ego(), 8.33, lane_offset=1.62,
                   rf_objs=[obj(3.0, -1.62, speed=0.0, L=4.39, W=1.80, H=1.4)])
    assert v < 1.0, f"종방향으로 겹친 차인데 {v*3.6:.1f}km/h"


def test_좁혀오는_차는_미리_본다():
    """실측 2026-08-25 코스 G: 오른쪽으로 추월하던 차가 급정거해 앞우측 6.9m 에 섰다.
    지금 여유만 보면 그때서야 걸리는데, 최대제동으로도 2.2m 밀려 **-0.40m 로 물렸고**
    그 뒤 503초 정지했다. 그 차는 직전 1초 동안 이미 우리 쪽으로 좁혀오고 있었다.
    -> 닿을 때까지의 시간 동안 지금 속도로 계속 좁혀온다고 보고 미리 잡는다."""
    def run(seq):
        b = Behavior()
        e = ego(v=5.0)
        out = None
        for fx, fy in seq:
            out = b.plan(e, 8.33, rf_objs=[obj(fx, fy, speed=0.0, L=4.4, W=1.8, H=1.5)])
        return out

    # ① 제자리에 있는 차 — 옆으로 4.5m 면 여유가 충분하다
    v0, _o, _t, r0 = run([(12.0, -4.5), (11.0, -4.5), (10.0, -4.5)])
    # ② 같은 자리에서 **좁혀오는** 차 — 닿기 전에 여유가 사라진다
    v1, _o, _t, r1 = run([(12.0, -4.5), (11.0, -3.2), (10.0, -2.4)])
    assert v1 < v0 - 0.5, (f"좁혀오는 차를 미리 안 본다 "
                           f"(정지 {v0 * 3.6:.1f}km/h {r0} / 접근 {v1 * 3.6:.1f}km/h {r1})")


def test_벌어지는_차에는_예측을_안_쓴다():
    """⚠️ 예측으로 판정을 **느슨하게** 만들면 안 된다. 멀어지는 쪽은 무시한다."""
    b = Behavior()
    e = ego(v=5.0)
    b.plan(e, 8.33, rf_objs=[obj(10.0, -1.0, speed=0.0, L=4.4, W=1.8, H=1.5)])
    _v, _o, _t, r = b.plan(e, 8.33, rf_objs=[obj(10.0, -1.2, speed=0.0, L=4.4, W=1.8, H=1.5)])
    assert r in ("NARROW_BLOCK", "NARROW_PASS"), f"벌어진다고 봐준다 ({r})"


def test_충분히_넓으면_안_막는다():
    """옆차선(3.5m)에 차가 있는 정상 상황까지 서행시키면 안 된다."""
    b = Behavior()
    car = obj(8.0, -3.5, speed=8.0, L=4.39, W=1.80, H=1.4)
    v, *_ = b.plan(ego(), 8.33, rf_objs=[car])
    assert v > 7.0, f"옆차선 정상 통행인데 {v*3.6:.1f}km/h 로 느려짐"


def test_검증된_연료통_통과폭은_유지된다():
    """v5 에서 실여유 +0.45m 로 통과한 게 검증됐다 — 이걸 서행시키면 회귀."""
    b = Behavior()
    can = obj(8.0, -1.62, speed=0.0, L=0.15, W=0.46, H=0.61)
    v, *_ = b.plan(ego(), 8.33, rf_objs=[can])
    assert v > 7.0, f"연료통 회피 통과가 막힘 ({v*3.6:.1f}km/h)"



def test_좌회전은_좌회전신호에서만():
    """직진 녹색에 좌회전하면 신호위반. 규칙 자체가 없어서 우연히 통과하고 있었다(2026-08-15)."""
    from vtd_io import TL_GREEN, TL_LEFT, TL_GREEN_LEFT
    b = Behavior()
    # 정지선 25m 앞(제동 가능) — 직진 녹색인데 우리 경로는 좌회전 -> 감속해서 대기
    v, *_ = b.plan(ego(tl_state=TL_GREEN, tl_id=74), 8.33, tl_stop_dist=25.0, route_turn=+1)
    assert v < 8.0, f"직진 녹색에 좌회전하러 감 ({v*3.6:.1f}km/h)"
    # 정지선 코앞이면 사실상 정지
    #  ⚠️ 거리는 **stop_margin 상대**로 쓴다. 절대값으로 박으면 기준이 바뀔 때
    #     (2026-08-31 대회 2m 규칙으로 6.0 -> 5.0) 테스트가 같이 깨진다.
    v0, *_ = b.plan(ego(v=1.0, tl_state=TL_GREEN, tl_id=74), 8.33,
                    tl_stop_dist=b.stop_margin + 0.3, route_turn=+1)
    assert v0 < 1.0, f"정지선인데 안 섬 ({v0*3.6:.1f}km/h)"
    # 좌회전 화살표면 간다
    for st in (TL_GREEN_LEFT, TL_LEFT):
        vv, *_ = b.plan(ego(tl_state=st, tl_id=74), 8.33, tl_stop_dist=25.0, route_turn=+1)
        assert vv > 8.0, f"좌회전 신호({st})인데 안 감 ({vv*3.6:.1f}km/h)"


def test_직진은_좌회전화살표만으로_가지_않는다():
    from vtd_io import TL_LEFT
    b = Behavior()
    v, *_ = b.plan(ego(tl_state=TL_LEFT, tl_id=74), 8.33, tl_stop_dist=25.0, route_turn=0)
    assert v < 8.0, f"좌회전 화살표만 켜졌는데 직진함 ({v*3.6:.1f}km/h)"


def test_좌회전_딜레마존은_통과():
    """이미 못 멈추는 거리에서 신호가 애매하면 서다가 교차로에 갇히는 게 더 위험하다."""
    from vtd_io import TL_GREEN
    b = Behavior()
    v, *_ = b.plan(ego(tl_state=TL_GREEN, tl_id=74), 8.33, tl_stop_dist=5.0, route_turn=+1)
    assert v > 5.0, f"코앞인데 급정지 ({v*3.6:.1f}km/h)"



def test_사람_옆은_더_넉넉히_벌어질_때까지_안_붙는다():
    """실측 2026-08-15 EV_PED: 통로 안전망이 차량만 봐서, 사람 옆을 실여유 0.11m 로 스쳤다."""
    b = Behavior()
    ped = obj(10.0, -1.3, speed=0.0, L=0.23, W=0.40, H=1.40)   # 옆 1.3m = 아직 부족
    v, *_ = b.plan(ego(), 8.33, rf_objs=[ped])
    assert v < 6.0, f"사람 옆 1.3m 인데 {v*3.6:.1f}km/h 로 붙음"
    # 차로 하나를 벌리고 '지나가는 중(ped_bypass)' 이면 통로 검사는 통과해야 한다
    far = obj(10.0, -2.6, speed=0.0, L=0.23, W=0.40, H=1.40)
    v2, _, _, why = b.plan(ego(), 8.33, rf_objs=[far], ped_bypass=True)
    assert why != "NARROW_PASS", f"충분히 벌렸는데 통로 검사에 막힘 ({v2*3.6:.1f}km/h, {why})"


# ---------------------------------------------------------------- 우회전 일시정지
# 사용자 지적 2026-08-16: "우회전할때 멈췄다가 가야지".
# 도교법 제25조 — 우회전 전 일시정지. 녹색이어도 서지 않으면 횡단보도 보행자를
# 확인할 시간이 없다. 그동안 코드에 이 규칙이 아예 없었다(grep 0건).

def test_우회전은_녹색이면_서지_않고_서행한다():
    """★2026-09-06 실측 코스 E 신호109 · G 신호130 — 녹색에 일단 섰다가 서 있는 동안
    황->적으로 바뀌자 '정지 마쳤다'로 적색 진입 0.4초 만에 출발했다. 심판 기준(적색에서
    정지선 2.0m 미만 0.5초)에 못 미쳐 **항목7 중대 -6 이 두 판**. 녹색 30m 내 정차는 항목8
    위험이기도 하다. 그래서 녹색(직진녹·좌회전동시)은 서지 않고 서행으로 돈다."""
    for st in (TL_GREEN, TL_GREEN_LEFT):
        b = Behavior()
        v, _o, _t, r = b.plan(ego(v=8.33, tl_state=st, tl_id=5),
                              8.33, tl_stop_dist=20.0, route_turn=-1)
        assert r == "RIGHT_TURN_SLOW", r
        assert v <= b.rt_green_v + 1e-6, v
        assert not b._rt_armed, "녹색인데 정지를 무장했다"


def test_무신호_우회전은_여전히_일단_선다():
    """녹색 예외는 **신호가 있을 때** 얘기다. 신호 없는 교차로 우회전은 전과 같다."""
    b = Behavior()
    v, _o, _t, r = b.plan(ego(v=8.33, tl_state=TL_UNSET, tl_id=0),
                          8.33, junc_dist=20.0, route_turn=-1)
    assert r == "RIGHT_TURN_STOP", r
    assert v < 8.0, v


def test_적색_우회전의_일시정지는_적색_중에_한_것만_인정한다():
    """황색에 선 뒤 적색이 되면 적색에서 stop_hold_s 를 다시 채워야 RIGHT_ON_RED."""
    b = Behavior()
    b.plan(ego(v=8.33, tl_state=TL_YELLOW, tl_id=5), 8.33, tl_stop_dist=20.0, route_turn=-1)
    hold_stop(b, lambda t: b.plan(ego(v=0.0, tl_state=TL_YELLOW, tl_id=5, t=t), 8.33,
                                  tl_stop_dist=b.front_overhang + 1.0, route_turn=-1))
    assert b._rt_done and not b._rt_done_red
    _v, _o, _t, r = b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=5.0), 8.33,
                           tl_stop_dist=b.front_overhang + 1.0, route_turn=-1)
    assert r == "RED_STOP", f"적색 직후 바로 출발한다 ({r})"
    out = hold_stop(b, lambda t: b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=t), 8.33,
                                        tl_stop_dist=b.front_overhang + 1.0,
                                        route_turn=-1), t0=5.0)
    assert out[3] == "RIGHT_ON_RED", out[3]


def test_적색에서_선_것은_바로_적신호_우회전이_된다():
    b = Behavior()
    b.plan(ego(v=8.33, tl_state=TL_RED, tl_id=5), 8.33, tl_stop_dist=20.0, route_turn=-1)
    out = hold_stop(b, lambda t: b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=t), 8.33,
                                        tl_stop_dist=b.front_overhang + 1.0,
                                        route_turn=-1))
    assert b._rt_done and b._rt_done_red
    _v, _o, _t, r = b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=9.0), 8.33,
                           tl_stop_dist=b.front_overhang + 1.0, route_turn=-1)
    assert r == "RIGHT_ON_RED", r


def test_직진은_녹색에서_안_선다():
    b = Behavior()
    _v, _o, _t, r = b.plan(ego(v=8.33, tl_state=TL_GREEN, tl_id=5),
                           8.33, tl_stop_dist=20.0, route_turn=0)
    assert r != "RIGHT_TURN_STOP", r


def test_한번_서면_그_우회전은_통과시킨다():
    """(2026-09-06 녹색은 더 이상 안 서므로 **무신호** 우회전으로 바꿨다 — 계약은 같다)"""
    b = Behavior()
    e = ego(v=8.33, tl_state=TL_UNSET, tl_id=0)
    b.plan(e, 8.33, junc_dist=20.0, route_turn=-1)             # 접근
    hold_stop(b, lambda t: b.plan(ego(v=0.0, tl_state=TL_UNSET, tl_id=0, t=t), 8.33,
                                  junc_dist=8.0, route_turn=-1))      # 정지 유지
    _v, _o, _t, r = b.plan(ego(v=1.0, tl_state=TL_UNSET, tl_id=0, t=9.0), 8.33,
                           junc_dist=7.0, route_turn=-1)        # 재출발
    assert r != "RIGHT_TURN_STOP", f"섰는데도 계속 세운다 ({r})"


def test_우회전_일시정지는_한_프레임_스치는_걸로는_안_된다():
    """★실측 2026-08-30 코스 E 신호 200: **1km/h 이하가 0.40초**밖에 안 됐다.
    안내문 항목 7 은 "범퍼 기준 정지선 2.0m 미만에서 0.5초 이상" 이라 못 미친다.
    원인은 `speed < rt_stop_v` 가 참인 **첫 프레임**에 바로 풀어 준 것 —
    감속 곡선의 최저점을 스치기만 해도 '섰다'가 됐다."""
    b = Behavior()
    assert b.stop_hold_s > 0.5, "심판 문턱(0.5초)보다 엄해야 한다"
    assert b.stop_hold_v < 1 / 3.6, "심판 문턱(1km/h)보다 엄해야 한다"
    b.plan(ego(v=8.33, tl_state=TL_RED, tl_id=5), 8.33, tl_stop_dist=20.0, route_turn=-1)
    # 0.4초만 서 있으면(= 실측값) 아직 인정 안 된다
    hold_stop(b, lambda t: b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=t), 8.33,
                                  tl_stop_dist=b.front_overhang + 1.0,
                                  route_turn=-1), secs=0.4)
    assert not b._rt_done, "0.4초 정지를 '섰다'로 인정했다"
    # stop_hold_s 를 채우면 인정
    hold_stop(b, lambda t: b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=t), 8.33,
                                  tl_stop_dist=b.front_overhang + 1.0, route_turn=-1),
              t0=1.4, until=lambda: b._rt_done)
    assert b._rt_done


def test_정지_중_다시_움직이면_유지시계는_초기화된다():
    b = Behavior()
    b.plan(ego(v=8.33, tl_state=TL_RED, tl_id=5), 8.33, tl_stop_dist=20.0, route_turn=-1)
    hold_stop(b, lambda t: b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=t), 8.33,
                                  tl_stop_dist=b.front_overhang + 1.0,
                                  route_turn=-1), secs=0.5)
    b.plan(ego(v=2.0, tl_state=TL_RED, tl_id=5, t=1.6), 8.33,
           tl_stop_dist=7.0, route_turn=-1)               # 다시 굴렀다
    assert b._rt_still_since is None
    hold_stop(b, lambda t: b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=t), 8.33,
                                  tl_stop_dist=b.front_overhang + 1.0,
                                  route_turn=-1), t0=1.7, secs=0.4)
    assert not b._rt_done, "굴렀는데 이전 정지시간을 이어 세었다"


def test_적신호_우회전은_일시정지_후_간다():
    """시행규칙 별표2 '적색의 등화' 단서 — 정지 후 다른 차마를 방해하지 않으면 우회전 가능.
    전엔 녹색까지 기다렸다. 실측 대가: 코스 B 34.0초 · 코스 E 28.9초(2026-08-25)."""
    b = Behavior()
    _rtor_ready(b)
    v, _o, _t, r = b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=9.0), 8.33,
                          tl_stop_dist=b.front_overhang + 1.0, route_turn=-1)
    assert v > 0.1 and r == "RIGHT_ON_RED", f"적신호 우회전을 안 간다 (v={v*3.6:.1f} {r})"


def test_적신호_우회전도_직진은_선다():
    """단서는 **우회전에만** 적용된다. 직진이면 그대로 정지."""
    b = Behavior()
    v, _o, _t, r = b.plan(ego(v=8.33, tl_state=TL_RED, tl_id=5), 8.33,
                          tl_stop_dist=20.0, route_turn=0)
    assert r == "RED_STOP", f"적신호 직진인데 {r}"


def test_적신호_우회전은_안_섰으면_안_간다():
    """'정지 후'가 조건이다. 서지 않고 굴러들어오면 그냥 신호위반이다."""
    b = Behavior()
    v, _o, _t, r = b.plan(ego(v=8.33, tl_state=TL_RED, tl_id=5), 8.33,
                          tl_stop_dist=20.0, route_turn=-1)
    assert r != "RIGHT_ON_RED", f"안 섰는데 우회전한다 ({r})"


def test_적신호_우회전은_오는_차가_있으면_기다린다():
    """단서의 조건 = '신호에 따라 진행하는 다른 차마의 교통을 방해하지 아니하고'."""
    b = Behavior()
    _rtor_ready(b)
    e = ego(v=0.0, tl_state=TL_RED, tl_id=5, t=9.0)
    e.objects = [_oncoming(12.0, 3.0, 10.0)]     # 1.2초 뒤 내 자리를 지나갈 차
    v, _o, _t, r = b.plan(e, 8.33, tl_stop_dist=8.0, route_turn=-1)
    assert v < 0.1, f"오는 차가 있는데 우회전한다 (v={v*3.6:.1f} {r})"


def _rtor_ready(b, tl=5):
    """일시정지까지 마친 적신호 우회전 상황을 만든다(정지 유지 시간까지 채운다)."""
    b.plan(ego(v=8.33, tl_state=TL_RED, tl_id=tl), 8.33, tl_stop_dist=20.0, route_turn=-1)
    hold_stop(b, lambda t: b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=tl, t=t), 8.33,
                                  tl_stop_dist=b.front_overhang + 1.0, route_turn=-1),
              until=lambda: b._rt_done)


def _rtor_stop_result(gap):
    """앞범퍼~정지선 `gap`에서 적신호 우회전 정지를 시도한다."""
    b = Behavior()
    b.plan(ego(v=8.33, tl_state=TL_RED, tl_id=5), 8.33,
           tl_stop_dist=20.0, route_turn=-1)
    hold_stop(b, lambda t: b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=t), 8.33,
                                  tl_stop_dist=b.front_overhang + gap,
                                  route_turn=-1))
    return b, b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=9.0), 8.33,
                     tl_stop_dist=b.front_overhang + gap, route_turn=-1)


def test_적신호_우회전은_앞범퍼_1m에서_08초_정지하면_허용한다():
    b = Behavior()
    stop_dist = b.front_overhang + 1.0
    b.plan(ego(v=8.33, tl_state=TL_RED, tl_id=5), 8.33,
           tl_stop_dist=20.0, route_turn=-1)
    b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=1.0), 8.33,
           tl_stop_dist=stop_dist, route_turn=-1)
    b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=1.79), 8.33,
           tl_stop_dist=stop_dist, route_turn=-1)
    assert not b._rt_done, "0.8초 전에 정지를 인정했다"
    b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=1.8), 8.33,
           tl_stop_dist=stop_dist, route_turn=-1)
    out = b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=1.81), 8.33,
                 tl_stop_dist=stop_dist, route_turn=-1)
    assert abs(b._rt_done_bumper_gap - 1.0) < 1e-9
    assert out[3] == "RIGHT_ON_RED", out[3]


def test_적신호_우회전은_앞범퍼_2m_경계를_허용하지_않는다():
    for gap in (2.0, 2.01):
        b, out = _rtor_stop_result(gap)
        assert not b._rt_done, f"범퍼 거리 {gap}m 정지를 인정했다"
        assert out[3] != "RIGHT_ON_RED", (gap, out[3])


def test_적신호_우회전은_정지선을_지나_정지하면_허용하지_않는다():
    b, out = _rtor_stop_result(-0.01)
    assert not b._rt_done, "앞범퍼가 정지선을 지난 후의 정지를 인정했다"
    assert out[3] != "RIGHT_ON_RED", out[3]


def test_적신호_우회전은_먼_곳_정지_후_유효구간에서_다시_서야_한다():
    b = Behavior()
    b.plan(ego(v=8.33, tl_state=TL_RED, tl_id=5), 8.33,
           tl_stop_dist=b.front_overhang + 4.0, route_turn=-1)
    far = hold_stop(
        b, lambda t: b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=t), 8.33,
                            tl_stop_dist=b.front_overhang + 4.0, route_turn=-1))
    assert not b._rt_done and far[3] != "RIGHT_ON_RED"

    # 정지선 앞으로 이동한 뒤에는 새 0.8초를 채워야 한다.
    b.plan(ego(v=1.0, tl_state=TL_RED, tl_id=5, t=3.0), 8.33,
           tl_stop_dist=b.front_overhang + 1.0, route_turn=-1)
    hold_stop(b, lambda t: b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=t), 8.33,
                                  tl_stop_dist=b.front_overhang + 1.0,
                                  route_turn=-1), t0=4.0, until=lambda: b._rt_done)
    out = b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5, t=9.0), 8.33,
                 tl_stop_dist=b.front_overhang + 1.0, route_turn=-1)
    assert b._rt_done and out[3] == "RIGHT_ON_RED", out[3]


def test_적신호_우회전은_정지선_직전에서_선_것만_인정():
    """실측 2026-08-25 코스 A tl130: 정체 뒤 **43.7m** 지점에서 완전정지한 걸
    '일시정지 완료'로 치고 정지선은 14.8km/h 로 통과했다. 회전 자체는 -98.4° 로
    멀쩡했지만 **어디서 섰느냐**가 틀렸다 — 별표2 단서는 '정지선 직전에서 정지'다.
    (우회전 일시정지(제25조)는 접근 어디서 서도 목적을 이루지만, 적신호 통과는 아니다.)"""
    b = Behavior()
    # 정지선 40m 앞에서 정지(정체 뒤) -> 그대로 접근
    b.plan(ego(v=8.33, tl_state=TL_RED, tl_id=5), 8.33, tl_stop_dist=45.0, route_turn=-1)
    b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5), 8.33, tl_stop_dist=40.0, route_turn=-1)
    v, _o, _t, r = b.plan(ego(v=0.0, tl_state=TL_RED, tl_id=5), 8.33,
                          tl_stop_dist=8.0, route_turn=-1, rtor_point=(8.0, -4.0))
    assert r != "RIGHT_ON_RED", f"정지선 40m 뒤에서 선 걸로 적신호를 통과한다 ({r})"


def test_적신호_우회전은_옆으로_지나가는_차에_막히지_않는다():
    """실측 2026-08-25 코스 H: **왼쪽 3.1m 의 1.4m/s 짜리 차 하나**에 18초를 섰다.
    양보 판정을 자차 위치로 하면 뒤차·왼쪽 차까지 충돌 상대가 된다 —
    우회전의 상대는 **회전을 마친 뒤 내가 있을 자리**를 지나갈 차다.
    (실측: 양보 A 29.9s·H 18.0s 인데 통과는 A 3.1s·H 0s 였다.)"""
    b = Behavior()
    _rtor_ready(b)
    e = ego(v=0.0, tl_state=TL_RED, tl_id=5, t=9.0)
    e.objects = [Obj(601, 0.9, 3.0, 0.0, 0.0, 1.4, 4.4, 1.8, 1.5)]   # 왼쪽 옆, 같은 방향
    v, _o, _t, r = b.plan(e, 8.33, tl_stop_dist=8.0, route_turn=-1, rtor_point=(8.0, -4.0))
    assert v > 0.1 and r == "RIGHT_ON_RED", f"옆 차에 막힌다 (v={v*3.6:.1f} {r})"


def test_적신호_우회전은_합류지점을_지날_차에는_선다():
    """기준점을 옮긴다고 느슨해지면 안 된다 — **들어갈 자리를 지나갈 차**는 여전히 막는다."""
    b = Behavior()
    _rtor_ready(b)
    e = ego(v=0.0, tl_state=TL_RED, tl_id=5, t=9.0)
    # 합류지점(8,-4) 아래 15m 에서 그쪽으로 8m/s 로 올라온다 -> 1.9초 뒤 정확히 그 자리
    e.objects = [Obj(602, 8.0, -19.0, 0.0, math.pi / 2, 8.0, 4.4, 1.8, 1.5)]
    v, _o, _t, r = b.plan(e, 8.33, tl_stop_dist=8.0, route_turn=-1, rtor_point=(8.0, -4.0))
    assert v < 0.1, f"합류지점을 지날 차가 있는데 간다 (v={v*3.6:.1f} {r})"


def test_신호없는_교차로도_교차로거리로_선다():
    b = Behavior()
    _v, _o, _t, r = b.plan(ego(v=8.33, tl_state=TL_UNSET, tl_id=0),
                           8.33, tl_stop_dist=None, route_turn=-1, junc_dist=18.0)
    assert r == "RIGHT_TURN_STOP", r


# ------------------------------------------------- 옆 차선 느린 차 접근 감속
# 배경(2026-08-16): 제한을 지도값 50km/h 로 올린 직후 v1 에서 접촉 9프레임(-0.41m).
# 우측 차선 18km/h 차가 10.8m 앞에 있는데 50.5km/h 로 접근했고, 그 차가 가속하며
# 끼어들어 급제동했지만 스쳤다. 30 으로 달릴 땐 이 상황 자체를 안 만들었을 뿐이다.

def test_옆차선_느린차에_접근하면_미리_줄인다():
    b = Behavior(speed_limit=13.89)                      # 50km/h
    e = ego(v=14.0)                                      # 실측: 50.5km/h
    objs = [(10.82, -3.29, 5.0, 4.4, 1.8, 1.5, 7)]       # 우측 3.29m, 10.8m 앞, 18km/h
    v, _o, _t, r = b.plan(e, 13.89, rf_objs=objs)
    assert r == "ADJ_SLOW", r
    assert v * 3.6 < 32, f"{v*3.6:.1f}km/h — 충분히 안 줄었다"


def test_옆차선_같은속도_차에는_안_줄인다():
    b = Behavior(speed_limit=13.89)
    objs = [(10.82, -3.29, 13.5, 4.4, 1.8, 1.5, 7)]      # 나와 비슷한 속도
    v, _o, _t, r = b.plan(ego(v=14.0), 13.89, rf_objs=objs)
    assert r != "ADJ_SLOW", r
    assert v > 13.0, v


def test_멀리_있는_옆차선_차에는_안_줄인다():
    b = Behavior(speed_limit=13.89)
    objs = [(60.0, -3.29, 5.0, 4.4, 1.8, 1.5, 7)]        # 60m 앞 = 여유 있음
    _v, _o, _t, r = b.plan(ego(v=14.0), 13.89, rf_objs=objs)
    assert r != "ADJ_SLOW", r


def test_보행자는_이_규칙_대상이_아니다():
    b = Behavior(speed_limit=13.89)
    objs = [(10.0, -3.0, 1.0, 0.4, 0.4, 1.7, 7)]         # 인도 위 보행자
    _v, _o, _t, r = b.plan(ego(v=14.0), 13.89, rf_objs=objs)
    assert r != "ADJ_SLOW", r


def test_회전에_진입해_route_turn이_0이_돼도_계속_세운다():
    """실측 2026-08-16 9경유지: 12회 중 2회가 30->15km/h 로 줄이다 **재가속해 안 섰다**.
    _upcoming_turn 은 전방을 내다봐 꺾임을 찾으므로 회전에 진입하면 0 이 된다.
    거기서 무장을 풀면 서다 말고 통과한다."""
    b = Behavior()
    # 접근: 우회전이 앞에 보임
    b.plan(ego(v=8.4), 8.33, tl_stop_dist=None, route_turn=-1, junc_dist=20.0)
    # 진입: route_turn 이 0 이 됐지만 아직 안 섰다 -> 계속 세워야 한다
    v8, _o, _t, r = b.plan(ego(v=4.2), 8.33, tl_stop_dist=None, route_turn=0, junc_dist=8.0)
    assert r == "RIGHT_TURN_STOP", f"회전 진입에서 놓아버린다 ({r})"
    # 정지선에 가까워질수록 목표속도가 0 으로 수렴해야 한다(감속 프로파일)
    v6, *_ = b.plan(ego(v=3.0), 8.33, tl_stop_dist=None, route_turn=0,
                    junc_dist=b.stop_margin + 1.5)
    v0, *_ = b.plan(ego(v=2.0), 8.33, tl_stop_dist=None, route_turn=0,
                    junc_dist=b.stop_margin + 0.1)
    assert v8 > v6 > v0, f"감속하지 않는다 ({v8:.2f} {v6:.2f} {v0:.2f})"
    assert v0 < 0.1, f"정지선에서 목표가 0 이 아니다 ({v0*3.6:.1f}km/h)"
    # 정지점을 지나 junc_dist 가 사라져도 설 때까지는 유지
    v, _o, _t, r = b.plan(ego(v=1.5), 8.33, tl_stop_dist=None, route_turn=0, junc_dist=None)
    assert r == "RIGHT_TURN_STOP", f"정지점이 사라졌다고 놓아버린다 ({r})"
    # 서고 나면 통과 (한 프레임 스친 게 아니라 stop_hold_s 만큼 유지해야 한다)
    hold_stop(b, lambda t: b.plan(ego(v=0.0, t=t), 8.33, tl_stop_dist=None,
                                  route_turn=0, junc_dist=None))
    _v, _o, _t, r = b.plan(ego(v=2.0, t=9.0), 8.33, tl_stop_dist=None,
                           route_turn=0, junc_dist=None)
    assert r != "RIGHT_TURN_STOP", f"섰는데도 계속 세운다 ({r})"

# ------------------------------------------------- 속도 제약 기록(cap_by)
def test_cap_by_는_실제로_속도를_잡은_규칙을_가리킨다():
    """reason 과 cap_by 는 다를 수 있고, **다를 때가 중요하다**.

    속도를 깎는 곳이 18군데인데 reason 은 규칙마다 갱신 방식이 다르다(무조건 덮어쓰는
    것도, LANE_KEEP 일 때만 쓰는 것도 있다). 그래서 로그의 reason 이 실제로 속도를
    잡은 규칙과 어긋날 수 있고, 2026-08-16 하루 동안 '왜 느리지 / 왜 멈췄지'를
    CSV 로 역추적하는 데 그만큼 시간을 썼다.
    """
    b = Behavior()

    # 아무 제약도 없으면 프로파일이 이긴다
    b.plan(ego(v=8.33), 8.33)
    assert b.cap_by in ("PROFILE", "SPEED_LIMIT"), b.cap_by

    # 적신호가 잡으면 RED_STOP
    b2 = Behavior()
    b2.plan(ego(v=8.33, tl_state=TL_RED, tl_id=5), 8.33, tl_stop_dist=20.0)
    assert b2.cap_by == "RED_STOP", b2.cap_by

    # ★적신호 reason 이지만 **더 낮은 제약**이 따로 있으면 cap_by 는 그쪽을 가리킨다
    b3 = Behavior()
    _v, _o, _t, r = b3.plan(ego(v=8.33, tl_state=TL_RED, tl_id=5), 8.33,
                            tl_stop_dist=60.0,          # 멀어서 약한 제동
                            rf_objs=[obj(8.0, 0.0, 0.0, 4.4, 1.8, 1.5)])  # 코앞 정지차
    assert r != b3.cap_by, (r, b3.cap_by)        # reason 과 이긴 제약이 갈린다
    assert b3.cap_by == "OBSTACLE_STOP", b3.cap_by   # 실제로 잡은 건 장애물 쪽


def test_속도제약을_전부_기록한다():
    """min() 이 흩어져 있어도 기록은 한 곳에 모인다 — 빠진 규칙이 있으면 여기서 걸린다."""
    b = Behavior()
    b.plan(ego(v=8.33, tl_state=TL_RED, tl_id=5), 8.33, tl_stop_dist=20.0,
           rf_objs=[obj(12.0, 0.0, 0.0, 4.4, 1.8, 1.5)])
    names = [n for _v, n in b._caps]
    assert "SPEED_LIMIT" in names and "PROFILE" in names, names
    assert "RED_STOP" in names and "OBSTACLE_STOP" in names, names


# ── 무단횡단(2026-08-19 v6 실측) ───────────────────────────────────────────────
# 조깅(3.0m/s)으로 진로를 가로지르는 사람. 그날 로그에서 자차는 **한 번도 서지 않았고**
# 사람 앞을 3.58m/s 로 굴러 지나갔다(최소 실여유 3.97m). 세워준 건 마침 켜져 있던
# 적신호였다(cap_by 가 내내 RED_STOP). 신호가 녹색이면 그대로 양보실패다.

def test_사람_정지간격은_앞범퍼_기준이다():
    """`fx - 6.0` 은 뒷축 기준이라 실여유가 6.0-3.81=2.19m 였다. 사람한테 2.19m 는 없다."""
    b = Behavior()
    # 뒷축에서 9m = 앞범퍼에서 5.2m. 여기서는 이미 서 있어야 한다.
    v, *_ = b.plan(ego(v=5.0), 8.33, rf_objs=[obj(9.0, 0.0, 3.0, 0.6, 0.7, 1.8, 301)])
    assert v < 1.5, f"앞범퍼 5.2m 앞 사람이면 정지여야 함 (v={v})"
    assert b.ped_gap - 0.0 >= 4.0, b.ped_gap


def _cross(b, ds, fy0, vy=3.0, n=6, dt=0.04, ev=13.9, vp=13.9):
    """진로 쪽으로 vy[m/s] 로 다가오는 사람을 n 프레임 먹인다 -> (마지막 지령, cap_by)."""
    v = None
    for _ in range(n):
        v, *_ = b.plan(ego(v=ev), vp, rf_objs=[obj(ds, fy0, abs(vy), 0.6, 0.7, 1.8, 301)])
        fy0 += vy * dt if fy0 < 0 else -vy * dt
    return v, b.cap_by


def test_뛰어드는_사람은_창에_들기_전에_미리_줄인다():
    """|fy|<4.5 에 들어와야 반응하면 3m/s 로 뛰어드는 사람에겐 1.4초밖에 안 남는다."""
    b = Behavior(speed_limit=13.9)
    v, cap = _cross(b, 35.0, -8.0)
    assert v < 12.0, f"진로로 다가오는 사람 앞에서는 미리 줄여야 함 (v={v}, {cap})"
    assert cap == "PED_CAUTION", cap
    # 미리 줄이는 것만으로 서 버리면 안 된다(멀리 있는 사람에 갇힌다)
    assert v >= b.ped_caution_min, v


def test_멀리_옆에서_뛰어들어도_미리_본다():
    """★한 번 틀렸던 자리. 감시 폭을 `사람속도 × 자차도달시간` 으로 잡으면 가까워질수록
       폭이 사람보다 빨리 좁아져 **아무것도 안 잡힌다**(2026-08-19 v6 실측, 실여유 1.09m).
         t=13.02  사람 28.3m 앞·옆 11.4m  reach 7.0  -> 발동 안 함
         t=14.50  사람  8.5m 앞·옆  5.0m  reach 2.8  -> 발동 안 함
       폭은 자차 속도에 줄면 안 된다."""
    b = Behavior(speed_limit=13.9)
    v, cap = _cross(b, 28.3, -11.4)
    assert v < 11.0, f"28m 앞·옆 11m 에서 뛰어드는 사람이면 이미 줄었어야 함 (v={v}, {cap})"


def test_인도를_걷는_사람에는_안_줄인다():
    """예측을 넣었다고 옆으로 지나가는 사람마다 서면 완주를 못 한다.
       나란히 걷는 사람은 **횡속도가 0** 이라 걸리지 않아야 한다."""
    b = Behavior(speed_limit=13.9)
    v = None
    for _ in range(6):                      # |fy| 가 안 변한다 = 진로 쪽으로 안 온다
        v, *_ = b.plan(ego(v=13.9), 13.9,
                       rf_objs=[obj(30.0, 9.0, 1.2, 0.6, 0.7, 1.8, 301)])
    assert v > 12.0, f"9m 옆 산책(1.2m/s)에는 반응하지 않아야 함 (v={v})"


def test_서_있고_진로_밖인_사람은_서행_통과():
    """실측 2026-08-26 코스 E: 옆 2.9m 에 **가만히 선** 사람 하나가 215초를 세웠다
    (차체 여유 +3.63m 인데도). 구조적 모순이었다 — 정지 판정은 4.5m 까지 거는데
    우회 검토는 2.5m 까지라(`drive.py::ped_block`) 그 사이 대역은 세우기만 하고
    비켜갈 검토는 영영 안 된다. 제27조는 '횡단하거나 횡단하려는' 보행자 보호다."""
    b = Behavior()
    still = obj(12.0, 2.9, speed=0.0, L=0.4, W=0.5, H=1.7)
    v, _o, _t, r = b.plan(ego(), 8.33, rf_objs=[still])
    assert v > 1.0, f"서 있는 사람 옆에서 멈춰버린다 (v={v * 3.6:.1f}km/h {r})"


def test_진로_안의_사람에는_여전히_선다():
    """느슨해지면 안 된다 — 내 차로 안(또는 경계)에 있으면 그대로 정지."""
    b = Behavior()
    # ⚠️ 12m 거리면 정지 프로필상 11km/h 가 정상이다(감속 중). **설 자리에서** 봐야 한다.
    on_path = obj(7.0, 0.5, speed=0.0, L=0.4, W=0.5, H=1.7)
    v, _o, _t, r = b.plan(ego(), 8.33, rf_objs=[on_path])
    assert v < 1.0, f"진로 위 사람인데 안 선다 (v={v * 3.6:.1f}km/h {r})"


def test_움직이는_사람에는_여전히_선다():
    """횡단 중인 사람은 진로 밖이어도 정지 — 곧 진로로 들어온다."""
    b = Behavior()
    crossing = obj(7.0, 2.9, speed=1.2, L=0.4, W=0.5, H=1.7)
    v, _o, _t, r = b.plan(ego(), 8.33, rf_objs=[crossing])
    assert v < 1.0, f"횡단 중인 사람인데 안 선다 (v={v * 3.6:.1f}km/h {r})"


def test_다_건너간_사람은_계속_안_기다린다():
    """멀어지는 중이면 통과. (예측 조항이 이 탈출구를 막으면 영원히 못 간다)

    ★문턱을 `lane_half + ped_leave_pad`(2.6m)로 올렸다(2026-09-11 코스 A TRV).
      예전 2.6m 에서는 횡단 중인 사람이 우리 차로만 지나면 바로 풀려 **찔끔 출발**했다 —
      실측 t=71.3 에 옆 3.0m 에서 출발했고, 1.2초 뒤 그 사람이 5.0m 에서 멈춰 서자
      다시 급정거(3.0 -> 0.07km/h). 사용자: "왜 사람 보고도 멈추지를 않고 찔끔찔끔".
    """
    b = Behavior()
    lo = b.lane_half + b.ped_leave_pad
    b.plan(ego(v=5.0), 8.33, rf_objs=[obj(10.0, lo + 0.1, 3.0, 0.6, 0.7, 1.8, 301)])
    v, _o, _t, why = b.plan(ego(v=5.0), 8.33,
                            rf_objs=[obj(10.0, lo + 0.2, 3.0, 0.6, 0.7, 1.8, 301)])
    assert why == "PED_LEAVING" or b.cap_by == "PED_LEAVING", (why, b.cap_by)
    assert v > 1.0, f"멀어지는 사람에 멈추면 안 됨 (v={v})"
    assert lo < b.ped_watch, "탈출구가 완전히 막히면 다 건너간 사람에 교착된다"


def test_차로만_지난_사람에는_아직_안_간다():
    """실측 t=71.3: 옆 3.0m 에서 출발했다. 우리 차로만 지난 것이지 다 건넌 게 아니다."""
    b = Behavior()
    b.plan(ego(v=5.0), 8.33, rf_objs=[obj(10.0, 2.9, 3.0, 0.6, 0.7, 1.8, 302)])
    v, _o, _t, why = b.plan(ego(v=5.0), 8.33,
                            rf_objs=[obj(10.0, 3.0, 3.0, 0.6, 0.7, 1.8, 302)])
    assert why != "PED_LEAVING" and b.cap_by != "PED_LEAVING", (why, b.cap_by)


# ── 비보호 좌회전 (2026-08-20 새 코스 E 실측) ────────────────────────────────
# 마지막 교차로(tl 108)가 RED->GREEN->YELLOW 만 돌고 **좌회전 화살표가 없는데**
# 코스는 거기서 좌회전을 요구한다. '좌회전은 화살표에서만' 규칙이 영원히 기다려
# 목표 84m 앞에서 미완주로 끝났다.

def test_화살표_없는_교차로는_비보호_좌회전이다():
    b = Behavior()
    v = why = None
    for st in (TL_GREEN, TL_RED, TL_RED, TL_GREEN):     # 한 주기. 화살표는 한 번도 없다
        v, _o, _t, why = b.plan(ego(v=0.0, tl_state=st, tl_id=108), 8.33,
                                tl_stop_dist=8.0, route_turn=1)
    assert why == "UNPROTECTED_LEFT", why
    assert 0 < v <= b.unprotected_left_v + 1e-6, f"서행으로 지나가야 함 (v={v})"


def test_화살표_있는_교차로는_녹색에_안_꺾는다():
    """한 번이라도 화살표를 본 신호에서는 직진 녹색에 좌회전하면 신호위반이다."""
    b = Behavior()
    v = why = None
    for st in (TL_GREEN, TL_GREEN_LEFT, TL_RED, TL_GREEN):
        v, _o, _t, why = b.plan(ego(v=0.0, tl_state=st, tl_id=109), 8.33,
                                tl_stop_dist=8.0, route_turn=1)
    assert why == "WAIT_LEFT_ARROW", why


def test_비보호_판정은_신호마다_따로다():
    """화살표 있는 신호를 봤다고 다른 신호까지 보호로 보면 안 된다(그 반대도)."""
    b = Behavior()
    for st in (TL_GREEN, TL_GREEN_LEFT):
        b.plan(ego(v=0.0, tl_state=st, tl_id=200), 8.33, tl_stop_dist=8.0, route_turn=1)
    why = None
    for st in (TL_GREEN, TL_RED, TL_GREEN):
        _v, _o, _t, why = b.plan(ego(v=0.0, tl_state=st, tl_id=201), 8.33,
                                 tl_stop_dist=8.0, route_turn=1)
    assert why == "UNPROTECTED_LEFT", why


def _oncoming(fx, fy=3.5, spd=8.0):
    """마주오는 차 하나(ego 는 원점·heading 0 이므로 fx,fy 가 그대로 상대좌표)."""
    return Obj(501, fx, fy, 0.0, math.pi, spd, 4.8, 1.9, 1.5)


def _no_arrow(b, tl=108, objs=(), v=0.0):
    """화살표 없는 신호에서 한 주기를 보여주고 마지막 판정을 돌려준다."""
    out = None
    for st in (TL_GREEN, TL_RED, TL_GREEN):
        e = ego(v=v, tl_state=st, tl_id=tl)
        e.objects = list(objs)
        out = b.plan(e, 8.33, tl_stop_dist=20.0, route_turn=1)
    return out


def test_비보호_좌회전은_대향_직진차를_보내고_간다():
    """처음엔 5초 틈만 보고 출발해 **그대로 충돌했다**(오프라인 판 50점).
       좌회전은 정지에서 출발해 대향 차로를 비우기까지 8초 남짓 걸린다."""
    b = Behavior()
    _v, _o, _t, why = _no_arrow(b, objs=[_oncoming(60.0)])      # 60/8 = 7.5초 -> 부족
    assert why == "YIELD_ONCOMING", why


def test_대향차가_멀면_그냥_간다():
    b = Behavior()
    _v, _o, _t, why = _no_arrow(b, objs=[_oncoming(85.0)])      # 85/8 = 10.6초 -> 충분
    assert why == "UNPROTECTED_LEFT", why


def test_같은_방향으로_멀어지는_차는_안_막는다():
    """앞서 가는 차 때문에 좌회전을 영영 못 하면 안 된다."""
    b = Behavior()
    same = Obj(502, 40.0, 0.0, 0.0, 0.0, 8.0, 4.8, 1.9, 1.5)    # heading 0 = 같은 방향
    _v, _o, _t, why = _no_arrow(b, objs=[same])
    assert why == "UNPROTECTED_LEFT", why


def test_옆에서_가로지르는_차도_잡는다():
    """★한 번 틀렸던 자리. `heading 차 120° 이상 = 대향차` 로 짰다가 **실주행에서
       한 대도 못 걸렀다**(2026-08-20 HL_FMA_LEFTONC, T자로). 그 교차로의 충돌 상대는
       정면이 아니라 옆 90° 에서 온다 — ego 헤딩 308°, 상대 218°.
       실측 좌표 그대로: 19.5m 앞·6.1m 왼쪽에서 4m/s 로 내 앞을 가로지른다."""
    b = Behavior()
    cross = Obj(503, 19.5, 6.1, 0.0, math.radians(-90.1), 4.0, 4.8, 1.9, 1.5)
    _v, _o, _t, why = _no_arrow(b, objs=[cross])
    assert why == "YIELD_ONCOMING", why


def test_뒤로_지나가는_차는_안_막는다():
    """내 뒤를 가로지르는 차까지 기다리면 좌회전을 못 한다(같은 실측의 반대편 차)."""
    b = Behavior()
    behind = Obj(504, -20.5, 5.8, 0.0, math.radians(-90.1), 4.0, 4.8, 1.9, 1.5)
    _v, _o, _t, why = _no_arrow(b, objs=[behind])
    assert why == "UNPROTECTED_LEFT", why


def test_정지선을_지난_적신호에는_교차로를_빠져나간다():
    """실측 2026-08-20: 좌회전 중 적색이 되자 **교차로 한복판에서 11초** 서 있었다.
       거기 서 있는 게 더 위험하다 — 빠져나가는 게 맞다."""
    b = Behavior()
    v, _o, _t, why = b.plan(ego(v=5.0, tl_state=TL_RED, tl_id=7), 8.33,
                            tl_stop_dist=None, tl_passed=True)
    assert why == "RED_CLEARING" and v > 1.0, (why, v)


def test_모르는_신호의_적색에는_여전히_선다():
    """위 예외가 '정지선 모르면 정지' 규칙을 통째로 무력화하면 안 된다."""
    b = Behavior()
    v, _o, _t, why = b.plan(ego(v=5.0, tl_state=TL_RED, tl_id=8), 8.33, tl_stop_dist=None)
    assert why == "RED_STOP_BLIND" and v == 0.0, (why, v)


def test_신호없는_교차로_좌회전은_가로지르는_차에_양보한다():
    """실측 2026-08-26 (1457,938) — 신호 없는 교차로(tl 0/UNSET)에서 73° 좌회전.
    코스 E 와 H 가 **같은 자리에서 같은 식으로** 스쳤다:
      E t=160.4 왼쪽 21m 에 47km/h 로 가로지르는 차 -> 가속해서 진입 -> t=162.2 실여유 -0.65m
      H t=307   같은 자리, 58km/h 짜리에 -0.19m
    2.4초 전부터 보이는 차였는데 `LANE_KEEP/PROFILE` 로 25km/h 까지 가속했다.
    신호 있는 비보호 좌회전(`YIELD_ONCOMING`)만 있었고 무신호 교차로엔 규칙이 없었다.

    ★기준점은 자차가 아니라 **회전을 마친 뒤 있을 자리**여야 한다 — 실측값으로
      자차 기준 CPA 는 12.1m(안 걸림), 회전 뒤 자리 기준은 3.3m(걸림)."""
    import math as _m
    from behavior import Behavior
    from vtd_io import Obj, State, TL_UNSET

    def run(objs, rtor_point, junc=12.0, turn=1):
        b = Behavior()
        ego = State(); ego.x = ego.y = 0.0; ego.heading = 0.0
        ego.speed = 5.7; ego.t = 100.0; ego.tl_id = 0; ego.tl_state = TL_UNSET
        ego.objects = objs
        return b.plan(ego, 8.33, tl_stop_dist=None, junc_dist=junc,
                      route_turn=turn, rtor_point=rtor_point)

    # 실측 기하: 왼쪽 21m · 앞 8.9m · 47km/h · 방위 -81°, 회전 뒤 자리는 앞 8m 왼 6m
    dh = _m.radians(-81); v = 13.1
    crossing = [Obj(1, 8.9, 21.1, 0.0, dh, v, 4.9, 1.9, 1.5)]
    _v, _o, _t, reason = run(crossing, (8.0, 6.0))
    assert reason == "YIELD_CROSS", f"가로지르는 차에 안 선다 ({reason})"

    # ★같은 차라도 **자차 기준**으로 재면 안 걸린다 — 기준점이 중요하다는 회귀
    b = Behavior()
    ego = State(); ego.x = ego.y = 0.0; ego.heading = 0.0; ego.speed = 5.7
    ego.objects = crossing
    assert b._oncoming_clear(ego, at=None, gap=5.0, radius=8.0) is True
    assert b._oncoming_clear(ego, at=(8.0, 6.0), gap=5.0, radius=8.0) is False

    # ★비어 있으면 서행으로 지난다(멈추지 않는다)
    _v, _o, _t, reason = run([], (8.0, 6.0))
    assert reason == "NOSIG_LEFT", f"빈 교차로에서 뭘 하고 있나 ({reason})"

    # ★★**신호가 있는 교차로에는 개입하지 않는다** — 기존 신호 규칙이 담당한다
    _v, _o, _t, reason = run(crossing, (8.0, 6.0), junc=12.0, turn=1)
    b2 = Behavior()
    ego2 = State(); ego2.x = ego2.y = 0.0; ego2.heading = 0.0
    ego2.speed = 5.7; ego2.t = 100.0; ego2.tl_id = 5; ego2.tl_state = TL_UNSET
    ego2.objects = crossing
    r2 = b2.plan(ego2, 8.33, tl_stop_dist=30.0, junc_dist=12.0,
                 route_turn=1, rtor_point=(8.0, 6.0))[3]
    assert r2 != "YIELD_CROSS", f"신호 있는 교차로에 끼어든다 ({r2})"

    # ★우회전에는 안 건다(우회전 일시정지가 따로 담당)
    _v, _o, _t, reason = run(crossing, (8.0, 6.0), turn=-1)
    assert reason != "YIELD_CROSS", f"우회전에 걸었다 ({reason})"


def test_무신호_좌회전_양보는_횡단보도_앞에서_선다():
    """교차로 진입 목표보다 앞에 횡단보도가 있으면 본체 위까지 기어가서 서면 안 된다."""
    from behavior import Behavior
    from vtd_io import State, TL_UNSET

    e = State(); e.x = e.y = 0.0; e.heading = 0.0; e.speed = 5.7
    e.t = 100.0; e.tl_id = 0; e.tl_state = TL_UNSET; e.objects = []

    b = Behavior()
    b._oncoming_clear = lambda *_args, **_kwargs: False
    v, _o, _t, reason = b.plan(
        e, 8.33, tl_stop_dist=None, junc_dist=12.0, route_turn=1,
        rtor_point=(8.0, 6.0), yield_stop_dist=1.2)
    assert reason == "YIELD_CROSS", reason
    expected = b._stop_target_speed(1.2)
    assert abs(v - expected) < 1e-9, (v, expected)
    assert v < b._stop_target_speed(12.0 - b.stop_margin), v


def test_닿을_코스면_우선순위와_무관하게_선다():
    """실측 2026-08-27 — 주변교통 3판이 같은 날 같은 식으로 스쳤고, **셋 다 양보 규칙의
    조건 밖**이었다(우회전 중 · 뒤에서 추월해 파고든 차 · 직진 중 횡단):

      코스 A t=653  우회전 중, 왼쪽 90° 에서 34km/h  -> 실여유 -0.71m
      코스 B t=363  뒤에서 33km/h 로 추월해 들어옴   -> 실여유 -1.03m
      코스 D t=186  직진 중, 왼쪽 -99° 에서 28km/h   -> 실여유 -1.45m

    셋 다 1.1~2.0초 전부터 보이던 차인데 그 시간에 우리는 **가속**하고 있었다.
    규칙(제26조)으로는 못 잡으므로 기하(CPA)로 한 겹 더 깐다."""
    import math as _m
    from behavior import Behavior
    from vtd_io import Obj, State

    def gap_to(objs, ego_v):
        b = Behavior()
        ego = State(); ego.x = ego.y = 0.0; ego.heading = 0.0
        ego.speed = ego_v; ego.objects = objs
        return b._cpa_conflict(ego)

    # ① 코스 A 실측: fx 10.1 · fy 13.2 · 34.2km/h · dh -89° · 자차 17.7km/h
    a = [Obj(1, 10.1, 13.2, 0.0, _m.radians(-89), 9.5, 4.9, 1.9, 1.5)]
    assert gap_to(a, 4.9) is not None, "90° 로 다가오는 차를 못 본다"
    # ② 코스 B 실측: 뒤에서 추월 fx -5.0 · fy 4.9 · 33.1km/h · dh -17° · 자차 13.1km/h
    bb = [Obj(2, -5.0, 4.9, 0.0, _m.radians(-17), 9.2, 3.7, 1.6, 1.5)]
    assert gap_to(bb, 3.64) is not None, "뒤에서 파고드는 차를 못 본다"
    # ③ 코스 D 실측: fx 6.3 · fy 5.6 · 27.7km/h · dh -99° · 자차 13.4km/h
    d = [Obj(3, 6.3, 5.6, 0.0, _m.radians(-99), 7.7, 4.9, 1.9, 1.5)]
    assert gap_to(d, 3.72) is not None, "횡단하는 차를 못 본다"

    # ★★**나란히 가는 차에는 안 걸려야 한다** — 실측 2026-08-28 코스 A(사용자 지적):
    #   옆차로 49~54km/h 짜리 dh +1~3° 에 CPA_BRAKE(a=-5.00) 가 걸렸다. 3° 면 2.5초에
    #   1.8m 라 직선 외삽이 문턱을 넘긴다. 대향차(dh 163°)도 마찬가지.
    for deg, lab in ((1, "옆차로 +1°"), (3, "옆차로 +3°"), (-2, "옆차로 -2°"),
                     (163, "대향차 163°"), (174, "대향차 174°")):
        near = [Obj(9, 6.0, -3.0, 0.0, _m.radians(deg), 13.5, 4.4, 1.8, 1.5)]
        assert gap_to(near, 7.8) is None, f"{lab} 에 급제동한다"

    # ★★지나갈 차에는 안 걸려야 한다 — 여기서 막히면 아무 데도 못 간다
    # 옆 차로 병주(3.5m 옆, 같은 방향 같은 속도)
    para = [Obj(4, 12.0, 3.5, 0.0, 0.0, 8.0, 4.4, 1.8, 1.5)]
    assert gap_to(para, 8.0) is None, "나란히 가는 옆차로 차에 선다"
    # 반대차로 대향차(3.5m 옆, 마주 옴)
    onc = [Obj(5, 30.0, -3.5, 0.0, _m.pi, 11.0, 4.4, 1.8, 1.5)]
    assert gap_to(onc, 8.0) is None, "반대차로 대향차에 선다"
    # 내 뒤로 지나갈 차
    behind = [Obj(6, -12.0, 8.0, 0.0, _m.radians(-90), 9.0, 4.4, 1.8, 1.5)]
    assert gap_to(behind, 5.0) is None, "내 뒤로 갈 차에 선다"
    # 정지차·서행차는 여기서 안 본다(차체 안전망·ACC 담당)
    slow = [Obj(7, 8.0, 0.0, 0.0, 0.0, 0.0, 4.4, 1.8, 1.5)]
    assert gap_to(slow, 5.0) is None, "정지차를 CPA 로 잡는다"
    # 사람도 안 본다(보행자 규칙 담당)
    ped = [Obj(8, 8.0, 2.0, 0.0, _m.radians(-90), 3.0, 2.0, 0.6, 1.7)]
    assert gap_to(ped, 5.0) is None, "사람을 CPA 로 잡는다"


def test_적신호우회전_허가는_다음_신호등으로_안_넘어간다():
    """실측 2026-08-28 코스 A: tl 181 적신호 우회전 직후 5m 앞 tl 213(적색)을 그냥 통과.

        x=1341.4 y=145.7  tl=213/1  [RIGHT_ON_RED]
        x=1343.4 y=130.1  tl=0/0    a=+2.00      <- 적신호 통과

    `_rtor_go` 래치 해제 조건이 '다음 정지선이 멀 때'라, 정지선이 코앞이면 영영 안 풀렸다.
    """
    b = Behavior()
    # ① tl 181 적신호 우회전: 정지선 앞에서 선다 -> 허가
    b.plan(ego(v=8.33, tl_state=TL_RED, tl_id=181), 8.33,
           tl_stop_dist=20.0, route_turn=-1)
    hold_stop(b, lambda t: b.plan(ego(v=0.05, tl_state=TL_RED, tl_id=181, t=t), 8.33,
                                  tl_stop_dist=b.front_overhang + 1.0, route_turn=-1))
    v, _o, _t, reason = b.plan(ego(v=1.0, tl_state=TL_RED, tl_id=181, t=9.0), 8.33,
                               tl_stop_dist=b.front_overhang + 1.0, route_turn=-1)
    assert b._rtor_go, "적신호 우회전 허가가 나야 한다"
    assert reason == "RIGHT_ON_RED", reason

    # ② 회전 중 tl_id 가 잠깐 비는 건 해제 사유가 아니다(교차로 안 재정지 방지).
    #    실제 판이 그랬다 — 다음 정지선이 코앞(5m)이라 기존 해제 조건도 안 걸렸다.
    b.plan(ego(v=4.0, tl_state=TL_UNSET, tl_id=0), 8.33, route_turn=0, junc_dist=8.0)
    assert b._rtor_go, "회전 도중 tl_id=0 에서 래치가 풀리면 교차로 한복판에 선다"

    # ③ 다음 신호등(213) 적색 -> 허가는 무효. 정지선 앞에서 서야 한다.
    v, _o, _t, reason = b.plan(ego(v=8.0, tl_state=TL_RED, tl_id=213), 8.33,
                               tl_stop_dist=5.0, route_turn=0)
    assert not b._rtor_go, "허가가 다음 신호등으로 넘어갔다"
    assert reason == "RED_STOP", f"다음 적신호는 서야 한다 (reason={reason})"
    assert v < 1.0, f"정지선 5m 앞인데 v={v}"


def test_교차로_안에서는_우회전_일시정지를_무장하지_않는다():
    """실측 2026-08-28 코스 A 남행 (1339,157): 우회전은 35m 앞 다음 교차로인데
    지금 있는 교차로 때문에 junc_dist=0.0 → 그 자리에서 29.9km/h → a=-5.00 급정지.
    선 자리가 교차로 한복판이라 [법 제32조] 위반이기도 하다."""
    b = Behavior()
    # 교차로 안(junc_dist=0.0), 우회전은 앞에 있다
    v, _o, _t, reason = b.plan(ego(v=8.3), 8.33, route_turn=-1, junc_dist=0.0)
    assert not b._rt_armed, "교차로 안에서 무장하면 한복판에 선다"
    assert reason != "RIGHT_TURN_STOP", reason
    assert v > 5.0, f"교차로 안에서 급정지 (v={v})"
    # 교차로를 빠져나오면(junc_dist 가 살아나면) 정상 무장
    v, _o, _t, reason = b.plan(ego(v=8.3), 8.33, route_turn=-1, junc_dist=18.7)
    assert b._rt_armed, "교차로 입구가 보이면 무장해야 한다"
    assert reason == "RIGHT_TURN_STOP", reason


def _tw(fx, fy, spd, dh_deg, oid=7):
    """이륜차 한 대 — VTD 는 보행자도 자전거도 2.0x0.6x1.7 로 준다.
    (ds, d, speed, olen, owid, hgt, id, 원폭, 원길이, 방위차[rad])"""
    return (fx, fy, spd, 2.0, 0.60, 1.70, oid, 0.66, 2.0, math.radians(dh_deg))


def test_옆차로_달리는_이륜차에는_안_선다():
    """실측 2026-08-28 코스 A t=289.2 (1186,93): 경로횡 +3.4m 옆차로를 **60km/h** 로
    달리는 이륜차를 보고 YIELD_PED, 30.3 -> 10.4km/h. 보행자는 60km/h 로 못 간다."""
    b = Behavior()
    v, _o, _t, reason = b.plan(ego(v=8.4), 8.33, rf_objs=[_tw(0.5, 3.40, 16.4, 1)])
    assert reason != "YIELD_PED", reason
    assert v > 6.0, f"옆차로 이륜차에 감속하면 안 됨 (v={v})"


def test_마주오는_이륜차에는_안_선다():
    """실측 2026-08-28 코스 A t=688.4 (1188,-574): 경로횡 +2.7m 대향차로, 방위차 -177°,
    7.0m/s 이륜차를 보고 **완전정지**(a=-5.00)."""
    b = Behavior()
    v, _o, _t, reason = b.plan(ego(v=8.4), 8.33, rf_objs=[_tw(8.6, 2.80, 7.7, -177)])
    assert reason != "YIELD_PED", reason
    assert v > 6.0, f"대향 이륜차에 서면 안 됨 (v={v})"


def test_길을_가로지르는_자전거에는_선다():
    """반대로, **진로를 가로지르는** 자전거는 그대로 양보 대상이다 [법 제48조]."""
    b = Behavior()
    v, *_ = b.plan(ego(v=8.4), 8.33, rf_objs=[_tw(10.0, 3.00, 4.0, 90)])
    assert v < 5.0, f"횡단하는 자전거 앞에서는 줄여야 함 (v={v})"


def test_내_차로_안_이륜차에는_그대로_선다():
    """빠르든 나란하든 **내 차로 안**이면 사람처럼 다룬다 — 뒤를 따라간다."""
    b = Behavior()
    v, *_ = b.plan(ego(v=8.4), 8.33, rf_objs=[_tw(8.0, 0.40, 9.0, 0)])
    assert v < 5.0, f"내 차로 안 이륜차에는 줄여야 함 (v={v})"


def test_멀리_있는_사람에는_고정캡을_안_건다():
    """실측 2026-08-28 코스 A t=480.0 (1183,-505): **앞범퍼 64m 앞** 이륜차가 멀어지는
    중이라 PED_LEAVING -> vcmd 18km/h, 30.9km/h 에서 a=-5.00 급제동.
    사용자: "유령 보고 멈춤". YIELD_PED 는 거리에 비례하는데 이 고정 캡만 안 그랬다."""
    b = Behavior()
    far = _tw(67.8, 2.60, 5.7, 136)
    # 멀어지는 중으로 만들려면 두 프레임 필요(횡거리가 커져야 leaving)
    b.plan(ego(v=8.6), 8.33, rf_objs=[_tw(68.5, 2.80, 5.7, 136)])
    v, _o, _t, reason = b.plan(ego(v=8.6), 8.33, rf_objs=[_tw(67.8, 3.00, 5.7, 136)])
    assert reason != "PED_LEAVING", reason
    assert v > 7.0, f"64m 앞 사람에 18km/h 캡을 걸면 안 됨 (v={v})"
    del far

    # 가까우면(앞범퍼 10m) 종전대로 서행 통과
    b2 = Behavior()
    b2.plan(ego(v=8.6), 8.33, rf_objs=[(13.0, 2.80, 1.2, 0.6, 0.6, 1.75, 9, 0.6, 0.6, 0.0)])
    v2, _o, _t, r2 = b2.plan(ego(v=8.6), 8.33,
                             rf_objs=[(13.0, 3.00, 1.2, 0.6, 0.6, 1.75, 9, 0.6, 0.6, 0.0)])
    assert v2 <= 5.0, f"가까운 보행자 옆은 서행해야 함 (v={v2}, {r2})"


def test_무신호_교차로_정지_크레딧은_신호등에_못_쓴다():
    """실측 2026-08-28 스윕 코스 A: 무신호 교차로에서 선 크레딧(_rt_tl=None)이 살아남아
    다음 **신호** 교차로 tl 130 적신호를 14.8km/h 로 그냥 통과했다
    (t=260.6~263.0 (1101,-13)->(1093,-4), 그 앞 정지 없음)."""
    b = Behavior()
    # ① 무신호 교차로에서 우회전 일시정지 (tl_id=0)
    hold_stop(b, lambda t: b.plan(ego(v=0.1, tl_state=TL_UNSET, tl_id=0, t=t), 8.33,
                                  route_turn=-1, junc_dist=6.0))
    assert b._rt_done and b._rt_tl is None, (b._rt_done, b._rt_tl)
    # ② 다음 신호 교차로가 적색 -> 그 크레딧은 무효. 서야 한다.
    v, _o, _t, reason = b.plan(ego(v=8.0, tl_state=TL_RED, tl_id=130), 8.33,
                               tl_stop_dist=b.stop_margin + 2.0, route_turn=-1)
    assert reason != "RIGHT_ON_RED", reason
    assert v < 3.0, f"적신호에 그냥 지나가면 안 됨 (v={v}, {reason})"


def test_화살표_받고_들어간_좌회전은_신호가_끊겨도_계속_간다():
    """실측 2026-08-30 코스 E (1464,900) — 사용자: "정지선에 가까이 가서 멈추지도 않고
    혼자 멈췄다가 혼자 빨간불에 좌회전함".

      t=169~173.4 tl=221/GREEN_LEFT  25->23km/h 로 진입
      t=173.7     tl=0/UNSET         <- VTD 가 보고를 끊는다
      t=175.1     [YIELD_CROSS] a=-5.00  22.2 -> 0.8km/h  <- 교차로 **안**에서 급정지
      t=176.5     [NOSIG_LEFT]       <- 혼자 다시 출발

    `tl_stop_dist` 가 None 이 되면서 '무신호 좌회전' 가지로 떨어진 것이다.
    화살표를 받고 들어간 회전은 대향차 양보 대상이 아니고, 교차로 안에 서는 것
    자체가 [법 제32조] 위반이다."""
    from vtd_io import Obj
    b = Behavior()
    # ① 좌회전 화살표를 받고 진입
    b.plan(ego(v=6.5, tl_state=TL_GREEN_LEFT, tl_id=221), 8.33,
           tl_stop_dist=12.0, route_turn=1, junc_dist=12.0)
    assert b._left_go and b._left_tl == 221, (b._left_go, b._left_tl)

    # ② 회전 중 신호 보고가 끊긴다(tl_id=0, 정지선 미상) + 대향차가 있다
    onc = [Obj(9, 14.0, 6.0, 0.0, math.radians(175), 9.0, 4.4, 1.8, 1.5)]
    e = ego(v=6.2, tl_state=TL_UNSET, tl_id=0)
    e.objects = onc
    v, _o, _t, reason = b.plan(e, 8.33, route_turn=1, junc_dist=3.0)
    assert reason != "YIELD_CROSS", f"화살표 받고 들어갔는데 비보호로 강등됐다 ({reason})"
    assert v > 3.0, f"교차로 한복판에 섰다 (v={v}, {reason})"

    # ③ 교차로를 벗어나면 허가는 풀린다
    b.plan(ego(v=8.0, tl_state=TL_UNSET, tl_id=0), 8.33, route_turn=0, junc_dist=None)
    assert not b._left_go, "교차로를 벗어났는데 허가가 남아 있다"


def test_커브에서_옆차선_지키는_차에는_브레이크_안_잡는다():
    """사용자 지적 2026-08-30: "꺾인 도로에서 옆차선 차량 보고 브레이크 잡지 마.
    일반적인 경우는 다 차선 따라가지. 핸들을 갑자기 트는 경우에만 브레이크 잡아라."

    실측 코스 E (776,566): 옆차로 이륜차의 **경로기준 횡위치는 -3.5~-2.7 로 안정**인데
    자차기준 fy 는 우리가 커브를 도는 탓에 -3.1 -> +1.1 로 쓸려 들어와 CPA_BRAKE 가
    a=-5.00 을 걸었다. 자차 헤딩 기준 필터는 커브에서 무너진다."""
    from vtd_io import Obj
    b = Behavior()
    # 옆차로(경로횡 -3.0)를 제 차선 그대로 가는 차. 자차 헤딩과는 30° 어긋나 보인다.
    def frame(fy_ego, dh_deg):
        e = ego(v=8.3)
        e.objects = [Obj(5, 16.0, fy_ego, 0.0, math.radians(dh_deg), 6.5, 4.4, 1.8, 1.5)]
        return e
    rf = [(16.0, -3.0, 6.5, 4.4, 1.8, 1.5, 5, 1.8, 4.4, 0.0)]
    b.plan(frame(-3.0, 20), 8.33, rf_objs=rf)
    v, _o, _t, reason = b.plan(frame(-1.0, 30), 8.33, rf_objs=rf)
    assert reason != "CPA_BRAKE", f"차선 지키는 옆차로 차에 급제동 ({reason})"
    assert v > 5.0, f"급제동했다 (v={v})"

    # 반대로 **내 차로로 파고드는** 차는 그대로 잡는다
    b2 = Behavior()
    for lat in (-3.0, -2.0, -1.0):
        rf2 = [(14.0, lat, 8.0, 4.4, 1.8, 1.5, 6, 1.8, 4.4, 0.0)]
        e2 = ego(v=8.3)
        e2.objects = [Obj(6, 14.0, lat, 0.0, math.radians(35), 8.0, 4.4, 1.8, 1.5)]
        v2, _o, _t, r2 = b2.plan(e2, 8.33, rf_objs=rf2)
    assert v2 < 8.0, f"파고드는 차를 놓쳤다 (v={v2}, {r2})"




def test_커브_투영점프로_들어온_옆차는_앞차가_아니다():
    """실측 2026-08-30 코스 E (763,555): 갈래 2개 모퉁이(93° 굽이)에서 옆차선 차의
    경로투영이 프레임 사이에 몇 m 씩 튀어 내 차로 안으로 들어와 보였다 → FOLLOW
    vcmd=0 급제동. 실제 차는 프레임당 0.25m 안팎으로만 움직인다 — 그보다 큰 점프는
    투영 잡음이므로 앞차로 잡지 않는다."""
    b = Behavior()
    e = ego(v=8.0)
    # 옆차선(3.2m)을 몇 프레임 유지하다가 → 한 프레임에 0.9m 로 점프
    #  (fx=26: 차체통로 안전망 body_watch(25m) 밖 — 여기서 보려는 건
    #   FOLLOW 게이트지 안전망이 아니다. 안전망의 점프 폐기는 별도 테스트가 있다)
    for k in range(4):
        v, _o, _t, r = b.plan(e, 13.9, rf_objs=[(26.0, 3.2, 8.0, 4.4, 1.8, 1.5, 77)])
    v, _o, _t, r = b.plan(e, 13.9, rf_objs=[(26.0, 0.9, 8.0, 4.4, 1.8, 1.5, 77)])
    assert r not in ("FOLLOW", "HEADON_STOP", "OBSTACLE_STOP"), r
    assert v >= 8.0, v          # 제한속도(8.33) 그대로 = 브레이크 없음


def test_투영이_들락거려도_streak가_끊겨_안_잡는다():
    """FOLLOW ↔ LANE_KEEP 이 번갈아 나오던 그 판. 들어왔다 나갔다 하면 연속성이
    쌓이지 않으므로 계속 무시한다."""
    b = Behavior()
    e = ego(v=8.0)
    b.plan(e, 13.9, rf_objs=[(26.0, 3.2, 8.0, 4.4, 1.8, 1.5, 77)])
    for k in range(6):
        d = 0.8 if k % 2 == 0 else 3.2         # 매 프레임 경계를 들락거림
        v, _o, _t, r = b.plan(e, 13.9, rf_objs=[(26.0, d, 8.0, 4.4, 1.8, 1.5, 77)])
        assert r != "FOLLOW", (k, r)


def test_진짜_앞차는_연속성이_쌓이면_잡는다():
    """같은 차로를 계속 달리는 앞차 — 3프레임 안에 FOLLOW 가 잡혀야 한다."""
    b = Behavior()
    e = ego(v=8.0)
    got = None
    for k in range(5):
        v, _o, _t, r = b.plan(e, 13.9, rf_objs=[(26.0, 0.1, 4.0, 4.4, 1.8, 1.5, 77)])
        if r == "FOLLOW":
            got = k; break
    assert got is not None and got <= 3, got


def test_처음_보인_차가_내_차로면_즉시_잡는다():
    """스캔에 막 들어온 차(첫 관측)가 내 차로 안이면 기다릴 이유가 없다."""
    b = Behavior()
    e = ego(v=8.0)
    v, _o, _t, r = b.plan(e, 13.9, rf_objs=[(26.0, 0.1, 4.0, 4.4, 1.8, 1.5, 77)])
    assert r == "FOLLOW", r


def test_진짜_컷인은_연속_접근이라_잡는다():
    """물리적으로 가능한 속도(0.15m/프레임)로 좁혀 들어오는 차는 cutting_in 으로 잡는다."""
    b = Behavior()
    e = ego(v=8.0)
    got = False
    d = 2.9
    for k in range(14):
        d -= 0.15
        v, _o, _t, r = b.plan(e, 13.9, rf_objs=[(10.0, d, 7.0, 4.4, 1.8, 1.5, 77)])
        if r in ("FOLLOW", "CUTIN", "FOLLOW_BRAKE") or v < 13.0:
            got = True
    assert got


def test_정지물은_점프여도_첫_프레임부터_선다():
    """정지 장애물은 연속성 필터 대상이 아니다 — 늦게 서는 쪽이 더 위험하다."""
    b = Behavior()
    e = ego(v=8.0)
    for k in range(3):
        b.plan(e, 13.9, rf_objs=[(26.0, 3.2, 0.0, 4.4, 1.8, 1.5, 77)])
    v, _o, _t, r = b.plan(e, 13.9, rf_objs=[(26.0, 0.0, 0.0, 4.4, 1.8, 1.5, 77)])
    assert r == "OBSTACLE_STOP", r




def test_안전망도_움직이는_차의_투영점프_프레임은_버린다():
    """차체통로 안전망(NARROW)은 FOLLOW 와 같은 rf_objs 를 쓰므로 유령 투영에 같이
    속는다. 움직이는 차가 한 프레임에 2m 넘게 '순간이동'했으면 그 프레임은 버린다."""
    b = Behavior()
    e = ego(v=8.0)
    e.t = 1.0
    b.plan(e, 13.9, rf_objs=[(12.0, 3.2, 8.0, 4.4, 1.8, 1.5, 77)])
    e.t = 1.05
    v, _o, _t, r = b.plan(e, 13.9, rf_objs=[(12.0, 0.9, 8.0, 4.4, 1.8, 1.5, 77)])
    assert not r.startswith("NARROW"), r


def test_안전망은_정지물_점프는_그대로_본다():
    """정지물을 늦게 보는 쪽이 더 위험하다 — 점프 폐기는 움직이는 차에만 적용된다."""
    b = Behavior()
    e = ego(v=8.0)
    e.t = 1.0
    b.plan(e, 13.9, rf_objs=[(12.0, 3.2, 0.0, 4.4, 1.8, 1.5, 77)])
    e.t = 1.05
    v, _o, _t, r = b.plan(e, 13.9, rf_objs=[(12.0, 0.9, 0.0, 4.4, 1.8, 1.5, 77)])
    assert r.startswith("NARROW") or r == "OBSTACLE_STOP", r


def test_안전망은_연속으로_좁혀오는_차는_계속_잡는다():
    """EV_CUTIN 회귀 — 물리적으로 가능한 속도로 좁혀오는 차는 점프 폐기에 안 걸린다."""
    b = Behavior()
    e = ego(v=8.0)
    d, caught = 3.2, False
    for k in range(12):
        e.t = 1.0 + k * 0.05
        d -= 0.12                              # 2.4m/s 횡접근(가능한 속도)
        v, _o, _t, r = b.plan(e, 13.9, rf_objs=[(12.0, d, 8.0, 4.4, 1.8, 1.5, 77)])
        if r.startswith("NARROW") or v < 6.0:
            caught = True
    assert caught




def test_나란히_가는_옆차선_차의_잡음을_컷인으로_안_읽는다():
    """실측 2026-08-30 코스 E (1260,-355), 사용자 지적 "옆차선 차 보고 브레이크":
    옆 차로 차가 2초에 5cm(접근속도 0.000 m/s)만 움직였는데 ±1cm 양자화 잡음이
    끼어듦으로 읽혀 앞차 취급 -> FOLLOW_BRAKE 로 30->24.5km/h 급제동했다."""
    b = Behavior()
    e = ego(v=8.3)
    d, brake = 3.07, False
    for k in range(50):
        e.t = 1.0 + k * 0.04
        d -= 0.01 if k % 2 == 0 else -0.005      # 5cm/2초 + ±1cm 잡음
        v, _o, _t, r = b.plan(e, 8.33, rf_objs=[(10.6, -d, 14.0, 4.6, 1.9, 1.4, 41)])
        if v < 8.0 or r in ("FOLLOW", "FOLLOW_BRAKE", "HEADON_STOP"):
            brake = True
    assert not brake, "나란히 가는 옆차에 브레이크를 밟았다"


def test_진짜_끼어드는_차는_여전히_잡는다():
    """3.5m 를 3초에 좁히는 실제 컷인(~1.2m/s)."""
    b = Behavior()
    e = ego(v=8.3)
    d, caught = 3.1, False
    for k in range(40):
        e.t = 1.0 + k * 0.04
        d -= 1.2 * 0.04
        v, _o, _t, r = b.plan(e, 8.33, rf_objs=[(14.0, -max(d, 0.0), 8.0, 4.6, 1.9, 1.4, 42)])
        if v < 8.0 or r in ("FOLLOW", "FOLLOW_BRAKE"):
            caught = True
    assert caught, "진짜 끼어드는 차를 놓쳤다"


def test_느린_차선변경도_컷인으로_잡는다():
    """차선변경 ~0.7m/s 는 잡음(0.1m/s)과 실제 기동 사이 — 잡아야 한다."""
    b = Behavior()
    e = ego(v=8.3)
    d, caught = 3.1, False
    for k in range(60):
        e.t = 1.0 + k * 0.04
        d -= 0.7 * 0.04
        v, _o, _t, r = b.plan(e, 8.33, rf_objs=[(14.0, -max(d, 0.0), 8.0, 4.6, 1.9, 1.4, 43)])
        if v < 8.0 or r in ("FOLLOW", "FOLLOW_BRAKE"):
            caught = True
    assert caught


def test_컷인은_한_프레임_잡음으로_켜지지_않는다():
    """cutin_hold — 한 프레임 크게 튄 것만으로는 끼어듦이 아니다."""
    b = Behavior()
    e = ego(v=8.3)
    for k in range(6):
        e.t = 1.0 + k * 0.04
        v, _o, _t, r = b.plan(e, 8.33, rf_objs=[(12.0, -3.1, 9.0, 4.6, 1.9, 1.4, 44)])
    e.t = 1.0 + 6 * 0.04
    v, _o, _t, r = b.plan(e, 8.33, rf_objs=[(12.0, -3.0, 9.0, 4.6, 1.9, 1.4, 44)])
    assert r not in ("FOLLOW", "FOLLOW_BRAKE", "HEADON_STOP"), r




def test_트랙이_끊겼다_돌아온_차를_정면접근으로_오판하지_않는다():
    """감시 반경을 나갔다 20초 뒤 돌아온 차: `_prev_fx` 가 살아 있으면
    close=(80-30)/0.04 = 1250 m/s -> head_on -> HEADON_STOP 급제동(유령).
    dt_f 는 0.5s 넘는 공백이면 공칭값(0.04)으로 떨어져 시간으로도 안 걸러진다."""
    b = Behavior()
    e = ego(v=8.0)
    e.t = 1.0
    b.plan(e, 13.9, rf_objs=[(80.0, 0.1, 6.0, 4.4, 1.8, 1.5, 55)])
    e.t = 21.0                                     # 20초 공백(반경 밖에 있었다)
    v, _o, _t, r = b.plan(e, 13.9, rf_objs=[(30.0, 0.1, 6.0, 4.4, 1.8, 1.5, 55)])
    assert r != "HEADON_STOP", r
    assert v > 5.0, v


def test_이어지는_트랙은_그대로_판정한다():
    """진짜 정면 접근(연속 관측)은 여전히 잡는다 — 정리 로직이 과하면 안 된다."""
    b = Behavior()
    e = ego(v=8.0)
    caught = False
    fx = 60.0
    for k in range(30):
        e.t = 1.0 + k * 0.04
        fx -= (8.0 + 10.0) * 0.04                  # 접근속도 18 m/s(정면)
        v, _o, _t, r = b.plan(e, 13.9, rf_objs=[(fx, 0.1, 10.0, 4.4, 1.8, 1.5, 56)])
        if r == "HEADON_STOP":
            caught = True
    assert caught


def test_트랙정리가_id_없는_객체에서_죽지_않는다():
    b = Behavior()
    e = ego(v=8.0)
    e.t = 1.0
    v, _o, _t, r = b.plan(e, 13.9, rf_objs=[(30.0, 3.2, 6.0, 4.4, 1.8, 1.5)])  # 6칸 = id 없음
    assert v > 0




def test_적색점멸은_2m_이내에서_05초_이상_서야_인정():
    """[대회 안내문 · 평가항목 9] "범퍼 기준 정지선 2.0m 미만에서 0.5초 이상 1회 이상
    정지 필요 / 무정차 통과 시 중대(-6)".
    9m 에서 선 걸 인정하면 정지선 9m 뒤에 서고 그냥 통과하는 그림이 된다."""
    from vtd_io import TL_FLASH
    b = Behavior()
    assert b.flash_near <= 2.0, b.flash_near
    e = ego(v=0.1, tl_state=TL_FLASH, tl_id=117)
    # ① 멀리서(범퍼 5m) 선 것은 인정 안 된다
    e.t = 1.0
    for k in range(20):
        e.t = 1.0 + k * 0.1
        b.plan(e, 8.33, tl_stop_dist=5.0 + b.front_overhang)
    assert 117 not in b._flash_done, "멀리서 선 것을 인정했다"
    # ② 2m 안이어도 유지 시간 전에는 인정 안 된다
    e.t = 10.0
    _v, _o, _t, r = b.plan(e, 8.33, tl_stop_dist=1.0 + b.front_overhang)
    assert 117 not in b._flash_done and r == "FLASH_STOP", r
    # ③ 유지 시간을 채우면 통과 허용. ★기준(0.5초)이 아니라 우리 문턱(stop_hold_s)이다 —
    #   실측 신호 117 이 1km/h 이하 0.32초라 기준에 못 미쳤기 때문이다.
    assert b.stop_hold_s > 0.5 and b.stop_hold_v < 1 / 3.6
    e.t = 10.0 + b.stop_hold_s + 0.05
    b.plan(e, 8.33, tl_stop_dist=1.0 + b.front_overhang)
    assert 117 in b._flash_done


def test_적색점멸은_2m_경계와_정지선_통과_후_정지를_인정하지_않는다():
    from vtd_io import TL_FLASH

    for tl, gap in ((201, 2.0), (202, 2.01), (203, -0.01)):
        b = Behavior()
        hold_stop(b, lambda t, tl=tl, gap=gap: b.plan(
            ego(v=0.0, tl_state=TL_FLASH, tl_id=tl, t=t), 8.33,
            tl_stop_dist=b.front_overhang + gap))
        assert tl not in b._flash_done, f"범퍼 거리 {gap}m 정지를 인정했다"


def test_점멸_정지_도중_다시_움직이면_처음부터():
    from vtd_io import TL_FLASH
    b = Behavior()
    e = ego(v=0.1, tl_state=TL_FLASH, tl_id=118)
    e.t = 1.0
    b.plan(e, 8.33, tl_stop_dist=1.0 + b.front_overhang)
    e.speed = 2.0                       # 다시 굴렀다
    e.t = 1.2
    b.plan(e, 8.33, tl_stop_dist=1.0 + b.front_overhang)
    e.speed = 0.1
    e.t = 1.4
    b.plan(e, 8.33, tl_stop_dist=1.0 + b.front_overhang)
    e.t = 1.7                            # 처음부터 0.3초밖에 안 됐다
    b.plan(e, 8.33, tl_stop_dist=1.0 + b.front_overhang)
    assert 118 not in b._flash_done




def test_화살표_없는_교차로에서_녹색에_오래_안_선다():
    """[대회 안내문 · 평가항목 8] 녹색신호 통과 — 정지선 30m 이내 이유 없는 정차
    5초 이상 경미(-3) / 10초 이상 중대(-6).
    옛 규칙('녹색을 두 번 봐야 화살표 없음 확정')은 첫 녹색 한 주기를 통째로 버려
    그것만으로 -6 이었다. 이번 녹색에서 green_probe 안에 결론을 낸다."""
    from vtd_io import TL_GREEN
    b = Behavior()
    assert b.green_probe < 5.0, b.green_probe
    e = ego(v=1.0, tl_state=TL_GREEN, tl_id=108)
    e.t = 100.0
    _v, _o, _t, r = b.plan(e, 8.33, tl_stop_dist=20.0, route_turn=+1)
    assert r in ("WAIT_LEFT_ARROW",), r          # 처음엔 화살표를 기다린다
    e.t = 100.0 + b.green_probe + 0.2            # 그 안에 화살표가 없었다
    _v, _o, _t, r = b.plan(e, 8.33, tl_stop_dist=20.0, route_turn=+1)
    assert r in ("UNPROTECTED_LEFT", "YIELD_ONCOMING"), r


def test_화살표가_곧_나오는_신호는_기다린다():
    """green_probe 안에 화살표가 나오면 그건 보호 좌회전이다 — 그대로 간다."""
    from vtd_io import TL_GREEN, TL_GREEN_LEFT
    b = Behavior()
    e = ego(v=1.0, tl_state=TL_GREEN, tl_id=109)
    e.t = 200.0
    b.plan(e, 8.33, tl_stop_dist=20.0, route_turn=+1)
    e.tl_state = TL_GREEN_LEFT
    e.t = 201.0
    _v, _o, _t, r = b.plan(e, 8.33, tl_stop_dist=20.0, route_turn=+1)
    assert r not in ("WAIT_LEFT_ARROW", "UNPROTECTED_LEFT"), r
    assert 109 in b._tl_has_left


def test_무신호_좌회전_양보는_교차로_안에서_무장하지_않는다():
    """실측 2026-08-30 코스 E (1454,940): jx=1 교차로 **한복판**에서
    `_junction_ahead` 가 0.0 을 주는 바람에 `_stop_target_speed(0-5.0)`=0 이 되어
    회전 도중 **2.5초 완전정지**했다(방위 114°->151°, d_ego 는 내내 0.00).
    [법 제32조] 교차로 내 정차이고, 대회 채점으로는 그 2.5초가 차로 판정 밖이라
    항목 4·5(각 중대 -6)에 걸린다.
    우회전 일시정지(rt_min_arm)에서 이미 고친 버그가 여기만 안 들어가 있었다."""
    from vtd_io import TL_UNSET

    class _O:
        pass

    def _cross():
        o = _O()
        o.x, o.y, o.heading = 6.0, -8.0, 1.57
        o.speed, o.length, o.width, o.height = 10.0, 4.4, 1.8, 1.5
        return o

    #  ① 교차로 **밖**(12m) — 가로지르는 차가 있으면 서서 양보한다
    b = Behavior()
    e = ego(v=4.1, tl_state=TL_UNSET, tl_id=0)
    e.objects = [_cross()]
    e.t = 1.0
    _v, _o, _t, r = b.plan(e, 8.33, tl_stop_dist=None, route_turn=+1, junc_dist=12.0)
    assert r == "YIELD_CROSS", r

    #  ② 교차로 **안**(0m) — 이 규칙은 무장하지 않는다. 서면 그게 제32조 위반이다.
    b2 = Behavior()
    e2 = ego(v=4.1, tl_state=TL_UNSET, tl_id=0)
    e2.objects = [_cross()]
    e2.t = 1.0
    _v, _o, _t, r2 = b2.plan(e2, 8.33, tl_stop_dist=None, route_turn=+1, junc_dist=0.0)
    assert r2 != "YIELD_CROSS", r2


def test_교차로_안이어도_닿을_코스면_선다():
    """양보 규칙을 껐다고 충돌까지 허용하는 건 아니다 — CPA 가드가 아래 겹으로 남는다."""
    from vtd_io import TL_UNSET

    class _O:
        pass

    o = _O()
    o.x, o.y, o.heading = 6.0, -8.0, 1.57
    o.speed, o.length, o.width, o.height = 10.0, 4.4, 1.8, 1.5
    b = Behavior()
    e = ego(v=4.1, tl_state=TL_UNSET, tl_id=0)
    e.objects = [o]
    e.t = 1.0
    v, _o2, _t, r = b.plan(e, 8.33, tl_stop_dist=None, route_turn=+1, junc_dist=0.0)
    assert r == "CPA_BRAKE" and v < 4.0, (r, v)


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"  ✅ {name}")
            except AssertionError as e:
                fails += 1
                print(f"  ❌ {name}: {e}")
    print("실패 없음" if not fails else f"{fails}건 실패")
    sys.exit(1 if fails else 0)


# ---------------------------------------------------------------- 2026-09-06 실주행 지적
def test_비보호좌회전_양보에_뒤차는_상대가_아니다():
    """코스 E 신호108: 우리 차로 뒤 78m 에서 40km/h 로 오는 차의 CPA 가 내 자리라 11초를 양보했다."""
    b = Behavior()
    rear = Obj(502, -40.0, 0.0, 0.0, 0.0, 11.0, 4.8, 1.9, 1.5)     # 내 뒤 40m, 같은 방향
    _v, _o, _t, why = _no_arrow(b, objs=[rear])
    assert why == "UNPROTECTED_LEFT", why


def test_비보호좌회전_양보에_대향차는_여전히_상대다():
    b = Behavior()
    _v, _o, _t, why = _no_arrow(b, objs=[_oncoming(60.0)])
    assert why == "YIELD_ONCOMING", why


def test_앞에서_나보다_빨리_달아나는_이륜차는_통로가_아니다():
    """코스 H (983,667): 10m 앞 우측 2.4m 이륜차 42km/h(우리 35) 에 35->6km/h 급제동."""
    b = Behavior()
    v0, _o, _t, _r = b.plan(ego(v=9.7), 13.9)                       # 아무도 없을 때
    v, _o, _t, r = b.plan(ego(v=9.7), 13.9,
                          rf_objs=[obj(9.8, -2.4, speed=11.8, L=2.0, W=0.6, H=1.7)])
    assert r not in ("NARROW_PASS", "NARROW_BLOCK"), r
    assert abs(v - v0) < 1e-6, (v, v0)                              # 속도를 깎지 않는다
    # 같은 자리에 **서 있는** 이륜차는 여전히 통로다
    b2 = Behavior()
    v2, _o, _t, r2 = b2.plan(ego(v=9.7), 13.9,
                             rf_objs=[obj(9.8, -2.4, speed=0.0, L=2.0, W=0.6, H=1.7)])
    assert v2 < v, (v2, v)


def test_뒤에서_오는_빠른_이륜차는_그대로_본다():
    """코스 E 의 끼어든 이륜차(fx -3.7 -> 0.6, 61km/h)는 '앞에서 달아나는' 예외에 안 걸린다."""
    b = Behavior()
    v, _o, _t, r = b.plan(ego(v=9.0), 13.9,
                          rf_objs=[obj(-3.7, -2.4, speed=16.9, L=2.0, W=0.6, H=1.7)])
    assert r in ("NARROW_PASS", "NARROW_BLOCK"), r
