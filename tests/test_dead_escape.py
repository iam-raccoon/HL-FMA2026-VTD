"""사고현장 우회 — **죽은 차** 앞 마지막 탈출 (2026-09-09 코스 E (915,216) 65초 교착·미완주).

장면(자차 기준 · 경로 폭): 14.6m 앞 자전거(-140°, +0.0~+1.75) · 15.6m 앞 직각 승용차(100°,
-5.4~-1.0) · 우측 차로 8.8m 앞 정지차 · 대향 차로 32m 앞 정지차. 차로계획 j=1, l=1.51 r=4.45
xl=xr=0, 물리 폭 pl=4.25 pr=4.5. 옛 코드는 우측(막힘)만 후보로 고르고 영영 FOLLOW 였다.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "vtd"))
from drive import DrivingStack, straight_route                         # noqa: E402
from overtake import Overtaker                                         # noqa: E402

PLAN_E = {"lane": -1, "w": 3.02, "l": 1.51, "r": 4.45, "xl": 0.0, "xr": 0.0, "need": 0.0,
          "sig": 0, "j": 1, "jx": 1, "lim": 13.889, "pl": 4.25, "pr": 4.5}


def _row(ds, d, spd, olen, owid, oid, raw_w=None, raw_l=None, dh=0.0):
    """rf_objs 한 줄: (ds, d, speed, olen, owid, hgt, id, raw_w, raw_l, dh)."""
    return (ds, d, spd, olen, owid, 1.5, oid, raw_w or owid, raw_l or olen, dh)


BIKE = _row(14.6, 0.88, 0.0, 1.74, 1.74, 901, raw_w=0.6, raw_l=2.0, dh=-2.44)   # 누운 자전거
CAR90 = _row(15.6, -3.16, 0.0, 2.5, 4.41, 902, raw_w=1.8, raw_l=4.2, dh=1.75)   # 직각 승용차
RIGHT = _row(8.8, -2.77, 0.0, 4.6, 2.03, 903)                                    # 우측 차로 정지차
ONCOM = _row(32.4, 3.48, 0.0, 4.0, 1.89, 904)                                    # 대향 정지차


def _stack():
    return DrivingStack(route=straight_route(300.0))


def _mark_still(st, since, ids, moved=()):
    for i in ids:
        st.overtaker.still_since[i] = since
    for i in moved:
        st.overtaker.was_moving.add(i)


# ---------------------------------------------------------------- 죽은 것 판정
def test_진로를_막은_것이_전부_45초_서_있어야_죽은_것이다():
    """문턱 45초의 근거: 로그 11판에서 **앞차가 다시 출발한** 대기의 최대가 13.7초였고,
    이 맵 적색이 18초라 정상 대기도 20초에 닿을 수 있다(drive.DEAD_BLOCK_S 주석)."""
    st = _stack()
    objs = [BIKE, CAR90, RIGHT, ONCOM]
    _mark_still(st, since=100.0, ids=(901, 902, 903, 904))
    dead, rows = st._dead_blockers(objs, 0.0, now=130.0)
    assert not dead and {r[6] for r in rows} == {901, 902}         # 막은 건 둘 — 30초는 아직
    dead, rows = st._dead_blockers(objs, 0.0, now=146.0)
    assert dead and {r[6] for r in rows} == {901, 902}             # 우측 차로 차·대향차는 막은 게 아니다


def test_하나라도_움직이면_죽은_게_아니다():
    st = _stack()
    moving_car = _row(15.6, -3.16, 2.0, 2.5, 4.41, 902)
    _mark_still(st, since=0.0, ids=(901,))
    dead, _ = st._dead_blockers([BIKE, moving_car], 0.0, now=60.0)
    assert not dead


def test_달리다_멈춘_차도_같은_45초를_본다():
    st = _stack()
    _mark_still(st, since=0.0, ids=(901, 902), moved=(902,))
    assert not st._dead_blockers([BIKE, CAR90], 0.0, now=30.0)[0]
    assert st._dead_blockers([BIKE, CAR90], 0.0, now=46.0)[0]
    assert st.DEAD_BLOCK_S == st.DEAD_BLOCK_MOVED_S == 45.0


def test_막은_게_없으면_죽은_것도_없다():
    st = _stack()
    assert st._dead_blockers([RIGHT, ONCOM], 0.0, now=999.0) == (False, [])


# ---------------------------------------------------------------- 탈출 폭
def test_코스E_장면은_좌측_3_19m_우측은_불가():
    st = _stack()
    rows = [BIKE, CAR90]
    w = st._dead_escape_width(PLAN_E, rows, 0.0, +1.0)
    # 최소 3.19(자전거 왼끝 1.75 + 반폭 + 0.5) 는 되고, 물리 폭 4.25 가 허락하는 3.31 까지 벌린다
    assert w is not None and abs(w - (PLAN_E["pl"] - st.HALF_WIDTH)) < 0.02
    assert w >= 1.75 + st.HALF_WIDTH + st.DEAD_ESCAPE_CLEAR - 1e-6
    wide = st._dead_escape_width(dict(PLAN_E, pl=6.0), rows, 0.0, +1.0)
    assert abs(wide - (1.75 + st.HALF_WIDTH + st.DEAD_ESCAPE_CLEAR_MAX)) < 0.02   # 넓어도 1.0 까지만
    assert st._dead_escape_width(PLAN_E, rows, 0.0, -1.0) is None  # 5.36+0.94+0.5+0.94 > 4.5


def test_물리_폭이_모자라면_안_나간다():
    st = _stack()
    plan = dict(PLAN_E, pl=4.0)                                    # 4.13 필요
    assert st._dead_escape_width(plan, [BIKE, CAR90], 0.0, +1.0) is None
    assert st._phys_room(PLAN_E, +1.0) == 4.25 and st._phys_room(PLAN_E, -1.0) == 4.5
    assert st._phys_room({"l": 3.0, "r": 1.0}, +1.0) == 3.0        # pl 이 없는 옛 계획 = 합법 폭
    assert st._phys_room(None, +1.0) == 0.0


# ---------------------------------------------------------------- 추월기 탈출 설정
def _ov():
    o = Overtaker(lane_width=3.02, side=-1.0)
    o.set_lane_width(3.02)
    return o


def _objs():
    """ov_objs: (id, fx, fy, spd, olen, owid, hgt, raw_w, raw_l)."""
    return [(901, 14.6, 0.88, 0.0, 1.74, 1.74, 1.7, 0.6, 2.0),
            (902, 15.6, -3.16, 0.0, 2.5, 4.41, 1.3, 1.8, 4.2),
            (903, 8.8, -2.77, 0.0, 4.6, 2.03, 1.4, 1.8, 4.6),
            (904, 32.4, 3.48, 0.0, 4.0, 1.89, 1.5, 1.7, 4.0)]


def test_차로_반폭으로_보면_막히고_차폭으로_보면_비어_있다():
    o = _ov()
    assert not o.side_clear(_objs(), 0.0, 0.0, +1.0, block_fx=15.6)         # 평소: 자전거 왼끝이 목표차로에 걸친다
    assert o.set_escape(3.19, 0.943 + 0.25, 3.0, 10.0)
    assert o.side_clear(_objs(), 0.0, 0.0, +1.0, block_fx=15.6)             # 탈출: 차폭 기준 + 대향차는 구간 밖
    assert not o.side_clear(_objs(), 0.0, 0.0, +1.0)                        # block_fx 없이는 32m 대향차가 막는다


def test_탈출_설정은_FOLLOW_WAIT_에서_매_프레임_지워지고_복귀하면_지워진다():
    o = _ov()
    assert o.set_escape(3.19, 1.2, 3.0, 10.0) and o.escaping
    o.set_lane_width(3.02)
    assert not o.escaping                                                  # 바깥이 프레임마다 다시 건다
    assert o.set_escape(3.19, 1.2, 3.0, 10.0)
    o.state = "PASS"
    o.set_lane_width(3.02)
    assert o.escaping and abs(o.W) == 3.19                                 # 기동 중에는 그대로
    o.state = "RETURN"; o.offset = 0.1
    o.plan(type("E", (), {"speed": 1.0})(), [], now=10.0)
    assert o.state == "FOLLOW" and not o.escaping
    o.set_escape(3.19, 1.2, 3.0, 10.0)
    o.reset()
    assert not o.escaping


# ---------------------------------------------------------------- 지도 물리 폭
def test_실제_지도_코스E_교착점의_물리_폭():
    import json
    import math
    from build_lane_plan import Map, phys_room
    here = os.path.dirname(os.path.abspath(__file__))
    mp = Map(os.path.join(here, "..", "map", "HL_FMA_VTD_LivingLab.xodr"))
    rt = json.load(open(os.path.join(here, "..", "routes", "HL_FMA_NEW_E.json"), encoding="utf-8"))["ego_route"]
    i = min(range(len(rt)), key=lambda k: (rt[k][0] - 915.1) ** 2 + (rt[k][1] - 216.4) ** 2)
    h = math.atan2(rt[i + 3][1] - rt[i - 3][1], rt[i + 3][0] - rt[i - 3][0])
    pl, pr = phys_room(mp, rt[i][0], rt[i][1], h)
    assert pl >= 4.0 and pr >= 4.0, (pl, pr)                               # 대향 연결로가 왼쪽에 있다
    lp = json.load(open(os.path.join(here, "..", "routes", "HL_FMA_NEW_E_lane.json"), encoding="utf-8"))["pts"]
    assert lp[i]["pl"] == pl and lp[i]["pr"] == pr                         # 파일에 같은 값이 들어 있다


# ---------------------------------------------------------------- 무신호 횡단보도 앞에서는 안 나간다
def _state(x=0.0, y=0.0, hdg=0.0, v=0.0):
    from vtd_io import State
    st = State(); st.x, st.y, st.heading, st.speed, st.t = x, y, hdg, v, 0.0
    return st


def test_앞에_무신호_횡단보도가_있으면_사고현장_우회를_안_한다():
    """법령에 따라 정지한 차(보행자 대기)를 앞지르는 것은 제22조 위반이다.
    사람은 그 차에 가려 안 보일 수 있으므로 **횡단보도 존재만으로** 막는다."""
    def _stack_with_cw(x):
        cw = [{"road": "1", "s": 0.0, "x": x, "y": 0.0, "stops": [], "signal": False,
               "lim": 13.889, "zone": 0}]
        return DrivingStack(route=straight_route(300.0), crosswalks=cw)

    near = _stack_with_cw(20.0)
    assert near._cw_blocks_escape(_state(x=0.0), s_ego=0.0)            # 20m 앞 — 막는다
    assert not near._cw_blocks_escape(_state(x=25.0), s_ego=25.0)      # 이미 지났다
    far = _stack_with_cw(40.0)
    assert not far._cw_blocks_escape(_state(x=0.0), s_ego=0.0)         # 40m 앞 — 아직 멀다
    sig = DrivingStack(route=straight_route(300.0),
                       crosswalks=[{"road": "1", "s": 0.0, "x": 20.0, "y": 0.0, "stops": [],
                                    "signal": True, "lim": 13.889, "zone": 0}])
    assert not sig._cw_blocks_escape(_state(x=0.0), s_ego=0.0)         # 신호 있는 횡단보도는 신호가 지킨다
    bare = DrivingStack(route=straight_route(300.0))
    assert not bare._cw_blocks_escape(_state(), s_ego=0.0)             # DB 없으면 꺼짐


# ---------------------------------------------------------------- 래치: 나가기로 했으면 지나갈 때까지
def test_나가기로_한_뒤에는_서_있던_시간을_다시_묻지_않는다():
    """`stuck_long` 은 1cm 만 움직여도 리셋된다. 래치가 없으면 기어나가는 순간 조건이 꺼져
    오프셋이 0 으로 돌아가고, 다시 막혀 서고, 20초 뒤 또 나가는 순환이 된다(2026-09-10)."""
    st = _stack()
    st.overtaker.still_since[901] = 1000.0          # 방금 본 것 — 45초 안 됐다
    dead, rows = st._dead_blockers([BIKE], 0.0, now=1001.0)
    assert not dead and rows                        # 평소에는 아직 아니다
    dead, rows = st._dead_blockers([BIKE], 0.0, now=1001.0, latched=True)
    assert dead and rows                            # 래치 중에는 '아직 앞에 있나'만 본다


def test_막은_것을_지나가면_래치가_풀린다():
    st = _stack()
    behind = _row(-4.0, 0.88, 0.0, 1.74, 1.74, 901, raw_w=0.6, raw_l=2.0, dh=-2.44)
    dead, rows = st._dead_blockers([behind], 0.0, now=9999.0, latched=True)
    assert not dead and rows == []                  # 뒤로 갔다 -> 막은 것 없음 -> 래치 해제
