"""옆에서 가로질러 오는 자전거를 CPA 가드가 못 봤다 — 2026-09-11 코스 A TRV 접촉.

실측(주변교통+사람+자전거 판, 신호143 적신호 출발 직후)
    t=41.6  25m 앞 · 27m 오른쪽에 2.0x0.6x1.7 물체가 7m/s(25km/h)로 보이기 시작
    t=41~44 우리는 적신호 출발 가속 중(vcmd 48km/h, a=+2.00)
    t=44.4  CPA_BRAKE — 최근접점이 앞범퍼 **1.2m** 앞. 전제동으로도 못 선다
    t=44.8  추정치가 4cm 흔들려 **브레이크가 0.4초 풀렸다**(cap=PROFILE)
    t=45.4  접촉(여유 -1.07m). 사용자: "좀 미리 스탑해야지 넘 늦게 스탑해서 사고나서 멈춤"

원인 둘
  ① 그 자전거는 취약대상이라 CPA 가드가 통째로 건너뛴다. 실제 판에서 걸린 건 그 물체가
     한 번 40km/h 를 넘겨 `_fast_ids` 에 들어간 **우연**이었다(녹화 재생: 취약대상 제외를
     그대로 두면 이 판에서 CPA 가 한 프레임도 안 걸린다).
  ② 직진 가정 CPA 는 **도는 물체**에 틀린다. 그 자전거는 상대방위가 83° -> 118° 로
     2초에 35° 돌며 파고들었다. 남은 시간에 비례해 반경을 넓혀야 미리 걸린다.

녹화 대조(코스 H 주변교통 409초 · H TRV 593초): ①은 +0프레임, ②는 +1 / +15프레임.
"""
import math
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from behavior import Behavior                                          # noqa: E402
from vtd_io import Obj, State                                          # noqa: E402


def _scene(fx, fy, ego_v, obj_v=6.9, deg=83.0, oid=7):
    """자차는 원점·방위 0. 물체는 자차 기준 (fx,fy) 에 상대방위 `deg`."""
    s = State()
    s.x = s.y = s.heading = 0.0
    s.speed = ego_v
    s.objects = [Obj(id=oid, x=fx, y=fy, z=0.0, heading=math.radians(deg),
                     speed=obj_v, length=2.0, width=0.6, height=1.7)]
    return s


def test_가로지르는_자전거를_차처럼_본다():
    """실측 t=44.4 기하. 취약대상 제외를 그대로 두면 이 프레임이 안 걸린다."""
    b = Behavior()
    assert b._cpa_conflict(_scene(11.6, -7.3, 8.28, deg=89.0)) is not None


def test_걷는_사람은_여전히_보행자_규칙_몫이다():
    """문턱은 `cpa_vru_v`(3.0m/s = 10.8km/h). 걷기 1.4 · 느린 조깅 2.5 는 안 넘는다."""
    b = Behavior()
    assert b.cpa_vru_v == 3.0
    for sp in (1.4, 2.5, 2.9):
        assert b._cpa_conflict(_scene(11.6, -7.3, 8.28, obj_v=sp, deg=89.0)) is None, sp


def test_멀리_내다본_예측일수록_반경을_넓힌다():
    """실측 t=43.80 기하(접촉 1.6초 전). 직진 가정으로는 3.66m 비껴 가는 것으로 나와
    문턱 2.47m 를 못 넘었고, 실제로 걸린 t=44.4 에는 최근접점이 앞범퍼 1.2m 앞이라
    전제동으로도 못 섰다."""
    b = Behavior()
    assert b.cpa_unc == 0.8
    got = b._cpa_conflict(_scene(16.0, -11.7, 7.16, deg=84.0))
    assert got is not None, "1.6초 전에는 걸려야 한다"
    assert got > 2.0, got                       # 최근접점이 앞범퍼보다 멀다 = 아직 설 수 있다
    old = Behavior()
    old.cpa_unc = 0.0                           # 옛 식(직진 가정 그대로)
    assert old._cpa_conflict(_scene(16.0, -11.7, 7.16, deg=84.0)) is None


def test_추정치가_흔들려도_브레이크가_안_풀린다():
    """실측 t=44.8: 예측 여유 2.51m 대 문턱 2.45m — **4cm** 차이로 0.4초 풀렸다."""
    b = Behavior()
    for fx, fy, ev, deg in ((11.6, -7.3, 8.28, 89.0),
                            (8.7, -4.4, 6.83, 97.0),
                            (5.7, -1.7, 5.97, 109.0)):
        assert b._cpa_conflict(_scene(fx, fy, ev, deg=deg)) is not None, (fx, fy)


def test_지나갈_차에는_안_걸린다():
    """반경은 실제 차체(+여유+불확실성)다 — 옆으로 충분히 비껴 가면 안 걸린다."""
    b = Behavior()
    assert b._cpa_conflict(_scene(30.0, -30.0, 8.0, obj_v=6.9, deg=89.0)) is None
    # 나란히 가는 차(방위차 0)·대향차(180)는 애초에 제외
    assert b._cpa_conflict(_scene(11.6, -7.3, 8.28, deg=0.0)) is None
    assert b._cpa_conflict(_scene(11.6, -7.3, 8.28, deg=180.0)) is None
