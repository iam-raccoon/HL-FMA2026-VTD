"""오프라인 채점기(eval/evaluate.py) 자체의 단위검증.

왜 필요한가(2026-08-30 변이 검증에서 배운 것): "회귀 21판 전부 PASS"는 채점기가
눈뜬장님이어도 나온다. 그래서 일부러 버그를 심어 봤는데(적신호 무시·보행자 무시·
제한속도 완화·정지물 무시) **9판 중 2판만 감점됐다.** 나머지는 채점기 잘못이 아니라
① 변이가 다른 안전층(추월 FSM·차체 안전망·Behavior 의 독립 구역제한)에 흡수돼
   주행 자체가 안 나빠졌거나
② 그 시나리오가 그 위반을 유발하지 않는 배치였다.
즉 스택을 통째로 돌리는 변이로는 **채점기의 눈**을 검증할 수 없다. 여기서는 합성
궤적을 직접 먹여 감점 항목이 하나씩 정확히 발화하는지 본다.

⚠️ 그리고 하나 더 — 채점기가 잡아도 **PASS 문턱(60점)이 그걸 가린다.**
   적신호 무시 변이가 80점(-20)이었는데 표기는 PASS 였다. 회귀는 반드시
   점수를 기준선과 비교해서 읽어야 한다(eval/expected_scores.json + run_all.sh).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "eval"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from evaluate import Scorer, PENALTY, GOAL_R          # noqa: E402
from scenario import Scenario, TrafficLight, Zone     # noqa: E402


def sc(**kw):
    base = dict(name="t", ego_start=[0, 0, 0], ego_goal=[500.0, 0.0],
                speed_limit=13.9)
    base.update(kw)
    return Scenario(**base)


def kinds(s):
    return [v.split("  ", 1)[1] for v in s.violations]


def test_속도위반은_110퍼센트_초과부터():
    s = Scorer(sc())
    s.update(1.0, 10.0, 0.0, 13.9 * 1.09, [], 0, 0.04)     # 9% 초과 -> 무감점
    assert not s.violations
    s.update(2.0, 20.0, 0.0, 13.9 * 1.2, [], 0, 0.04)      # 20% 초과 -> 감점
    assert s.score == 100 - PENALTY["speed_over"], s.violations


def test_보호구역은_가중감점():
    z = Zone("어린이보호구역", 50.0, 150.0, 8.33)
    s = Scorer(sc(zones=[z]))
    s.update(1.0, 100.0, 0.0, 8.33 * 1.2, [], 0, 0.04)
    assert s.score == 100 - PENALTY["school_over"], s.violations


def test_적신호는_넘는_순간만_위반():
    lt = TrafficLight(1, 100.0, [["RED", 60]])
    s = Scorer(sc(lights=[lt]))
    s.update(1.0, 95.0, 0.0, 5.0, [], 1, 0.04)             # 정지선 앞 -> 무감점
    assert not s.violations
    s.update(1.1, 99.0, 0.0, 5.0, [], 1, 0.04)             # 아직 앞 -> 무감점
    assert not s.violations
    s.update(1.2, 101.0, 0.0, 5.0, [], 1, 0.04)            # 넘는 순간 적색 -> 위반
    assert s.score == 100 - PENALTY["red_run"], s.violations


def test_녹색에_넘으면_무감점():
    lt = TrafficLight(1, 100.0, [["GREEN", 60]])
    s = Scorer(sc(lights=[lt]))
    s.update(1.0, 95.0, 0.0, 5.0, [], 3, 0.04)
    s.update(1.1, 101.0, 0.0, 5.0, [], 3, 0.04)            # 녹색(3)에 통과
    assert not s.violations


def test_정지선_너머_스폰은_위반이_아니다():
    """px(직전 프레임) 기준이라, 이미 지난 정지선은 '넘는 순간'이 없다."""
    lt = TrafficLight(1, 100.0, [["RED", 60]])
    s = Scorer(sc(lights=[lt]))
    s.update(1.0, 150.0, 0.0, 5.0, [], 1, 0.04)            # 첫 관측부터 너머
    s.update(1.1, 155.0, 0.0, 5.0, [], 1, 0.04)
    assert not s.violations


def test_보행자_3m_근접은_주행중일때만():
    #  ★충돌은 상자 대 상자(SAT)다(2026-09-09, evaluate 주석). 자차 상자는 뒷축 기준
    #    앞 +3.81 · 뒤 -1.04 · 반폭 0.943 이라 앞 2.8m 는 **상자 안**이다(옛 근사 2.55 는
    #    뒷축을 중심으로 잡아 틀렸다). 옆구리 1.8m 에 두면 실여유 +0.66 으로 안 닿고,
    #    뒷축 직선거리 1.87m 는 3m 근접 안이다.
    ped = {"id": 9, "type": "pedestrian", "x": 10.5, "y": 1.8, "size": [0.3, 0.4, 1.7]}
    s = Scorer(sc())
    s.update(1.0, 10.0, 0.0, 0.5, [ped], 0, 0.04)          # 거의 정지 -> 무감점
    assert not s.violations
    s2 = Scorer(sc())
    s2.update(1.0, 10.0, 0.0, 5.0, [ped], 0, 0.04)         # 주행 중 2.2m -> 감점
    assert s2.score == 100 - PENALTY["ped_near"], s2.violations


def test_충돌은_박스_겹침():
    car = {"id": 7, "type": "vehicle", "x": 12.0, "y": 0.5, "size": [4.4, 1.8, 1.5]}
    s = Scorer(sc())
    s.update(1.0, 10.0, 0.0, 5.0, [car], 0, 0.04)          # dx=2.0<4.6, dy=0.5<1.85
    assert s.collided and s.score == 100 - PENALTY["collision"], s.violations


def test_급감속과_리스폰_구분():
    s = Scorer(sc())
    s.update(1.00, 10.0, 0.0, 13.0, [], 0, 0.04)
    s.update(1.04, 10.5, 0.0, 12.6, [], 0, 0.04)           # -10m/s² -> 감점
    assert s.score == 100 - PENALTY["harsh"], s.violations
    s2 = Scorer(sc())
    s2.update(1.00, 10.0, 0.0, 13.0, [], 0, 0.04)
    s2.update(1.04, 40.0, 0.0, 0.0, [], 0, 0.04)           # 30m 점프 = 리스폰 -> 제외
    assert not s2.violations


def test_목표_도달_반경은_주최측_15m():
    s = Scorer(sc())
    s.update(1.0, 500.0 - GOAL_R - 1.0, 0.0, 5.0, [], 0, 0.04)
    assert not s.reached
    s.update(2.0, 500.0 - GOAL_R + 1.0, 0.0, 5.0, [], 0, 0.04)
    assert s.reached
    s.report(60.0)
    assert all("미도달" not in v for v in s.violations)


def test_미도달이면_감점():
    s = Scorer(sc())
    s.update(1.0, 100.0, 0.0, 5.0, [], 0, 0.04)
    s.report(60.0)
    assert s.score == 100 - PENALTY["goal_miss"], s.violations


def test_같은_위반은_한_번만():
    lt = TrafficLight(1, 100.0, [["RED", 60]])
    s = Scorer(sc(lights=[lt]))
    s.update(1.0, 99.0, 0.0, 5.0, [], 1, 0.04)
    s.update(1.1, 101.0, 0.0, 5.0, [], 1, 0.04)
    s.update(1.2, 99.5, 0.0, 5.0, [], 1, 0.04)             # 뒤로 갔다
    s.update(1.3, 101.5, 0.0, 5.0, [], 1, 0.04)            # 다시 넘음 -> key 같음
    assert s.score == 100 - PENALTY["red_run"], s.violations


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
