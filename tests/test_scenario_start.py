"""시나리오의 ego 출발점이 **경로 CSV 1번 지점**인가.

왜 필요한가(실측 2026-09-08): 사전테스트2 를 돌리려 했더니 `drive.sh` 가 막았다 —
  ego (324.3,-56.7)  경로 시작 (239.7,145.9)  차이 219.6m
  ❌ **시나리오와 경로가 다른 코스다**(220m 차이)

원인은 `vtd/make_course.py` 의 경유지 합치기였다. 같은 도로가 연달아 나오면 하나로 합치면서
**뒤 경유지의 s 로 덮어쓰는데**, 그게 첫 도로면 ego 출발점이 통째로 바뀐다.
사전테스트2 는 경유지 1·2 가 둘 다 road 429 로 잡혀(s=216.4, s=5.0) 출발점이 2번 지점으로
밀렸다. 첫 도로만 덮어쓰지 않도록 고쳤다.

⚠️ 이 테스트는 지도를 안 읽는다(느리다). 대신 **고친 뒤의 값을 그대로 박아** 둔다 —
   시나리오를 다시 만들었을 때 출발점이 또 밀리면 여기서 걸린다.
   좌표 확인은 해 뒀다: road 429 s=216.4 ≈ (248,144) = CSV 1번 (239.8,146.0)
                       road 429 s=5.0   ≈ (322,-54) = CSV 2번 (325.1,-58.3)  ← 옛 값
"""
import os
import re

HERE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

# 시나리오 -> (첫 Waypoint TrackId, s, 허용오차)
START = {
    "HL_FMA_PRETEST_1": ("2817", 166.8, 1.0),
    "HL_FMA_PRETEST_2": ("429", 216.4, 1.0),
}


def _first_waypoint(name):
    p = os.path.join(HERE, "scenarios", f"{name}.xml")
    s = open(p, encoding="utf-8", errors="ignore").read()
    m = re.search(r"<Waypoint[^>]*/>", s)
    assert m, f"{name}: Waypoint 가 없다"
    tid = re.search(r'TrackId="(\d+)"', m.group(0))
    sv = re.search(r's="([^"]+)"', m.group(0))
    assert tid and sv, f"{name}: Waypoint 에 TrackId/s 가 없다"
    return tid.group(1), float(sv.group(1))


def test_사전테스트_시나리오는_1번_경유지에서_출발한다():
    for name, (tid, s0, tol) in START.items():
        tid_got, s_got = _first_waypoint(name)
        assert tid_got == tid, f"{name}: 출발 도로가 {tid} 가 아니라 {tid_got}"
        assert abs(s_got - s0) <= tol, f"{name}: 출발 s 가 {s0} 가 아니라 {s_got:.1f}"


def test_ego_는_그_경로를_참조한다():
    """PathRef 가 없으면 ego 가 월드 원점(0,0)에 떨어진다(make_course 주석)."""
    for name in START:
        p = os.path.join(HERE, "scenarios", f"{name}.xml")
        s = open(p, encoding="utf-8", errors="ignore").read()
        ref = re.search(r"<PathRef[^>]*/>", s)
        assert ref, f"{name}: PathRef 가 없다"
        assert 'PathId="1"' in ref.group(0), ref.group(0)
