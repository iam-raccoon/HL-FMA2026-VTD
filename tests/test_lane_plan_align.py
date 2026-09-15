"""차로계획을 **우리가 달리는 경로**에 거리로 맞춰 까는가 (2026-09-11 코스 H).

차로계획은 원래 경로(ego_route) 점마다 만든다. 우리는 그걸 조밀화한 경로(`Scenario.route_points`,
2m 넘는 구간에 점을 끼운다)를 달린다. 코스 H 는 2015점 -> 2021점이라 880번부터 번호가 어긋나
경로 끝에서 ≈8m **앞**의 차로계획을 읽었고(지정차로·차선변경 지시등·교차로·종점 붙기), 마지막
6점은 차로계획이 아예 없었다. 사용자: "차선 인식 똑바로 하게".
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from drive import DrivingStack                                          # noqa: E402
from scenario import Scenario                                           # noqa: E402

R = os.path.join(HERE, "..", "routes")


def _load(course):
    sc = Scenario.load(os.path.join(R, f"HL_FMA_NEW_{course}.json"))
    lp = json.load(open(os.path.join(R, f"HL_FMA_NEW_{course}_lane.json"), encoding="utf-8"))["pts"]
    return sc, lp


def test_코스_H_는_경로점마다_같은_자리의_차로계획을_읽는다():
    sc, lp = _load("H")
    st = DrivingStack(scenario=sc, lane_plan=lp)
    assert len(st.lane_plan) == len(st.route) == 2021
    raw = sc.ego_route
    for i in range(0, len(st.route), 7):
        x, y = st.route[i]
        j = min(range(len(raw)), key=lambda k: (raw[k][0] - x) ** 2 + (raw[k][1] - y) ** 2)
        # 끼운 점은 구간 시작점 값을 쓰니 원래 점 하나(≤ 한 구간) 차이까지만 허용
        k = next((m for m, q in enumerate(lp) if q is st.lane_plan[i]), None)   # 같은 내용이 여럿 — 객체로 찾는다
        assert k is not None and abs(k - j) <= 1, (i, j, k)
    # 예전(번호 그대로) 이면 경로 끝에서 6점 어긋났다
    assert st.lane_plan[-1] is lp[-1]


def test_점_개수가_같은_코스는_그대로_둔다():
    for c in "ABDEG":
        sc, lp = _load(c)
        st = DrivingStack(scenario=sc, lane_plan=lp)
        assert st.lane_plan is lp, c
