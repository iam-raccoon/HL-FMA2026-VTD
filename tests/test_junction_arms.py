"""'교차로' 의 정의를 채점기와 주행기가 **같이** 쓰는지.

갈래 2개짜리 junction 은 교차로가 아니라 모퉁이다. 주행기는 `jx`(build_lane_plan 이
`junc_arms >= 3` 으로 심는다)로 이미 그렇게 보는데, `eval/check_route.py` 의 ⑨
노면 화살표 검사만 안 보고 있었다 — 그래서 사전테스트1 에서 **없는 위반**이 나왔다.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "eval"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "vtd"))
from check_route import is_real_junction                            # noqa: E402


class _Map:
    def __init__(self, arms):
        self.junc_arms = arms


def test_갈래_2개는_모퉁이다():
    assert is_real_junction(_Map({"91": 2}), "91") is False


def test_갈래_3개부터_교차로다():
    mp = _Map({"a": 3, "b": 4, "c": 5})
    assert all(is_real_junction(mp, k) for k in ("a", "b", "c"))


def test_모르는_교차로는_교차로가_아니다():
    """`junc_arms` 에 없으면 갈래를 셀 수 없다 — 없는 위반을 만들지 않는 쪽으로."""
    assert is_real_junction(_Map({}), "없는id") is False


def test_주행기와_같은_문턱을_쓴다():
    """★`build_lane_plan` 의 jx 문턱이 바뀌면 여기도 같이 바뀌어야 한다."""
    src = open(os.path.join(os.path.dirname(__file__), "..", "vtd",
                            "build_lane_plan.py"), encoding="utf-8").read()
    assert 'mp.junc_arms.get(rd.get("junction", "-1"), 0) >= 3' in src, \
        "주행기의 jx 문턱이 바뀌었다 — check_route.is_real_junction 도 맞출 것"
