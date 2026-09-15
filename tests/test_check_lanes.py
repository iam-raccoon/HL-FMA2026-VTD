"""공용 OBB 차로 판정 회귀 테스트."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "eval"))

import check_lanes as C  # noqa: E402
from run_logger import EGO_CX, EGO_HALF_L, EGO_HALF_W  # noqa: E402


class FakeRoad(dict):
    pass


class FakeMap:
    """직선 양방향 1차로와 바깥 sidewalk가 있는 최소 지도."""

    def __init__(self, junction="-1", marks="broken"):
        self.road = FakeRoad(id="road", junction=junction)
        self.marks = marks

    def locate(self, x, y):
        return self.road, x, y, 0.0

    def lanes(self, _road, _s):
        return [
            (-1, "driving", -3.2, 0.0, self.marks),
            (-2, "sidewalk", -5.2, -3.2, "solid"),
            (+1, "driving", 0.0, 3.2, self.marks),
            (+2, "shoulder", 3.2, 4.2, "solid"),
            (+3, "sidewalk", 4.2, 6.2, "solid"),
        ]


def classify_at(t, heading=0.0, mp=None):
    row = dict(t=0.0, x=10.0, y=t, h=heading)
    C.classify([row], mp or FakeMap())
    return row


def test_도로와_같은_방향인_직선에서_OBB_횡범위():
    center, radius, lo, hi = C.obb_lateral_interval(-1.6, 0.0, 0.0)
    assert math.isclose(center, -1.6)
    assert math.isclose(radius, EGO_HALF_W)
    assert math.isclose(lo, -1.6 - EGO_HALF_W)
    assert math.isclose(hi, -1.6 + EGO_HALF_W)


def test_차량이_회전하면_전장이_횡투영에_포함된다():
    delta = math.radians(30.0)
    center, radius, _lo, _hi = C.obb_lateral_interval(-1.6, delta, 0.0)
    assert math.isclose(center, -1.6 + EGO_CX * math.sin(delta))
    assert math.isclose(
        radius,
        EGO_HALF_L * abs(math.sin(delta)) + EGO_HALF_W * abs(math.cos(delta)))
    assert radius > EGO_HALF_W


def test_차로경계_침범깊이도_차량_OBB로_계산한다():
    row = classify_at(-1.6, math.radians(30.0))
    expected = max(0.0, -3.2 - row["obb_t_min"], row["obb_t_max"])
    assert math.isclose(row["lane_intrusion"], expected)
    assert row["edge"] == (expected > 0.0)


def test_후륜축은_정상차로지만_앞모서리가_중앙선을_06m_넘으면_검출():
    delta = math.radians(30.0)
    rear_t = 0.6 - (EGO_CX * math.sin(delta)
                    + EGO_HALF_L * abs(math.sin(delta))
                    + EGO_HALF_W * abs(math.cos(delta)))
    row = classify_at(rear_t, delta)
    assert -3.2 < rear_t < 0.0 and row["lane"] == -1
    assert math.isclose(row["center_intrusion"], 0.6, abs_tol=1e-9)
    assert len(C.centerline_violations([
        dict(row, t=t, x=t) for t in (0.0, 0.6)
    ])) == 1


def test_점선_중앙선도_OBB_깊이가_같으면_검출한다():
    rear_t = C.CENTER_M - EGO_HALF_W
    rows = [dict(classify_at(rear_t, mp=FakeMap(marks="broken")), t=t, x=t)
            for t in (0.0, 0.6)]
    assert len(C.centerline_violations(rows)) == 1


def test_기준점은_차도지만_OBB가_보도에_05m_들어가면_검출():
    rear_t = -3.2 + EGO_HALF_W - C.WALK_M
    row = classify_at(rear_t)
    assert row["lane"] == -1 and not row["off"]
    assert math.isclose(row["sidewalk_intrusion"], 0.5, abs_tol=1e-9)


def test_shoulder_겹침은_보도침범으로_세지_않는다():
    row = classify_at(3.2)  # OBB는 shoulder 안까지 가지만 4.2m의 sidewalk에는 못 닿는다
    assert row["sidewalk_intrusion"] == 0.0


def test_교차로_중첩구간은_이유를_남기고_판정보류한다():
    row = classify_at(-1.6, mp=FakeMap(junction="17"))
    assert row["classification_hold_reason"] == "junction_overlap"
    assert row["center_intrusion"] is None
    assert row["sidewalk_intrusion"] is None


def test_점선_중앙선을_넘었다_복귀해도_위반이다():
    rows = []
    for i in range(40):
        t = i * 0.1
        rows.append(dict(t=t, x=t, y=0.0,
                         center_intrusion=0.6 if 1.0 <= t <= 2.0 else 0.0,
                         mark="broken"))
    violations = C.centerline_violations(rows)
    assert len(violations) == 1
    assert violations[0][0] == 1.0 and violations[0][1] == 2.0


def test_중앙선판정에는_앞지르기_시간_면제가_없다():
    src = open(C.__file__, encoding="utf-8").read()
    assert "OVERTAKE_MAX_S" not in src
    assert "def _excused" not in src
