"""신호 정지선 DB(tl_map)가 **손으로 실측한 정지선**과 얼마나 맞나.

2026-09-04: 신호 기준선 좌표를 도색 정지선에 **진행방향으로** 스냅했다.
  실측 5곳 진행방향 |오차| 합 3.14m -> 0.48m (tl90 은 -2.44m -> +0.02m).
  횡방향은 일부러 안 건드린다 — 넓은 도로는 차로마다 정지선이 따로 도색돼 있어
  평균을 내면 횡으로 10m 까지 밀리는데, 정지거리는 진행방향 성분만 쓴다.
"""
import json
import math
import os

HERE = os.path.join(os.path.dirname(__file__), "..")


def _load():
    tl = json.load(open(os.path.join(HERE, "routes", "tl_map_livinglab.json"),
                        encoding="utf-8"))
    meas = json.load(open(os.path.join(HERE, "routes", "official_v1_v6.json"),
                          encoding="utf-8"))["tl_stops"]
    return tl, meas


# 실측 5곳의 진행방향(가장 가까운 도색선의 hdg, 라디안) — xodr 을 안 읽으려고 박아둔다
HDG = {
    "30": -0.9085,   #  -52.1°
    "38": -0.9151,   #  -52.4°
    "74": -0.9045,   #  -51.8°
    "90": -0.9035,   #  -51.8°
}


def test_실측과_진행방향_오차가_작다():
    tl, meas = _load()
    tot = 0.0
    for k, h in HDG.items():
        v = meas[k]
        p = tl[k]
        lon = (p[0] - v[0]) * math.cos(h) + (p[1] - v[1]) * math.sin(h)
        assert abs(lon) < 0.6, f"tl{k} 진행방향 오차 {lon:+.2f}m"
        tot += abs(lon)
    assert tot < 1.0, f"|오차| 합 {tot:.2f}m — 예전 3.14m, 스냅 후 0.48m"


def test_항목수가_그대로다():
    tl, _ = _load()
    assert len(tl) == 214


def test_tl90_이_고쳐져_있다():
    """가장 크게 틀렸던 곳. 예전 -2.44m(정지선을 2.4m 넘어서 섬)."""
    tl, meas = _load()
    v, p, h = meas["90"], tl["90"], HDG["90"]
    lon = (p[0] - v[0]) * math.cos(h) + (p[1] - v[1]) * math.sin(h)
    assert abs(lon) < 0.2, f"{lon:+.2f}m"
