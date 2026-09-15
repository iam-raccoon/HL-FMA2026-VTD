#!/usr/bin/env python3
"""경로 각 지점에서 **같은 방향 주행차선 안에서 좌/우로 얼마나 비킬 수 있는지**(lane room)를
xodr에서 미리 계산해 JSON으로 뽑는다.

왜 필요한가(2026-08-15 실측): 추월로 옆차선에 나갔다가 **리스폰**이 걸렸다
(1.59m 이동 + 헤딩 20° 스냅). 리스폰은 건당 감점이다. 9910 패킷에는 차선 정보가
전혀 없으므로, '왼쪽이 같은 방향 차선인지 반대차선(중앙선 너머)인지'는
지도에서 미리 알아두는 수밖에 없다. 신호 정지선 DB(build_tl_map.py)와 같은 접근.

부호 약속: 결과의 left/right 는 **주행 진행방향 기준**. xodr 의 t 부호가 아니다
(좌측 차선군은 도로 s 와 반대로 달리므로 부호가 뒤집힌다 — 이거 놓치면 정반대가 된다).

사용: python3 build_lane_room.py <xodr> <route.json> [out.json]
"""
import json
import math
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from build_tl_map import road_point                      # noqa: E402

DRIVABLE = {"driving"}


def poly(el, ds):
    return (float(el.get("a", 0)) + float(el.get("b", 0)) * ds
            + float(el.get("c", 0)) * ds * ds + float(el.get("d", 0)) * ds ** 3)


def lane_edges(rd, s):
    """s 지점 laneSection 의 차선 목록 [(id, type, t_lo, t_hi)] (t 기준)."""
    secs = rd.findall("lanes/laneSection")
    cand = [x for x in secs if float(x.get("s", 0)) <= s + 1e-6]
    if not cand:
        return []
    sec = max(cand, key=lambda x: float(x.get("s", 0)))
    ds = s - float(sec.get("s", 0))
    off = 0.0
    for lo in rd.findall("lanes/laneOffset"):
        if float(lo.get("s", 0)) <= s:
            off = poly(lo, s - float(lo.get("s", 0)))
    out = []
    for sign, side in ((+1, "left"), (-1, "right")):
        edge = off
        lanes = sorted(sec.findall(f"{side}/lane"), key=lambda l: abs(int(l.get("id"))))
        for ln in lanes:
            w = poly(ln.find("width"), ds) if ln.find("width") is not None else 0.0
            a, b = edge, edge + sign * w
            out.append((int(ln.get("id")), ln.get("type"), min(a, b), max(a, b)))
            edge = b
    return out


def build(xodr, route, step=4.0):
    roads = ET.parse(xodr).getroot().findall("road")
    # 도로 reference line 을 미리 샘플링(경로점마다 전 도로를 훑으면 너무 느리다)
    samples = []
    for rd in roads:
        L = float(rd.get("length"))
        s = 0.0
        while s <= L:
            try:
                x, y, h = road_point(rd, s)
            except Exception:
                break
            samples.append((x, y, h, rd, s))
            s += 2.0

    out = []
    for i, (px, py) in enumerate(route):
        # 진행방향
        j = min(i + 1, len(route) - 1)
        k = max(i - 1, 0)
        hdg = math.atan2(route[j][1] - route[k][1], route[j][0] - route[k][0])

        best = min(samples, key=lambda q: (q[0] - px) ** 2 + (q[1] - py) ** 2)
        x, y, h, rd, s = best

        # ⚠️ 교차로(junction) 안은 연결도로가 여러 개 겹쳐 있어 '가장 가까운 기준선'이
        #    엉뚱한 도로를 고른다(실측: 경로점이 그 도로의 유일한 차선 밖 t=+1.6 에 찍힘).
        #    잘못된 값을 주느니 **모른다(null)** 로 두고 호출측이 기존 동작을 쓰게 한다.
        if rd.get("junction", "-1") != "-1":
            out.append(None)
            continue

        t = (px - x) * -math.sin(h) + (py - y) * math.cos(h)
        # 주행방향이 도로 s 와 반대면 t 부호가 뒤집힌다
        flip = -1.0 if math.cos(hdg - h) < 0 else 1.0

        lanes = lane_edges(rd, s)
        mine = next((l for l in lanes if l[2] - 1e-6 <= t <= l[3] + 1e-6), None)
        if mine is None or mine[1] not in DRIVABLE:
            out.append(None)                          # 모르면 모른다고 한다
            continue
        # 내 차선과 '같은 부호(같은 방향)'인 주행차선들만 이어붙인다
        same = [l for l in lanes if l[1] in DRIVABLE and (l[0] > 0) == (mine[0] > 0)]
        lo = min(l[2] for l in same)
        hi = max(l[3] for l in same)
        # 주행방향 기준 좌/우 여유 (t 기준 거리 -> flip 으로 방향 환산)
        room_t_plus, room_t_minus = hi - t, t - lo
        left, right = ((room_t_plus, room_t_minus) if flip > 0
                       else (room_t_minus, room_t_plus))
        out.append([round(left, 2), round(right, 2)])
    return out


def main():
    xodr, route_path = sys.argv[1], sys.argv[2]
    out_path = sys.argv[3] if len(sys.argv) > 3 else "lane_room.json"
    route = json.load(open(route_path, encoding="utf-8"))["ego_route"]
    room = build(xodr, route)
    json.dump({"room": room}, open(out_path, "w", encoding="utf-8"))
    known = [r for r in room if r]
    unk = len(room) - len(known)
    L = sorted(r[0] for r in known); R = sorted(r[1] for r in known)
    print(f"경로 {len(room)}점 (모름 {unk}점 = 교차로/차선밖)")
    print(f"  좌여유 중앙값 {L[len(L)//2]:.2f}m  3.5m이상 {sum(1 for v in L if v>=3.5)*100//len(L)}%")
    print(f"  우여유 중앙값 {R[len(R)//2]:.2f}m  3.5m이상 {sum(1 for v in R if v>=3.5)*100//len(R)}%")
    print(f"  -> {out_path}")


if __name__ == "__main__":
    main()
