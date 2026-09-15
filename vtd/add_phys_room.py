#!/usr/bin/env python3
"""이미 있는 차로계획(`routes/<이름>_lane.json`)에 **물리 폭 pl/pr** 만 덧붙인다.

`build_lane_plan.py` 는 이제 pl/pr 을 같이 낸다(phys_room 주석). 그런데 코스 계획을
통째로 다시 돌리면 다른 값(need·sig 등)까지 지금 코드로 다시 계산돼 무엇이 바뀌었는지
가려진다. 대회 사흘 전에는 **키 두 개만 더하는 게** 맞다.

    python3 vtd/add_phys_room.py <xodr> routes/HL_FMA_NEW_A_lane.json [...]

경로는 같은 자리의 `<이름>.json` 에서 읽는다. 점 수가 다르면 그 파일은 건너뛴다.
"""
import json
import math
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from build_lane_plan import Map, phys_room, route_headings          # noqa: E402


def main():
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    mp = Map(sys.argv[1])
    for lane_path in sys.argv[2:]:
        if not lane_path.endswith("_lane.json"):
            print(f"  ⚠️ {lane_path}: _lane.json 이 아니다 — 건너뜀")
            continue
        route_path = lane_path[:-len("_lane.json")] + ".json"
        try:
            route = json.load(open(route_path, encoding="utf-8"))["ego_route"]
        except Exception as e:                                        # noqa: BLE001
            print(f"  ⚠️ {lane_path}: 경로 {route_path} 를 못 읽음({e}) — 건너뜀")
            continue
        lp = json.load(open(lane_path, encoding="utf-8"))
        pts = lp["pts"]
        if len(pts) != len(route):
            print(f"  ⚠️ {lane_path}: 점 수 {len(pts)} != 경로 {len(route)} — 건너뜀")
            continue
        hdgs = route_headings(route)
        n = 0
        for i, p in enumerate(pts):
            if p is None:
                continue
            p["pl"], p["pr"] = phys_room(mp, route[i][0], route[i][1], hdgs[i])
            n += 1
        json.dump(lp, open(lane_path, "w", encoding="utf-8"))
        wide = sum(1 for p in pts if p and (p["pl"] > p["l"] + 0.5 or p["pr"] > p["r"] + 0.5))
        print(f"  {lane_path}: {n}점에 pl/pr 기록 (합법 폭보다 0.5m+ 넓은 점 {wide})")


if __name__ == "__main__":
    main()
