#!/usr/bin/env python3
"""주행 CSV(--log-csv)에서 신호등 정지선 위치를 역산해 경로 JSON에 tl_stops로 병합.

원리(2026-08-14 실측): 9910 DataPacket은 '다음 관련 신호' 1개만 tl_id/tl_state로 보고하고,
그 신호를 통과하는 순간 tl_id가 0(또는 다음 신호)으로 바뀐다.
따라서 **tl_id가 바뀌기 직전 위치 ≈ 그 신호의 정지선(통과 지점)**.
behavior 쪽에서 stop_margin(기본 4m)만큼 앞에서 멈추므로 약간의 추정오차는 안전측으로 흡수된다.

사용: python3 map_lights.py <run.csv> <route.json> [out.json]
"""
import csv as _csv
import json
import math
import sys


def extract(csv_path):
    rows = []
    with open(csv_path, encoding="utf-8") as f:
        for r in _csv.DictReader(f):
            rows.append((float(r["x"]), float(r["y"]), int(r["tl_id"]), int(r["tl_state"])))
    stops, prev = {}, None
    for x, y, tid, st in rows:
        if prev is not None and tid != prev[2] and prev[2] != 0:
            stops[prev[2]] = [round(prev[0], 2), round(prev[1], 2)]     # 바뀌기 직전 = 통과 지점
        prev = (x, y, tid, st)
    return stops, rows


def main():
    csv_path, route_path = sys.argv[1], sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else route_path
    stops, rows = extract(csv_path)
    d = json.load(open(route_path, encoding="utf-8"))
    old = d.get("tl_stops", {})
    old.update({str(k): v for k, v in stops.items()})
    d["tl_stops"] = old
    json.dump(d, open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"CSV {len(rows)}프레임 -> 신호 {len(stops)}개 매핑 -> {out}")
    for k, v in sorted(stops.items()):
        print(f"   tl {k:>4} 정지선 ≈ ({v[0]:8.1f}, {v[1]:9.1f})")


if __name__ == "__main__":
    main()
