#!/usr/bin/env python3
"""시나리오 사전점검 — **도로망 밖에 놓인 오브젝트**를 찾는다.

왜 필요한가(2026-08-17 실측, v7):
  공식 v7 시나리오는 `Fuelcan01` 3개를 road 2063 의 t=+2.66~+5.00 에 찍어놨는데,
  그 도로는 **음(-)쪽에만 차로가 있다**(차로 범위 [-7.6, +0.0]). 즉 도로망 바깥이다.
  에고가 그 옆을 지나가는 순간 **VTD 출력이 25Hz -> 3.5Hz 로 떨어지고 영구히
  회복되지 않았다**(1028초 내내). 우리 계산은 0.8ms(프레임의 0.3%)였고 VTD PC 는
  32코어에 10% 미만이었다 — 밖에서 보이는 원인은 없고, 물체를 치우면 사라진다.

  · 연료통 1개만 남겨도 붕괴  -> 개수 문제 아님
  · 6m 만 옮겨 안 붙으면 정상 -> 근접이 방아쇠
  · **도로 안으로** 옮기면 정상 완주(125s, 느린프레임 0, 회피 1096프레임 유지)
  · v5 는 같은 연료통을 t=+4.90 에 두지만 road 128 은 +14.4 까지 있어 **차로 위** -> 멀쩡

대회 당일 코스를 받으면 주행 전에 이걸 돌려, 같은 함정이 있는지 몇 초 만에 본다.
고칠 수는 없어도 **알고 들어가는 것과 모르고 당하는 것은 다르다** — 그 구간에서
프레임율이 무너지면 신호를 놓치므로, 미리 알면 최소한 원인 진단에 시간을 안 뺏긴다.

사용: python3 vtd/check_scenario.py <xodr> <scenario.xml> [...]
"""
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from build_lane_plan import Map                              # noqa: E402


def placed(root, tag):
    for e in root.iter(tag):
        # ⚠️ `a or b` 로 쓰면 안 된다 — ElementTree 는 **자식 없는 요소가 falsy** 라
        #    <StartPosAbs .../> 가 False 로 평가돼 통째로 놓친다(실제로 한 번 당했다).
        p = e.find(".//StartPosAbs")
        if p is None:
            p = e.find(".//PosAbsolute")
        if p is not None and p.get("X") and p.get("Y"):
            d = e.find("Description")
            name = (e.get("Definition") or e.get("Name")
                    or (d.get("Definition") if d is not None else None) or tag)
            yield name, float(p.get("X")), float(p.get("Y"))


def main():
    mp = Map(sys.argv[1])
    bad_total = 0
    for path in sys.argv[2:]:
        root = ET.parse(path).getroot()
        print(f"\n=== {path.rsplit('/', 1)[-1]}")
        n = bad = 0
        for tag in ("Object", "Character"):
            for name, x, y in placed(root, tag):
                n += 1
                loc = mp.locate(x, y)
                if loc is None:
                    print(f"  ⚠️ {name:<16} ({x:8.1f},{y:9.1f})  도로를 못 찾음")
                    bad += 1
                    continue
                rd, s, t, _h = loc
                lanes = mp.lanes(rd, s)
                inside = any(l[2] - 1e-6 <= t <= l[3] + 1e-6 for l in lanes)
                if not inside:
                    lo = min((l[2] for l in lanes), default=0.0)
                    hi = max((l[3] for l in lanes), default=0.0)
                    print(f"  ❌ {name:<16} ({x:8.1f},{y:9.1f})  road {rd.get('id'):>5} "
                          f"t={t:+.2f}  차로 범위 [{lo:+.1f},{hi:+.1f}]  ← 도로망 밖")
                    bad += 1
        bad_total += bad
        print(f"  오브젝트 {n}개 중 도로 밖 {bad}개"
              + ("  ← 그 옆을 지나가면 VTD 프레임율이 무너질 수 있다" if bad else "  ✅"))
    return 1 if bad_total else 0


if __name__ == "__main__":
    sys.exit(main())
