#!/usr/bin/env python3
"""시나리오에 **마주오는 차** 하나를 심는다 — 비보호 좌회전 양보를 VTD 로 검증하려고.

왜 필요한가(2026-08-20): 화살표 없는 교차로에서 좌회전할 때 대향 직진차를 보내주는
로직을 넣었는데, **우리 맵의 그 교차로(tl 108)에는 대향차가 안 온다.** 오프라인
판으로는 '충돌 -> 완주'까지 확인했지만 실주행으로는 확인할 판이 없다. 그래서 만든다.

    python3 vtd/add_oncoming.py <in.xml> <out.xml> X Y HEADING_DEG [속도] [트리거거리]

  · `PosAbsolute` 로 놓는다 — `PathRef` 는 스폰이 안 되는 경우가 있었다(EV_CUTIN 1차 실패).
  · 처음엔 **정지**해 있다가, ego 가 트리거거리 안으로 들어오면 그 속도로 달린다.
    ego 가 신호를 기다리는 동안 지나가 버리면 시험이 안 되므로, 트리거를 짧게 잡아
    **ego 가 교차로에 도착한 뒤에** 출발시키는 게 요령이다.
"""
import math
import sys
import xml.etree.ElementTree as ET

CAR = "AlfaRomeo_Brera_10_BiancoSpino"


def add(in_xml, out_xml, x, y, hdg_deg, speed=8.0, trig=60.0, name="Oncoming",
        align=True):
    t = ET.parse(in_xml)
    root = t.getroot()
    ego = next((p for p in root.iter("Player")
                if p.find("Description") is not None
                and p.find("Description").get("Name") == "Ego"), None)
    if ego is None:
        raise SystemExit(f"{in_xml} 에 Ego 가 없다")
    parent = next(el for el in root.iter() if ego in list(el))

    p = ET.Element("Player")
    ET.SubElement(p, "Description", {
        "Driver": "DefaultDriver", "Control": "internal",
        "AdaptDriverToVehicleType": "true", "Type": CAR, "Name": name})
    ini = ET.SubElement(p, "Init")
    ET.SubElement(ini, "Speed", {"Value": "0.0000000000000000e+00"})
    ET.SubElement(ini, "PosAbsolute", {
        "X": f"{x:.16e}", "Y": f"{y:.16e}", "Z": "0.0000000000000000e+00",
        "Direction": f"{math.radians(hdg_deg) % (2 * math.pi):.16e}",
        "AlignToRoad": "true" if align else "false"})
    parent.insert(list(parent).index(ego) + 1, p)

    pa = ET.Element("PlayerActions", {"Player": name})
    act = ET.SubElement(pa, "Action", {"Name": ""})
    ET.SubElement(act, "PosRelative", {
        "NetDist": "false", "Distance": f"{trig:.16e}", "Pivot": "Ego"})
    ET.SubElement(act, "SpeedChange", {
        "Rate": "3.0000000000000000e+00", "Target": f"{speed:.16e}", "Force": "true",
        "ExecutionTimes": "1", "ActiveOnEnter": "true",
        "DelayTime": "0.0000000000000000e+00"})
    parent.insert(list(parent).index(p) + 1, pa)

    with open(out_xml, "w", encoding="utf-8") as f:
        f.write("<?xml version='1.0' encoding='utf-8'?>\n")
        f.write(ET.tostring(root, encoding="unicode"))
        f.write("\n")
    print(f"{out_xml}: {name} @({x:.1f},{y:.1f}) {hdg_deg:.0f}° "
          f"{speed}m/s, ego 가 {trig:.0f}m 안으로 오면 출발")


if __name__ == "__main__":
    a = sys.argv[1:]
    if len(a) < 5:
        raise SystemExit(__doc__)
    add(a[0], a[1], float(a[2]), float(a[3]), float(a[4]),
        float(a[5]) if len(a) > 5 else 8.0,
        float(a[6]) if len(a) > 6 else 60.0)
