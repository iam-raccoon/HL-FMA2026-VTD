#!/usr/bin/env python3
"""코스 시나리오에 **주변 교통(Pulk traffic)** 을 넣는다 -> `scenarios/<코스>_TR.xml`.

사용자 요청 2026-08-24: "그냥 차들이 반 도로처럼 많이 다니는 걸 원했다".
`add_hazards.py` 는 **각본 있는 액터 몇 대**를 심는 것이라 목적이 다르다. 도로에 차가
북적이게 하려면 VTD 의 교통 생성기를 써야 한다.

찾은 방법(2026-08-24, VTD 2025.2 샘플에서): `<Scenario>` 바로 밑의

    <PulkTraffic><PulkDef ... CentralPlayer="Ego" Count="50" .../></PulkTraffic>

"Pulk"은 독일어로 무리다. **ego 를 중심으로 한 타원 영역**에 차를 계속 채워 넣고,
멀어지면 지우고 앞쪽에 다시 만든다. 그래서 코스 어디를 달려도 교통량이 유지된다.
주최측 공식 시나리오는 `<TrafficElements/>` 가 비어 있고 PulkTraffic 도 없다 —
즉 **공식 판에는 주변 교통이 아예 없다.** 이건 우리 연습용이다.

기본값은 VTD 의 `Japan_Town_2019` 샘플에서 가져왔다(도심 맵이라 우리 LivingLab 과
성격이 같다). 고속도로 샘플(TrafficHighway)은 Count=100·Cars=0.8 로 더 빽빽하다.

    python3 vtd/add_traffic.py <코스이름> [대수] [fill]
      예: python3 vtd/add_traffic.py HL_FMA_NEW_A 50
      세 번째 인자 `fill` 을 주면 출발 때부터 가득 채운다(아래 ⚠️ 참고, 권장 안 함)

⚠️ 대수를 올리면 **VTD 프레임율이 떨어진다.** 프레임이 무너진 판은 수치를 못 믿으니
   (그래서 check_run 이 최악 구간 Hz 를 찍는다) 40~60 에서 시작하는 게 좋다.
⚠️ 9910 객체 배열은 **30칸**이다. 지금까지 최대 4칸만 썼는데, 교통을 넣으면 처음으로
   가득 찰 수 있다. 80m 안에서 가까운 순으로 잘리므로 먼 차가 먼저 빠진다.
"""
import sys
import xml.etree.ElementTree as ET

HERE = __file__.rsplit("/", 1)[0]

# Japan_Town_2019 샘플 값. 자전거 20%는 도심 기준이라 그대로 둔다 —
# 우리 로직은 높이 1.2m 이상을 사람으로 보므로 자전거가 어떻게 잡히는지도 볼 만하다.
# ⚠️ **FillAtStart 를 켜면 안 된다.** 실측 2026-08-24 첫 `_TR` 판:
#    t=0 에 자차 주위가 이미 차로 가득 차서(obj=7) **2.4m 가다 옆차와 접촉하고 물렸다**
#    (853.6,-12.4 에서 NARROW_BLOCK 고착). 코스 A 출발점은 교차로 횡단보도 위라
#    비킬 자리도 없다. false 로 두면 **달리면서 앞쪽부터 채워진다** — 마주치는 교통량은
#    같고 출발만 깨끗하다.
DEF = {
    "FillAtStart": "false", "CentralPlayer": "Ego", "VisibleInArea": "-1",
    "SemiMajorAxis": "4.0e+02", "SemiMinorAxis": "4.0e+02",
    "InnerRadius": "2.0e+02", "CenterOffset": "1.0e+02",
    "Cars": "6.2e-01", "Vans": "1.1e-01", "Trucks": "5.0e-02",
    "Buses": "2.0e-02", "Bikes": "2.0e-01",
    "OwnSide": "6.0e-01",
    "AreaF": "3.9e-01", "AreaB": "3.1e-01", "AreaL": "1.5e-01", "AreaR": "1.5e-01",
}


def main():
    name = sys.argv[1]
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 50
    fill = "true" if (len(sys.argv) > 3 and sys.argv[3] == "fill") else DEF["FillAtStart"]
    src = f"{HERE}/../scenarios/{name}.xml"
    out = f"{HERE}/../scenarios/{name}_TR.xml"
    t = ET.parse(src)
    root = t.getroot()
    for old in root.findall("PulkTraffic"):        # 두 번 돌려도 안 겹치게
        root.remove(old)
    pt = ET.SubElement(root, "PulkTraffic")
    ET.SubElement(pt, "PulkDef", dict(DEF, Count=str(count), FillAtStart=fill))
    with open(out, "w", encoding="utf-8") as f:
        f.write("<?xml version='1.0' encoding='utf-8'?>\n")
        f.write(ET.tostring(root, encoding="unicode"))
        f.write("\n")
    print(f"-> {out}  (ego 중심 400m 타원에 {count}대 · FillAtStart={fill}: "
          f"승용 62% 밴 11% 트럭 5% 버스 2% 자전거 20%)")


if __name__ == "__main__":
    main()
