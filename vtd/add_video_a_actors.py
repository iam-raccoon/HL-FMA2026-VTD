#!/usr/bin/env python3
"""A조 중계 영상의 액터를 고정 배치한 VTD 2025.2 시나리오를 만든다.

영상에서 확인한 순서를 난수 PulkTraffic 없이 재현한다.

  * 초반 교차 차량 3대와 선행 차량 1대
  * 중반 교차 차량 2대
  * 후반 대향 승용차 1대와 버스 1대
  * 터널 뒤 손수레 3개(우-좌-우)
  * 스쿨존 횡단 보행자 3명과 후반 횡단 보행자 1명
  * 종점 직전 라바콘 2개

사용:
    python3 vtd/add_video_a_actors.py
"""
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MAP_FILE = os.path.join(ROOT, "map", "HL_FMA_VTD_LivingLab.xodr")
ROUTE_FILE = os.path.join(ROOT, "routes", "HL_FMA_2026_VIDEO_A.json")
BASE_FILE = os.path.join(ROOT, "scenarios", "HL_FMA_2026_VIDEO_A.xml")
OUT_FILE = os.path.join(ROOT, "scenarios", "HL_FMA_2026_VIDEO_A_EXACT.xml")

sys.path.insert(0, HERE)
from add_crosswalk_peds import character_block, crossing_path  # noqa: E402
from add_hazards import elevation                              # noqa: E402
from build_lane_plan import Map                                # noqa: E402

CAR = "AlfaRomeo_Brera_10_BiancoSpino"
BUS = "MercedesTravego_10_HoneyYellow"
F = "{:.16e}".format
OBJ_Z_LIFT = 0.74


def route_data():
    route = json.load(open(ROUTE_FILE, encoding="utf-8"))["ego_route"]
    cum = [0.0]
    for a, b in zip(route, route[1:]):
        cum.append(cum[-1] + math.dist(a, b))
    return route, cum


def at_s(route, cum, target, lateral=0.0):
    """경로 거리와 횡방향 오프셋의 절대 좌표. lateral>0은 진행방향 왼쪽."""
    i = min(range(len(cum)), key=lambda k: abs(cum[k] - target))
    j = min(i + 3, len(route) - 1)
    h = math.atan2(route[j][1] - route[i][1], route[j][0] - route[i][0])
    x = route[i][0] - math.sin(h) * lateral
    y = route[i][1] + math.cos(h) * lateral
    return x, y, h


def vehicle(name, vtype, x, y, heading, speed, trigger, delay=0.0, align=True):
    p = ET.Element("Player")
    ET.SubElement(p, "Description", {
        "Driver": "DefaultDriver", "Control": "internal",
        "AdaptDriverToVehicleType": "true", "Type": vtype, "Name": name,
    })
    init = ET.SubElement(p, "Init")
    ET.SubElement(init, "Speed", {"Value": F(0.0)})
    ET.SubElement(init, "PosAbsolute", {
        "X": F(x), "Y": F(y), "Z": F(0.0),
        "Direction": F(heading % (2 * math.pi)),
        "AlignToRoad": "true" if align else "false",
    })

    actions = ET.Element("PlayerActions", {"Player": name})
    action = ET.SubElement(actions, "Action", {"Name": ""})
    ET.SubElement(action, "PosRelative", {
        "CounterID": "", "CounterComp": "COMP_EQ", "NetDist": "false",
        "Distance": F(trigger), "CounterVal": "0", "Pivot": "Ego",
    })
    ET.SubElement(action, "SpeedChange", {
        "Rate": F(3.0), "Target": F(speed), "Force": "true",
        "ExecutionTimes": "1", "ActiveOnEnter": "true", "DelayTime": F(delay),
    })
    return p, actions


def static_object(mp, name, definition, x, y, heading):
    loc = mp.locate(x, y)
    if loc is None:
        raise SystemExit(f"{name}: ({x:.1f},{y:.1f})가 도로망 밖")
    road, s, _t, _h = loc
    z = elevation(road, s) + OBJ_Z_LIFT
    obj = ET.Element("Object", {"Type": "other", "Name": name, "Definition": definition})
    ET.SubElement(obj, "StartPosAbs", {
        "X": F(x), "Y": F(y), "Z": F(z), "Direction": F(heading % (2 * math.pi)),
        "Pitch": F(0.0), "Roll": F(0.0),
    })
    return obj, ET.Element("ObjectActions", {"Object": name})


def shifted_path(p0, p1, offset):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dy)
    nx, ny = -dy / length, dx / length
    return ((p0[0] + nx * offset, p0[1] + ny * offset),
            (p1[0] + nx * offset, p1[1] + ny * offset))


def inset_path(p0, p1, amount=1.05):
    """인도 바깥 대기점을 차도 경계 안으로 넣어 VTD 도로망 이탈을 막는다."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dy)
    ux, uy = dx / length, dy / length
    return ((p0[0] + ux * amount, p0[1] + uy * amount),
            (p1[0] - ux * amount, p1[1] - uy * amount))


def pedestrian_blocks(mp):
    cws = json.load(open(os.path.join(ROOT, "routes", "crosswalks.json"),
                         encoding="utf-8"))["crosswalks"]

    def cross(road_id, road_s, start_right=True):
        cw = min((q for q in cws if q["road"] == str(road_id)),
                 key=lambda q: abs(float(q["s"]) - road_s))
        got = crossing_path(mp, cw, start_right)
        if got is None:
            raise SystemExit(f"road {road_id} s={road_s}: 횡단 경로 생성 실패")
        return got

    # 영상 2:45 부근: 같은 스쿨존 횡단보도를 세 명이 오른쪽에서 왼쪽으로 건넌다.
    p0, p1, _yaw, z, _width = cross(2263, 4.3, True)
    p0, p1 = inset_path(p0, p1)
    people = []
    specs = [
        (1, "SchoolPed1", "male_adult", "Christian", -0.5, 52.0, 1.18),
        (2, "SchoolPed2", "female_adult", "Hannah", 0.0, 48.0, 1.32),
        (3, "SchoolPed3", "male_adult", "Christian1", 1.5, 44.0, 1.46),
    ]
    for shape, name, ctype, appearance, offset, trigger, speed in specs:
        q0, q1 = shifted_path(p0, p1, offset)
        people.append(character_block(shape, name, ctype, appearance,
                                      [q0, q1], z, trigger, speed, move="walk"))

    # 영상 3:30 부근: 다음 횡단보도에서 한 명이 같은 방향으로 건넌다.
    p0, p1, _yaw, z, _width = cross(2190, 3.85, True)
    p0, p1 = inset_path(p0, p1)
    people.append(character_block(4, "FinalPed", "male_adult", "Christian2",
                                  [p0, p1], z, 45.0, 1.35, move="walk"))
    return "".join(people)


def add_players(traffic, route, cum):
    actors = []

    # 초반 넓은 교차로: 왼쪽에서 오른쪽으로 연속 통과하는 세 대.
    cx, cy, h = at_s(route, cum, 136.0)
    cross_h = h - math.pi / 2
    left_x, left_y = -math.sin(h), math.cos(h)
    for k, back in enumerate((18.0, 29.0, 40.0), 1):
        actors.append(vehicle(f"StartCross{k}", CAR,
                              cx + left_x * back, cy + left_y * back,
                              cross_h, 7.2, 92.0, delay=0.35 * (k - 1), align=False))

    # 초반부터 같은 방향으로 보이던 선행차.
    x, y, h = at_s(route, cum, 190.0)
    actors.append(vehicle("VideoLead", CAR, x, y, h, 7.0, 105.0))

    # 첫 구간 후반 교차로의 횡단 차량 두 대(지도상 연결 차로 중심 실측 좌표).
    for k, (x, y, hdg, delay) in enumerate((
            (1078.3208078988134, 27.866757181294894, math.radians(219.2), 0.0),
            (1083.7391303185198, 32.29866397164559, math.radians(219.2), 0.0)), 1):
        actors.append(vehicle(f"MidCross{k}", CAR, x, y, hdg, 7.5, 62.0,
                              delay=delay, align=True))

    # 영상 2:07~2:14: 언덕 정상에서 오는 승용차와 큰 승합/버스.
    for name, vtype, s, speed, trigger in (
            ("HillCar", CAR, 1450.0, 7.0, 95.0),
            ("HillBus", BUS, 1510.0, 6.0, 135.0)):
        x, y, h = at_s(route, cum, s, lateral=-5.8)
        actors.append(vehicle(name, vtype, x, y, h + math.pi, speed, trigger))

    path = traffic.find("Path")
    insert_at = list(traffic).index(path) if path is not None else len(traffic)
    for player, actions in actors:
        traffic.insert(insert_at, player)
        insert_at += 1
        traffic.insert(insert_at, actions)
        insert_at += 1
    return len(actors)


def add_objects(root, mp, route, cum):
    moving = root.find("MovingObjectsControl")
    if moving is None:
        raise SystemExit("MovingObjectsControl이 없다")
    moving.clear()

    # 계기거리 840~950m: 오른쪽, 왼쪽, 오른쪽 차로 순서의 손수레 세 개.
    for k, (s, lateral) in enumerate(((850.0, -3.5), (890.0, 0.0), (950.0, -3.5)), 1):
        x, y, h = at_s(route, cum, s, lateral)
        obj, actions = static_object(mp, f"WheelBarrow{k}", "WheelBarrow01",
                                     x, y, h + math.pi / 2)
        moving.extend((obj, actions))

    # 마지막 횡단보도 뒤, 영상처럼 중앙선 쪽과 우측 가장자리에 하나씩 둔다.
    for name, lateral in (("FinalCone1", 1.2), ("FinalCone2", -4.3)):
        x, y, h = at_s(route, cum, 2138.0, lateral)
        obj, actions = static_object(mp, name, "RdMiscPylon03-32cm", x, y, h)
        moving.extend((obj, actions))

    ped_root = ET.fromstring(f"<blocks>{pedestrian_blocks(mp)}</blocks>")
    moving.extend(list(ped_root))
    return 5, 4


def main():
    route, cum = route_data()
    mp = Map(MAP_FILE)
    tree = ET.parse(BASE_FILE)
    root = tree.getroot()

    pulk = root.find("PulkTraffic")
    if pulk is not None:
        pulk.clear()                 # 난수 교통은 빼고 영상 액터만 고정한다.

    traffic = root.find("TrafficControl")
    if traffic is None:
        raise SystemExit("TrafficControl이 없다")
    vehicles = add_players(traffic, route, cum)
    objects, people = add_objects(root, mp, route, cum)

    ET.indent(tree, space="    ")
    with open(OUT_FILE, "w", encoding="utf-8") as f:
        f.write("<?xml version='1.0' encoding='utf-8'?>\n")
        f.write(ET.tostring(root, encoding="unicode"))
        f.write("\n")
    print(f"-> {OUT_FILE}")
    print(f"   차량 {vehicles}대 · 보행자 {people}명 · 손수레 3개 · 라바콘 2개")


if __name__ == "__main__":
    main()
