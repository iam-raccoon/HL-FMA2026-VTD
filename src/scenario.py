"""
테스트 시나리오 정의 + 로더/프리셋.

ego 출발/목표, 제한속도, 신호등 위상, 액터(보행자/차량/장애물)의
'등장 조건(트리거)'과 '움직임'을 선언적으로 정의한다. mock_vtd.py 가 실행/채점.

액터 spawn 트리거:
  {"at_time": 5.0}                     # 5초에 등장
  {"ego_x": 60}                        # ego가 x=60 지날 때
  {"ego_within": 20, "of": [80, 0]}    # ego가 (80,0)의 20m 안에 들 때
액터 motion:
  {"kind":"static",   "pos":[x,y]}
  {"kind":"constant", "pos":[x,y], "vel":[vx,vy]}   # 등속(앞차/횡단차)
  {"kind":"crossing", "pos":[x,y], "vel":[vx,vy]}   # 횡단(보행자)
  {"kind":"waypoints","pts":[[x,y],...], "speed":v} # 경로추종(컷인 등)
"""
import json
from dataclasses import dataclass, field, asdict
from typing import List


@dataclass
class Actor:
    id: int
    type: str                 # "pedestrian" | "vehicle" | "obstacle"
    size: list                # [length, width, height]
    spawn: dict               # 등장 트리거
    motion: dict              # 움직임
    # 공식 시나리오(v1~v7) 재현용 — Ego 거리 트리거로 속도/차선 변경
    #   events: [{"ego_within": 30, "set_speed": 13.9, "rate": 4.0, "lane_change": +1}]
    events: list = field(default_factory=list)


@dataclass
class TrafficLight:
    id: int
    stop_line_x: float        # 정지선 x (채점 GT)
    phases: list              # [["GREEN",15],["YELLOW",3],["RED",18]] 반복


@dataclass
class Zone:
    """제한속도 구간 (예: 어린이보호구역). x_start~x_end 구간 limit[m/s] 적용."""
    name: str
    x_start: float
    x_end: float
    limit: float          # m/s (어린이보호구역 30km/h=8.33, 대회 규정따라 조정)


@dataclass
class Scenario:
    name: str
    ego_start: list                        # [x, y, heading]
    ego_goal: list                         # [x, y]
    speed_limit: float = 8.33              # m/s 기본 제한 (8.33=30km/h)
    duration: float = 60.0
    actors: List[Actor] = field(default_factory=list)
    lights: List[TrafficLight] = field(default_factory=list)
    zones: List[Zone] = field(default_factory=list)          # 보호구역 등 속도구간
    ego_route: list = field(default_factory=list)            # [[x,y],...] 비었으면 출발->목표 직선
    respawns: list = field(default_factory=list)             # 리스폰 시뮬 [{"at_time":8,"to":[x,y,heading]}]
    tl_stops: dict = field(default_factory=dict)             # {tl_id: [x,y]} 실측 정지선(9910 tl_id 대응)

    # ---- '지도 사전지식'(제어코드가 정당하게 아는 정적정보) ----
    def limit_at(self, x):
        """해당 x에서 유효 제한속도[m/s] (보호구역 반영)."""
        v = self.speed_limit
        for z in self.zones:
            if z.x_start <= x <= z.x_end:
                v = min(v, z.limit)
        return v

    def route_points(self, step=1.0):
        """ego 경로 웨이포인트. ego_route 있으면 조밀화, 없으면 출발->목표 직선."""
        pts = self.ego_route if self.ego_route else [self.ego_start[:2], self.ego_goal]
        out = []
        for i in range(len(pts) - 1):
            (x0, y0), (x1, y1) = pts[i], pts[i + 1]
            seg = max(1e-6, ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5)
            n = max(1, int(seg / step))
            for k in range(n):
                r = k / n
                out.append((x0 + r * (x1 - x0), y0 + r * (y1 - y0)))
        out.append(tuple(pts[-1]))
        return out

    def stop_line_ahead(self, x):
        """ego 전방 가장 가까운 정지선 x (없으면 None). RED일 때 정확 정지용."""
        cands = [t.stop_line_x for t in self.lights if t.stop_line_x > x]
        return min(cands) if cands else None

    @staticmethod
    def load(path):
        d = json.load(open(path, encoding="utf-8"))
        actors = [Actor(**a) for a in d.get("actors", [])]
        lights = [TrafficLight(**t) for t in d.get("lights", [])]
        zones = [Zone(**z) for z in d.get("zones", [])]
        return Scenario(
            d["name"], d["ego_start"], d["ego_goal"],
            d.get("speed_limit", 8.33), d.get("duration", 60.0),
            actors, lights, zones, d.get("ego_route", []),
            d.get("respawns", []), d.get("tl_stops", {}),
        )

    def save(self, path):
        json.dump(asdict(self), open(path, "w", encoding="utf-8"),
                  ensure_ascii=False, indent=2)
