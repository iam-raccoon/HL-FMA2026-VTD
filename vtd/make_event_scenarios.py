#!/usr/bin/env python3
"""공식 코스(v5 기반)에 '동적 이벤트'를 심은 테스트 시나리오 생성.

왜 필요한가(2026-08-14 실측): 공식 v1~v7은 상대차가 우리 차선에 없거나(경로에서 9m 밖 주차),
아예 스폰되지 않아(v2) **끼어들기·급정거·인레인 장애물이 VTD에서 한 번도 검증되지 않는다**.
같은 코스/같은 경로 위에 이벤트를 직접 심어 재현 가능한 테스트를 만든다.

⚠️ 배치 방법: PathRef(PathId=1)로 넣으면 스폰이 안 되는 경우가 있었다(EV_CUTIN 1차 실패).
   **PosAbsolute로 실제 x,y를 주는 방식은 확실히 스폰된다**(공식 v4가 그 방식). 그래서 우리가
   녹화한 공식 경로(routes/official_v1_v6.json)에서 좌표를 뽑아 배치한다.
   좌우 오프셋: 경로 진행방향 기준 왼쪽이 +. 옆차선은 보통 오른쪽(-3.5).

트리거는 <PosRelative Pivot="Ego" Distance="N"/> = ego가 N m 이내 접근 시 발동.

사용(VTD PC): python3 make_event_scenarios.py [route.json]
"""
import json
import math
import os
import sys

SCEN = "/home/user/Hexagon/VTD.2025.2/Data/Projects/Current/Scenarios"
BASE = "HL_FMA_VTD_LivingLab_v5.xml"        # ego만 있는 깨끗한 코스
ROUTE = sys.argv[1] if len(sys.argv) > 1 else "/tmp/v1.json"


def load_route():
    return json.load(open(ROUTE, encoding="utf-8"))["ego_route"]


def at_distance(route, s_target, lateral=0.0):
    """경로 s_target[m] 지점의 (x, y, heading). lateral>0 = 진행방향 왼쪽으로 이동."""
    acc = 0.0
    for i in range(len(route) - 1):
        (ax, ay), (bx, by) = route[i], route[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        if acc + seg >= s_target:
            r = (s_target - acc) / max(1e-6, seg)
            x, y = ax + r * (bx - ax), ay + r * (by - ay)
            hdg = math.atan2(by - ay, bx - ax)
            nx, ny = -math.sin(hdg), math.cos(hdg)      # 좌법선
            return x + lateral * nx, y + lateral * ny, hdg
        acc += seg
    bx, by = route[-1]
    ax, ay = route[-2]
    hdg = math.atan2(by - ay, bx - ax)
    return bx, by, hdg


def player_abs(name, vtype, x, y, hdg, speed=0.0):
    return f'''        <Player>
            <Description Driver="DefaultDriver" Control="internal" AdaptDriverToVehicleType="true" Type="{vtype}" Name="{name}"/>
            <Init>
                <Speed Value="{speed:.16e}"/>
                <PosAbsolute X="{x:.16e}" Y="{y:.16e}" Z="0.0000000000000000e+00" Direction="{hdg % (2*math.pi):.16e}" AlignToRoad="true"/>
            </Init>
        </Player>
'''


def actions(name, blocks):
    return f'        <PlayerActions Player="{name}">\n{"".join(blocks)}        </PlayerActions>\n'


# ⚠️ LaneChange 는 Force="true" 라 **충돌을 무시하고 밀어붙인다.** 우리가 완전히 정지해
#    있어도 옆에서 그대로 파고든다(2026-08-15 EV_COMBO_CHAIN 실측: ego 0.0km/h 인데
#    스크립트 차가 초당 3m 로 접근해 실여유 -2.57m). 그런 접촉은 제어 문제가 아니니
#    채점 결과를 읽을 때 구분할 것.
def act_lanechange(trigger_dist, direction=1, time=3.0):
    return f'''            <Action Name="">
                <PosRelative NetDist="false" Distance="{trigger_dist:.16e}" Pivot="Ego"/>
                <LaneChange Direction="{direction}" Force="true" ExecutionTimes="1" ActiveOnEnter="true" DelayTime="0.0000000000000000e+00" Time="{time:.16e}"/>
            </Action>
'''


def act_speed(trigger_dist, target, rate):
    return f'''            <Action Name="">
                <PosRelative NetDist="false" Distance="{trigger_dist:.16e}" Pivot="Ego"/>
                <SpeedChange Rate="{rate:.16e}" Target="{target:.16e}" Force="true" ExecutionTimes="1" ActiveOnEnter="true" DelayTime="0.0000000000000000e+00"/>
            </Action>
'''


def build(out_name, players_xml, actions_xml):
    t = open(os.path.join(SCEN, BASE), encoding="utf-8").read()
    anchor = '        <PlayerActions Player="Ego"/>\n'
    if anchor not in t:
        raise SystemExit("Ego PlayerActions 앵커 없음 — 베이스 XML 확인")
    t = t.replace(anchor, players_xml + anchor + actions_xml)
    open(os.path.join(SCEN, out_name), "w", encoding="utf-8").write(t)
    print(f"  생성: {out_name}")


def build_full(out_name, players_xml="", actions_xml="", objects_xml=""):
    """플레이어(차)와 MovingObject(사람)를 **같은 시나리오에** 넣는다 — 복합 이벤트용.

    둘은 앵커가 다르다: 차는 <PlayerActions Player="Ego"/>, 사람은 <ObjectActions Object="New Object"/>.
    """
    t = open(os.path.join(SCEN, BASE), encoding="utf-8").read()
    pa = '        <PlayerActions Player="Ego"/>\n'
    if players_xml or actions_xml:
        if pa not in t:
            raise SystemExit("Ego PlayerActions 앵커 없음 — 베이스 XML 확인")
        t = t.replace(pa, players_xml + pa + actions_xml)
    if objects_xml:
        oa = '        <ObjectActions Object="New Object"/>\n'
        if oa not in t:
            raise SystemExit("MovingObjectsControl 앵커를 못 찾음")
        t = t.replace(oa, oa + objects_xml)
    open(os.path.join(SCEN, out_name), "w", encoding="utf-8").write(t)
    print(f"  생성: {out_name}")


CAR = "AlfaRomeo_Brera_10_BiancoSpino"


# ⚠️ 보행자는 Player(Type="Hannah")로도, SCP Player|Create 로도 스폰되지 않았다(둘 다 실측 실패).
#    확실히 뜨는 방법 = MovingObjectsControl 오브젝트(공식 v5의 기름통과 같은 방식).
#    DummyPerson: L0.234 x W0.397 x H1.401 -> 우리 로직의 '높이 1.2m 이상 = 보행자' 판정에 걸린다.
PED_DEF = "DummyPerson"
GROUND_Z = 42.8        # 이 구간 지면 고도(기름통 Z=42.83 실측). 0 이면 지하에 박혀 안 보인다.


def moving_object(name, definition, x, y, z, hdg):
    return (f'        <Object Type="other" Name="{name}" Definition="{definition}">\n'
            f'            <StartPosAbs X="{x:.16e}" Y="{y:.16e}" Z="{z:.16e}" '
            f'Direction="{hdg % (2*math.pi):.16e}" Pitch="0.0" Roll="0.0"/>\n'
            f'        </Object>\n'
            f'        <ObjectActions Object="{name}"/>\n')


def build_ped(route):
    """우리 차선 안에 사람을 세워 양보/정지를 검증 (공식 v1~v7엔 보행자가 없음)."""
    x, y, h = at_distance(route, 120.0, lateral=0.0)
    src = open(os.path.join(SCEN, BASE), encoding="utf-8").read()
    anchor = '        <ObjectActions Object="New Object"/>\n'
    if anchor not in src:
        raise SystemExit("MovingObjectsControl 앵커를 못 찾음")
    out = src.replace(anchor, anchor + moving_object("Ped", PED_DEF, x, y, GROUND_Z, h + math.pi / 2))
    open(os.path.join(SCEN, "EV_PED.xml"), "w", encoding="utf-8").write(out)
    print(f"  생성: EV_PED.xml")
    print(f"      Ped    @({x:.1f},{y:.1f}) Z={GROUND_Z}")


def build_oncoming(route):
    """★추월해 들어갈 차선이 막혀 있는 경우: 내 차선 정지차 + **목표차선에도 정지차**.

    실측(2026-08-15): 이 코스의 좌측 차선은 반대차선이 아니라 같은 방향 차선이다
    (대향차를 놓아도 AlignToRoad 로 도로 방향에 정렬돼 우리와 같은 쪽으로 달림).
    그리고 기존 로직은 '속도>1인 차'만 위험으로 봐서 **목표차선의 정지차를 무시하고
    그대로 들이받으러 들어갔다** -> 정지차도 막도록 수정하고 이 시나리오로 검증한다.
    """
    bx, by, bh = at_distance(route, 210.0, lateral=0.0)          # 내 차선 정지차
    sx, sy, sh = at_distance(route, 218.0, lateral=3.5)          # 목표차선도 막힘
    players = (player_abs("Blocker", CAR, bx, by, bh, speed=0.0)
               + player_abs("SideBlock", CAR, sx, sy, sh, speed=0.0))
    acts = (actions("Blocker", [act_speed(500.0, target=0.0, rate=8.0)])
            + actions("SideBlock", [act_speed(500.0, target=0.0, rate=8.0)]))
    build("EV_BOTHBLOCK.xml", players, acts)
    print(f"      Blocker @({bx:.1f},{by:.1f})  SideBlock @({sx:.1f},{sy:.1f}) 목표차선")


def build_curve(route):
    """★곡선 구간 오판 검증: 굽은 구간의 '옆차선'에 정지차를 둔다.

    곡선에서 ego 직교좌표 fy 는 곡률에 오염돼 옆차선 차가 내 차선처럼 보인다
    (그래서 route_frame 투영을 쓴다). 투영이 제대로 되면 여기서 **멈추지 않고 지나가야** 한다.
    멈추면 = 헛브레이크 = 시간 손해. 코스에서 제일 굽은 두 곳(s=126 R127m, s=744 R338m).
    """
    ax, ay, ah = at_distance(route, 126.0, lateral=-3.5)
    bx, by, bh = at_distance(route, 744.0, lateral=3.5)
    players = (player_abs("CurveA", CAR, ax, ay, ah, speed=0.0)
               + player_abs("CurveB", CAR, bx, by, bh, speed=0.0))
    acts = (actions("CurveA", [act_speed(500.0, target=0.0, rate=8.0)])
            + actions("CurveB", [act_speed(500.0, target=0.0, rate=8.0)]))
    build("EV_CURVE.xml", players, acts)
    print(f"      CurveA @({ax:.1f},{ay:.1f}) 우측  CurveB @({bx:.1f},{by:.1f}) 좌측")


def build_combo_pass_ped(route):
    """★복합① 추월 '기동 도중에' 보행자가 나타난다.

    지금까지 검증은 이벤트를 하나씩만 봤다. 실제로 위험한 건 **한 판단이 진행 중일 때
    다른 판단이 끼어드는 것**이다. 여기선 추월 FSM 이 PASS 상태로 반대차선에 나가 있는
    동안 그 자리에 사람이 서 있다.
    기대: 추월을 시작해도 보행자 앞에서 선다(정지 우선). 특히 STALL_ESCAPE(오래 멈춰있으면
    강제 전진)가 보행자를 밀고 가면 안 된다 — 그래서 YIELD_PED 는 탈출 대상에서 제외돼 있다.
    """
    # ⚠️ 사람을 정지차보다 한참 뒤(s=228)에 두면 이미 복귀한 뒤라 시험이 안 된다.
    #    추월 궤적이 반대차선에 나가 있는 구간(정지차 ±6m) 한복판에 둬야 한다.
    bx, by, bh = at_distance(route, 210.0, lateral=0.0)      # 내 차선 정지차
    px, py, ph = at_distance(route, 214.0, lateral=3.2)      # 추월해 나갈 자리에 사람
    build_full("EV_COMBO_PASSPED.xml",
               players_xml=player_abs("Blocker", CAR, bx, by, bh, speed=0.0),
               actions_xml=actions("Blocker", [act_speed(500.0, target=0.0, rate=8.0)]),
               objects_xml=moving_object("PedPass", PED_DEF, px, py, GROUND_Z, ph + math.pi / 2))
    print(f"      Blocker @({bx:.1f},{by:.1f})  PedPass @({px:.1f},{py:.1f}) 목표차선")


def build_combo_chain(route):
    """★복합② 회복할 틈 없이 이벤트가 연달아 온다.

    급정거(s=60) -> 그 차를 추월 -> 추월 끝나자마자 옆차선 차가 끼어듦(s=115) -> 인레인 장애물(s=175).
    기대: 각 이벤트를 개별로는 통과했으니, 여기선 **상태가 남아 오염되는지**를 본다
    (추월 FSM 이 RETURN 을 못 끝낸 채 다음 이벤트를 만나는 상황).
    """
    lx, ly, lh = at_distance(route, 60.0, lateral=0.0)
    cx, cy, ch = at_distance(route, 115.0, lateral=-3.5)     # 옆차선(우측)에서 대기
    ox, oy, oh = at_distance(route, 175.0, lateral=0.0)
    players = (player_abs("Lead", CAR, lx, ly, lh, speed=0.0)
               + player_abs("Cutin2", CAR, cx, cy, ch, speed=0.0)
               + player_abs("Block2", CAR, ox, oy, oh, speed=0.0))
    acts = (actions("Lead", [act_speed(90.0, target=5.55, rate=3.0),
                             act_speed(25.0, target=0.0, rate=8.0)])
            + actions("Cutin2", [act_lanechange(25.0, direction=1, time=3.0),
                                 act_speed(24.0, target=3.0, rate=3.0)])
            + actions("Block2", [act_speed(500.0, target=0.0, rate=8.0)]))
    build_full("EV_COMBO_CHAIN.xml", players_xml=players, actions_xml=acts)
    print(f"      Lead @({lx:.1f},{ly:.1f})  Cutin2 @({cx:.1f},{cy:.1f})  Block2 @({ox:.1f},{oy:.1f})")


def main():
    route = load_route()

    # ① 끼어들기: 옆차선(우측 3.5m) 70m 지점에 배치 -> ego 30m 접근 시 우리 차선으로 + 감속
    x, y, h = at_distance(route, 45.0, lateral=-3.5)
    build("EV_CUTIN.xml",
          player_abs("Cutin", CAR, x, y, h, speed=0.0),
          actions("Cutin", [act_lanechange(25.0, direction=1, time=3.0),
                            act_speed(24.0, target=3.0, rate=3.0)]))
    print(f"      Cutin  @({x:.1f},{y:.1f}) hdg={h:.2f}")

    # ② 앞차 급정거: 같은 차선 60m 지점 -> 주행하다 ego 25m 접근 시 rate 8로 급정거
    x, y, h = at_distance(route, 60.0, lateral=0.0)
    build("EV_LEADBRAKE.xml",
          player_abs("Lead", CAR, x, y, h, speed=0.0),
          actions("Lead", [act_speed(90.0, target=5.55, rate=3.0),
                           act_speed(25.0, target=0.0, rate=8.0)]))
    print(f"      Lead   @({x:.1f},{y:.1f}) hdg={h:.2f}")

    # ③ 인레인 정지 장애물(사고차량): 우리 차선 150m 지점에 정차 -> 정지 후 추월해야 함
    x, y, h = at_distance(route, 210.0, lateral=0.0)
    # ⚠️ 실측: Init만 있고 '실제 Action 블록'이 없으면 스폰되지 않는다(빈 PlayerActions도 불가).
    #    끼어들기 차는 Action이 있어서 떴다 -> 장애물에도 무해한 액션(정지 유지)을 준다.
    build("EV_OBSTACLE.xml", player_abs("Blocker", CAR, x, y, h, speed=0.0),
          actions("Blocker", [act_speed(500.0, target=0.0, rate=8.0)]))
    print(f"      Blocker@({x:.1f},{y:.1f}) hdg={h:.2f}")

    build_ped(route)
    build_oncoming(route)
    build_curve(route)
    build_combo_pass_ped(route)
    build_combo_chain(route)


if __name__ == "__main__":
    main()
