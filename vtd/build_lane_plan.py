#!/usr/bin/env python3
"""경로 각 점에서 **어느 차로에 있어야 하고 어디로 비킬 수 있는지**를 xodr에서 미리 계산한다.

9910 패킷에는 차로 정보가 전혀 없다. 그래서 도로교통법을 지키려면 지도를 미리 읽어둘 수밖에 없다.
이 파일 하나가 아래를 전부 담당한다(2026-08-15 실측으로 필요성이 확인된 것들):

  ① 지정차로 — 좌회전을 직진 차로에서 하고 있었다. junction 39 실측:
     lane -1 만 좌회전 연결로로 이어지는데 우리는 lane -2(직진 전용)로 진입해 좌회전했다.
     -> 교차로 전에 미리 회전 차로로 옮기도록 목표 오프셋(need)을 깔아둔다.
  ② 반대차선 침범 조건 — 편도 1차선에서 중앙선이 **점선**일 때만 넘어갈 수 있다.
     이 코스는 대부분 편도 다차선이라 왼쪽으로 나가면 중앙선 침범 리스폰(감점)이다.
  ③ 차로 단위 회피 — 물건을 피할 때 1.6m만 걸치면 그 자체로 진로위반이다.
     옆 차로 하나를 통째로 쓸 수 있는지(폭과 여유)를 알아야 제대로 된 차선변경을 한다.

부호 약속: 결과의 좌/우·need 는 **주행 진행방향 기준**(왼쪽 +). xodr 의 t 부호가 아니다
(좌측 차선군은 도로 s 와 반대로 달리므로 뒤집힌다 — 놓치면 정확히 반대가 된다).

출력 JSON: {"pts": [ null | {"lane":int,"w":float,"l":float,"r":float,
                             "xl":float,"xr":float,"need":float, ..., "pl":float,"pr":float}, ... ]}
  l/r  = 같은 방향 주행차로 안에서 좌/우로 비킬 수 있는 거리[m]
  xl/xr= 중앙선을 넘어 반대차선으로 갈 수 있는 거리[m] (점선일 때만 >0, 실선/중앙분리대면 0)
  need = 지정차로를 지키려면 지금 있어야 할 횡오프셋[m] (0이면 현재 차로가 맞음)

사용: python3 build_lane_plan.py <xodr> <route.json> [out.json]
"""
import heapq
import json
import math
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from build_tl_map import road_point                      # noqa: E402

DRIVABLE = {"driving"}
CELL = 12.0                 # 공간 격자 한 칸[m] — 전 도로 선형탐색이면 너무 느리다
APPROACH = 110.0            # 교차로 이 거리 전부터 회전 차로로 옮기기 시작한다
FULL_BY = 45.0              # 교차로 이 거리 전에는 **다 옮겨져 있어야** 한다
TURN_MIN = 25.0             # 연결로 방위변화가 이 이상이면 좌/우회전으로 본다
SIG_LOOK = 22.0             # 경로 자체의 차선변경을 이 앞까지 내다보고 지시등
SIG_LEAD = 32.0             # 그 구간보다 **이만큼 앞에서부터** 켠다 [시행령 별표2 = 30m]
#   ⚠️ 32 인 이유: 경로 격자가 1.4m 라 walk-back 이 `lead` **바로 아래**에서 멈춘다.
#     30 으로 두면 실측 28.7~29.4m 가 나와 법 문턱을 0.6~1.3m 차이로 못 넘는다.
SIG_LEAD_SEC = 3.0          # 대회 기준[s] — 아래 참고
SIG_LEAD_MAX = 60.0         # 상한[m]
#   ★★대회 채점은 **거리가 아니라 시간**이다 [안내문 2026-08-27 · 평가항목 13]:
#     "차로 변경 시 **3초 내 미점등**: 경미(-3)".
#   법(30m)과 둘 다 만족해야 하므로 **그 지점 제한속도로 3초를 환산**해 더 큰 쪽을 쓴다.
#   50km/h 구간이면 13.9*3 = 41.7m 라 30m 로는 2.16초밖에 안 된다.
#   ⚠️ 자차 속도가 아니라 **제한속도**로 잡는다 — 차로계획은 정적이라 그때 속도를 모른다.
#     제한보다 느리게 달리면 시간은 더 넉넉해지므로 안전한 방향이다.


def sig_lead_for(lim):
    """그 지점 제한속도[m/s]에서 필요한 지시등 선행거리[m]."""
    if not lim:
        return SIG_LEAD
    return min(SIG_LEAD_MAX, max(SIG_LEAD, SIG_LEAD_SEC * float(lim) + 4.0))
#   ★★법이 "그 행위를 하려는 지점에 이르기 전 **30미터 이상**의 지점에 이르렀을 때"
#   신호하라고 한다. 그런데 여기는 **움직임이 감지되는 지점부터** 켜고 있었다 —
#   즉 이미 옮기기 시작한 뒤다. 사용자가 두 번 지적했다:
#     2026-08-27 코스 B (1460,389) "차선 이미 반쯤 들어간 상태로 킴"
#     2026-08-30 코스 E (1116,-42) "차선 왼으로 바꿀때 깜빡이 안 킴"
#   실측(코스 E t=646~649): 우측끝까지 1.45 -> 4.63m 로 옮겨가는데
#     t=646.1 sig=2(우, 직전 우회전의 잔상) · t=646.9~648.0 **sig=0(꺼짐)**
#     t=648.6 sig=1(좌)  <- 이동이 거의 끝난 뒤
#   -> 판정 구간 [i, j] 를 **뒤로 30m 늘려** i 이전부터 켠다.
SIG_MIN = 1.5               # 도로 t 가 이만큼 움직이면 차선 변경으로 본다(반차로)
SIG_JUNC_SETTLE = 0.0       # 교차로를 빠져나온 뒤 이만큼은 판정하지 않는다[m]
#   ★★★2026-08-28: **25.0 이었는데 0 으로 되돌렸다. 그 25m 는 틀린 처방이었다.**
#   원래 의도는 "우회전을 마치자마자 왼쪽 깜빡이가 켜진다"(사용자 지적 2026-08-27)를
#   막는 것이었다. 그런데 그 세 곳을 지도로 다시 재보니 **전부 진짜 차선변경**이었다 —
#     (960,-507) l 7.58->4.45  r 1.55->4.73  폭 9.13->9.18 (유지)  = 한 차로 왼쪽
#     (1099, 22) l 7.45->4.72  r 1.72->4.41  폭 9.17->9.13 (유지)  = 한 차로 왼쪽
#     (1127,-47) l 7.14->4.29  r 1.55->4.15  폭 8.69->8.44 (유지)  = 한 차로 왼쪽
#   진짜 원인은 **회전 지시등이 늦게 꺼지는 것**이었고 그건 따로 고쳤다
#   (drive.py TURN_SIG_HOLD_DPS 자차 회전율 래치, 커밋 fe972d4).
#   25m 를 두는 대가는 컸다 — 코스 A·B·G 실측:
#     SETTLE=25  점등 11구간 / 차선변경 23곳 -> **12곳이 깜빡이 없음** [법 제38조 위반]
#     SETTLE=12  점등 18구간 -> 5곳 없음
#     SETTLE= 6  점등 20구간 -> 3곳 없음
#     SETTLE= 0  점등 23구간 -> **0곳** · 오점등도 **0**
#   오점등은 아래 '도로 폭 보존' 검사가 이미 다 막는다(네 값 모두 오점등 0). 즉 이
#   억제는 **막는 것 없이 필요한 것만 죽이고 있었다.**
ZONE_REACH = 300.0          # 보호구역 표시에서 망거리 이 안이면 30 을 유지한다[m]
#   어린이보호구역은 주 출입문 300m 이내로 지정된다[학교보건법 시행령]. 그 값을 그대로 쓴다.
#   ⚠️ 문턱을 낮출수록 시간은 벌지만 **보호구역을 놓친다.** 실측(보호구역 무신호
#      횡단보도 64곳 기준, ZONE_REACH=300):
#        편도 2+ -> 보호구역 45/64 (**19곳 손실**) · 4697m 확보
#        편도 3+ -> 보호구역 62/64 (** 2곳 손실**) · 1526m 확보
#        편도 4+ -> 보호구역 **64/64(손실 0)**     ·  696m 확보   <- 이것만 안전
#      보호구역 미정지는 [법 제27조⑦] **확실한 위반**이고 시간 이득은 제한시간을
#      모르는 상태의 불확실한 이득이다. 안전 쪽으로 잡는다.
#   어린이보호구역은 주 출입문 300m 이내로 지정된다. 표시가 구간 안에 반복되므로
#   150m 면 표시 사이 공백을 덮는다. 보호구역을 놓치는 쪽이 훨씬 위험해서 넉넉히 잡았다.
ARROW_LOOK = 90.0           # 교차로 진입 전 이 거리 안의 노면 화살표를 그 차로의 지시로 본다
ARROW_T = 1.75              # 화살표 t 가 내 차로 중심에서 이 안이면 '내 차로 화살표'
NEED_MAX = 1.2              # 이보다 큰 횡이동은 need 로 때우지 않는다 — 아래 참고

# 노면 진행방향 표시(도로교통법 시행규칙 별표6). 이름은 이 맵의 오브젝트 그대로.
ARROW_MOVES = {
    "RM_537_ST":  {"straight"},                 # 537 직진
    "RM_537_LT":  {"left"},                     # 537 좌회전
    "RM_537_RT":  {"right"},                    # 537 우회전
    "RM_538_SLT": {"straight", "left"},         # 538 직진 및 좌회전
    "RM_538_SRT": {"straight", "right"},        # 538 직진 및 우회전
    "RM_539_LUT": {"left", "uturn"},            # 539 좌회전 및 유턴
    "RM_539_UT":  {"uturn"},                    # 539 유턴
}


def poly(el, ds):
    return (float(el.get("a", 0)) + float(el.get("b", 0)) * ds
            + float(el.get("c", 0)) * ds * ds + float(el.get("d", 0)) * ds ** 3)


def at_offset(els, ds):
    """sOffset 이 붙은 레코드 목록에서 ds 지점에 **유효한** 것을 고른다.

    ★차로 하나에 <width> 가 여러 개일 수 있다(이 맵: 5734개 차로 정의 중 650개,
      최대 15개). 각 레코드는 자기 sOffset 부터 다음 레코드까지만 유효하고,
      다항식 인자도 (ds - sOffset) 이다.
    ⚠️ 첫 레코드 하나만 쓰면(원래 그랬다) 3차항이 유효범위 밖에서 폭발한다 —
       실측: road 420 lane -1 의 폭이 33m 지점에서 2.98m 여야 하는데 **7.71m** 로
       계산돼, 차로 중심선이 laneSection 경계에서 2.5m 튀고 경로에 77° 헤딩 반전이
       생겼다. 차로 여유(lane_plan)와 중앙선 점선 판정도 같이 오염된다.
    """
    best = None
    for e in els:
        if float(e.get("sOffset", 0)) <= ds + 1e-6:
            best = e
    return best if best is not None else (els[0] if els else None)


class Map:
    def __init__(self, xodr):
        root = ET.parse(xodr).getroot()
        self.roads = {r.get("id"): r for r in root.findall("road")}
        self.juncs = root.findall("junction")
        # ★★**갈래가 2개뿐인 junction 은 교차로가 아니라 '모퉁이'다.**
        #   xodr 은 두 도로가 만나 꺾이는 곳도 junction 으로 표시한다. 실측 2026-08-30
        #   이 맵: junction 94개 중 **41개(44%)가 갈래 2개**다.
        #   예) (785,571) junction 91 = incomingRoad [2813, 2814] 둘뿐, road 3099 는
        #       52.7m 에 93° 꺾이는 **굽은 길**이다. 가로지를 교통 자체가 없다.
        #   이걸 교차로로 세면 커브에서 회전 지시등이 켜지고, 대향차 양보(YIELD_CROSS)와
        #   우회전 일시정지까지 걸린다. 사용자 지적 2026-08-30 코스 E (785,571):
        #     "길 자체가 그냥 휘어진 도로다. 좌회전이 아니라 그냥 길이 굽은 도로"
        #     -> t=284.8 [YIELD_CROSS] 26.9 -> 0.0km/h, 좌측 깜빡이까지 켜졌다.
        #   실제로 코스 A 는 j=1 점 699개 중 **259개(37%)가 모퉁이**다.
        self.junc_arms = {}
        for _j in self.juncs:
            _inc = {c.get("incomingRoad") for c in _j.findall("connection")}
            self.junc_arms[_j.get("id")] = len(_inc)
        self.grid = {}
        for rd in self.roads.values():
            L = float(rd.get("length"))
            s = 0.0
            while s <= L:
                try:
                    x, y, h = road_point(rd, s)
                except Exception:
                    break
                self.grid.setdefault((int(x // CELL), int(y // CELL)), []).append((x, y, h, rd, s))
                s += 2.0

    def locate(self, px, py):
        """가장 가까운 도로 reference line 위치 (road, s, t, hdg)."""
        gx, gy = int(px // CELL), int(py // CELL)
        cand = []
        rng = 1
        while not cand and rng <= 4:
            for a in range(gx - rng, gx + rng + 1):
                for b in range(gy - rng, gy + rng + 1):
                    cand.extend(self.grid.get((a, b), ()))
            rng += 1
        if not cand:
            return None
        x, y, h, rd, s = min(cand, key=lambda q: (q[0] - px) ** 2 + (q[1] - py) ** 2)
        t = (px - x) * -math.sin(h) + (py - y) * math.cos(h)
        return rd, s, t, h

    def lanes(self, rd, s):
        """[(id, type, t_lo, t_hi, roadMark)] — t 오름차순 아님(좌우 따로)."""
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
            for ln in sorted(sec.findall(f"{side}/lane"), key=lambda l: abs(int(l.get("id")))):
                we = at_offset(ln.findall("width"), ds)
                w = poly(we, ds - float(we.get("sOffset", 0))) if we is not None else 0.0
                a, b = edge, edge + sign * w
                mk = at_offset(ln.findall("roadMark"), ds)
                out.append((int(ln.get("id")), ln.get("type"), min(a, b), max(a, b),
                            mk.get("type") if mk is not None else None))
                edge = b
        return out


def road_speed_limits(mp):
    """도로별 제한속도[m/s] — xodr **노면표시 오브젝트**에서 읽는다.

    ★이 맵은 `<type><speed>` 도, 속도 표지판도 선언하지 않는다(전부 0개). 대신
      노면에 그려진 표시가 object 로 들어 있다(실측 2026-08-16):
        roadmark_speed_30.flt  71개 / 도로 35개  -> ⚠️**세지 않는다**(아래 2026-09-04)
        RM_517_50.flt          69개 / 도로 16개  -> 50km/h (517 = 속도제한)
        RM_536.flt             62개 / 도로 17개  -> 어린이보호구역 표시  ★
        RM_518.flt             77개 / 도로 23개  -> 보호구역 속도제한(숫자) ★
      공식 v1~v7 코스의 본선 도로(128·940·2011·2003·3195·1928)는 전부 **50** 이다
      — 30 으로 달리면 그만큼 손해다.

    ⚠️ 518·536 을 빠뜨렸다가 실주행에서 지적당했다("어린이 보호구역에서 50으로 다님").
       536(어린이보호구역)이 그려진 17개 도로 중 **16개에 roadmark_speed_30 이 없다**.
       이 맵은 보호구역을 536+518 쌍(≈5m 간격)으로만 표시한 데가 훨씬 많다.
       9경유지 코스 실측: s=1987~2528m 의 연속 스쿨존 7개 도로와 s=3386~3432m,
       합쳐 **419m 를 50km/h 로 통과**하고 있었다. 보호구역은 종류(어린이/노인/
       장애인) 불문 30km/h 라서 518 에 숫자가 안 붙어 있어도 30 으로 읽으면 된다.

    ★★★2026-09-04 주최측 공식 답변 — **`roadmark_speed_30` 은 세지 않는다.**
      다른 참가팀(인하대 A.I.M) 질의: "붉은색 노면이 아닌데도 제한속도가 적혀있는
      구간이 있습니다. 기존 50 인가요, 적힌 30 인가요?"
      주최측 답변:
        > **"본 대회에서는 붉은색 노면만 제한속도구간으로 평가합니다.
        >   해당 구간에서는 50kph로 주행하셔도 무방합니다."**
      한국에서 붉은 노면 = 보호구역이고, 이 맵에서는 RM_536(어린이보호구역)·
      RM_518(보호구역 속도제한)이 그것이다. `roadmark_speed_30` 은 **일반 노면에
      적힌 30** 이라 평가 대상이 아니다. 둘은 거의 겹치지 않는다(35개 도로 중
      보호구역과 겹치는 건 173·2818 둘뿐).
      실측 효과: 7개 코스 합 **367초** 단축(B -103s · G -80s · A -74s).

    ⚠️ 표시가 없는 도로가 많다(대부분 교차로 연결로). 그건 '제한 없음'이 아니라
       **직전 값 유지**다(실제 표지판과 같은 규칙). 그 처리는 호출부에서 한다.
    """
    MARK = {"RM_517_50": 50 / 3.6,
            "RM_536": 30 / 3.6,            # 어린이보호구역 (붉은 노면)
            "RM_518": 30 / 3.6}            # 보호구역 속도제한 (붉은 노면)
    out = {}
    for rid, rd in mp.roads.items():
        for o in rd.findall("objects/object"):
            v = MARK.get((o.get("name") or "").split(".")[0])
            if v is not None:
                # 한 도로에 여러 표시가 섞이면 **낮은 쪽**을 따른다(보호구역 우선).
                out[rid] = min(v, out.get(rid, v))
    return out


ZONE_MARGIN = 15.0      # 표시 s 범위 앞뒤로 붉은 노면이 이어진다고 볼 여유[m]
ZONE_CW_REACH = 30.0    # 횡단보도의 보호구역 판정에만 쓰는 추가 여유[m]
ZONE_CLUSTER = 25.0     # 표시들이 이 안에 모여 있으면 한 군집(한 진입부)으로 본다[m]


ZONE_ARM_REACH = 90.0   # 보호구역 교차로에 붙은 **표시 없는** 도로는 교차로 쪽 이만큼을 구역으로 본다[m] — 아래 2026-09-06 (2)
ZONE_MARKS = ("RM_536", "RM_518")


def mark_dir(o):
    """표시가 향한 진행방향: +1 = +s 로 달리는 차로(우측통행이라 lane id < 0), -1 = -s.
    xodr object 의 hdg 는 도로 기준 상대방위다(0 ≈ +s 방향 운전자가 읽음, π ≈ -s). 없으면 t 부호."""
    hdg = o.get("hdg")
    if hdg is not None:
        return 1 if math.cos(float(hdg)) >= 0.0 else -1
    return 1 if float(o.get("t") or 0.0) < 0.0 else -1


def lane_dir(lane):
    """차로 번호 -> 진행방향(+1/-1). 우측통행: 음수 차로가 +s 방향. 모르면 +1."""
    try:
        return -1 if int(lane) > 0 else 1
    except (TypeError, ValueError):
        return 1


def _touches(mp, ext, rd, margin=0.0):
    """연결로 `rd` 의 양끝 link 가 붙은 표시 도로의 구간이 **그 붙은 끝**에 닿아 있나."""
    for tag in ("predecessor", "successor"):
        el = rd.find(f"link/{tag}")
        if el is None or el.get("elementType") != "road":
            continue
        nb = el.get("elementId")
        nbr = mp.roads.get(nb)
        if nbr is None:
            continue
        L_nb = float(nbr.get("length"))
        cp = el.get("contactPoint")
        for d in (1, -1):
            e = ext.get((nb, d))
            if e is None:
                continue
            lo, hi = e
            touch_start = lo <= ZONE_MARGIN + margin
            touch_end = hi >= L_nb - ZONE_MARGIN - margin
            if ((cp == "start" and touch_start) or (cp == "end" and touch_end)
                    or (cp is None and (touch_start or touch_end))):
                return True
    return False


def zone_extent(mp):
    """{(rid, dir): (s_lo, s_hi)} — 표시가 있는 도로에서 **붉은 노면으로 볼 s 구간**, 진행방향별.

    ★★왜 도로 전체가 아니라 s 구간인가(2026-09-05 실측, 코스 A 사용자 지적 ④):
      road 2312 는 290m 인데 RM_536/518 표시가 **s=14~75 에만** 있다. 사용자가
      "붉은 도로가 아닌데 30 을 지킨다" 고 한 자리는 s=116~256 이었다. 도로 단위로
      30 을 주면 210m 를 8.3m/s 로 기어간다 — 감점은 아니지만(주최측: 붉은 노면만 평가)
      15분 시계에서 10초를 버린다. 표시 범위 ± ZONE_MARGIN 이 붉은 노면의 최선 근사다.
      road 2386(127m)은 표시가 s=20~26 과 101~113 양끝에 있어 거의 전체가 남는다 —
      진입부마다 표시를 두는 이 맵의 습관과 맞는다.

    ★★표시 **군집이 하나뿐인 도로는 그 표시가 향한 방향에 대해 도로 전체**를 쓴다(안전측).
      진입부 표시는 방향마다 구역 경계에 놓이므로 군집이 둘 이상이면 그 사이가 붉은
      노면이다(2312: {14,20}·{59~75} -> 14~75, 사용자가 s>100 은 붉지 않다고 확인).
      그런데 road 2818(476m, 편도1차로 스쿨존 이면도로)은 표시가 **s=378 한 군집**뿐이다
      — 구역이 378 에서 어느 쪽으로 뻗는지 지도로는 알 수 없다. 좁게 잡아 붉은 노면에서
      50 으로 달리면 항목2 **중대 -6** 이고, 넓게 잡으면 시간만 잃는다(주최측: 저속은
      감점 아님). 그래서 모르면 넓게 간다.

    ★★2026-09-06 **방향별**로 나눴다(코스 H 사용자 지적 3 "빨간 도로 아닌데 30"):
      road 2818 의 유일한 표시(s=378)는 hdg≈0·t<0, 즉 **+s 방향 차로의 것**이다. 우리는
      -s 차로(lane 1)로 s=462→402 를 달렸고 그 자리는 붉지 않았다. '도로 전체' 확장은
      **표시가 향한 방향에만** 준다. 반대 방향은 표시 ±ZONE_MARGIN 만 남긴다(붉은
      노면이 도로폭 전체일 가능성 대비). 군집이 둘 이상이면 예전처럼 **양 방향** 같은
      구간 — 2312 는 양방향 표시가 다 있고 그 규칙을 사용자가 확인한 것이다.
      이 맵에서 한 방향만 표시된 도로는 9개, 그중 476m 짜리는 2818 하나다.

    ★★표시 없는 도로도 **보호구역 교차로에 붙어 있으면** 교차로 쪽 ZONE_ARM_REACH 를
      구역으로 본다(코스 E 사용자 지적 2 "빨간 도로인데 50"): road 2264 는 표시가 하나도
      없는데 junction 60 쪽 s≈34 까지 붉었다. 이 맵의 보호구역 교차로는 팔마다 붉은
      노면이 이어지는데 표시는 그중 일부 팔에만 있다.
      xodr 에 '붉은 노면' 객체는 없다 — 도로 텍스처라 지도로는 이게 최선이다.
      (2) 2026-09-06 재주행: 40m 로는 부족했다 — 2264 는 **반대쪽 끝(junction 58) 직후부터**
          붉었다(코스 A·E "빨간 도로인데 시속 30 이상"). 2312 의 표시 구간도 교차로에서
          90m 라, 이 맵의 구역은 교차로에서 **약 90m** 뻗는다고 본다. 도로가 그보다 짧으면 전체.
      (3) '보호구역 교차로' 는 **표시 창(±MARGIN)** 이 닿는 교차로만이다 — 한 군집 도로의
          '도로 전체' 확장으로 닿은 건 세지 않는다. junction 89 가 그 경우였다: 2818 의
          유일한 표시(s=378)가 98m 떨어져 있는데 도로 전체 확장으로 닿아 팔 4개(2815·2816·
          2817·3060)를 구역으로 만들었다. 사용자(코스 E): "여기 왜 30 으로 감??" — 붉지 않았다.
    """
    out, hard = {}, {}
    for rid, rd in mp.roads.items():
        L = float(rd.get("length"))
        by = {1: [], -1: []}
        for ob in rd.findall("objects/object"):
            if (ob.get("name") or "").split(".")[0] in ZONE_MARKS:
                by[mark_dir(ob)].append(float(ob.get("s", 0)))
        ss = sorted(by[1] + by[-1])
        if not ss:
            continue
        clusters = 1 + sum(1 for a_, b_ in zip(ss, ss[1:]) if b_ - a_ > ZONE_CLUSTER)
        win = (max(0.0, ss[0] - ZONE_MARGIN), min(L, ss[-1] + ZONE_MARGIN))
        hard[(rid, 1)] = hard[(rid, -1)] = win        # 표시가 **실제로** 있는 창(교차로 판정용)
        for d in (1, -1):
            if clusters >= 2 or not by[d]:
                out[(rid, d)] = win                   # 표시 범위 ± 여유(반대 방향도 여기까진 본다)
            else:
                out[(rid, d)] = (0.0, L)              # 한 군집 + 그 방향의 표시 -> 도로 전체(안전측)
    # 표시 없는 도로 중 **보호구역 교차로**에 붙은 것 -> 교차로 쪽 ZONE_ARM_REACH
    zone_j = set()
    for rid, rd in mp.roads.items():
        jid = rd.get("junction", "-1")
        if jid != "-1" and _touches(mp, hard, rd):       # (3) 표시 창이 닿는 교차로만
            zone_j.add(jid)
    for rid, rd in mp.roads.items():
        if rd.get("junction", "-1") != "-1" or (rid, 1) in out:
            continue
        L = float(rd.get("length"))
        ends = set()
        for tag in ("predecessor", "successor"):
            el = rd.find(f"link/{tag}")
            if (el is not None and el.get("elementType") == "junction"
                    and el.get("elementId") in zone_j):
                ends.add(tag)
        if not ends:
            continue
        if len(ends) == 2 or L <= ZONE_ARM_REACH:
            seg = (0.0, L)                            # 양끝이 다 보호구역 교차로 -> 사이도 붉다고 본다
        elif "predecessor" in ends:
            seg = (0.0, ZONE_ARM_REACH)
        else:
            seg = (L - ZONE_ARM_REACH, L)
        out[(rid, 1)] = out[(rid, -1)] = seg
    return out


ZONE_LEAD_IN = 15.0    # 붉은 노면 **진입 쪽**으로 이만큼 미리 30 으로 본다[m] — 아래 in_zone 주석

def in_zone(mp, ext, rid, s, margin=0.0, dir=None, lead=0.0):
    """(rid, s) 가 붉은 노면 위인가. `dir` = 진행방향(+1/-1), None 이면 어느 방향이든.

    표시 도로 -> 그 방향의 표시 s 구간(±margin) 안이면 참.
    교차로 연결로 -> 양끝 link 가 붙은 표시 도로의 구간이 **그 붙은 끝**에 닿아 있으면 참
                  (구역 안 교차로는 붉게 칠해져 있고, 사이 도로는 전부 연결로 12~36m 다).
    그 밖 -> 거짓.
    """
    if (rid, 1) in ext or (rid, -1) in ext:
        for d in ((1, -1) if dir is None else (dir,)):
            e = ext.get((rid, d))
            if e is None:
                continue
            # ★★**진입 쪽으로만 미리 잡는다**(2026-09-11 사용자 관찰: "빨간 도로 들어가고
            #   속도를 줄여서 초반에는 35 정도 찍힌다"). 구간은 **노면표시(RM)가 그려진 s
            #   범위**로 잡는데, 화면에서 붉게 칠해진 아스팔트는 그 표시보다 앞에서 시작한다.
            #   표시 기준으로 30 을 걸면 눈에 보이는 붉은 구간 초입에서 아직 빠르다.
            #   나가는 쪽은 안 늘린다 — 시간만 버린다.
            lo, hi = e
            if lead > 0.0:
                if d > 0:
                    lo -= lead
                else:
                    hi += lead
            if lo - margin <= s <= hi + margin:
                return True
        return False
    rd = mp.roads.get(rid)
    if rd is None or rd.get("junction", "-1") == "-1":
        return False
    return _touches(mp, ext, rd, margin)


def zone_roads(mp):
    """**어린이·노인 보호구역 표시(RM_536/RM_518)가 그려진 도로.**

    제한속도 전파에서 '30' 이 어디서 왔는지를 가르는 데 쓴다 — 보호구역에서 온
    30 과 이면도로에서 온 30 은 성질이 다르다(아래 road_speed_limits_net 참고).
    """
    out = set()
    for rid, rd in mp.roads.items():
        for o in rd.findall("objects/object"):
            if (o.get("name") or "").split(".")[0] in ("RM_536", "RM_518"):
                out.add(rid)
                break
    return out


def road_graph(mp):
    """도로 **연결 그래프**(무향). xodr 의 link 와 junction connection 에서 만든다.

    Map 은 도로를 낱개로만 갖고 있어서 '옆 도로가 뭔지'를 아무도 모른다. 제한속도
    전파(아래)와 같이 **망을 따라가야 하는 계산**에는 이게 있어야 한다.
    """
    adj = {r: set() for r in mp.roads}

    def link(a, b):
        if a in adj and b in adj and a != b:
            adj[a].add(b)
            adj[b].add(a)

    for rid, rd in mp.roads.items():
        for tag in ("link/predecessor", "link/successor"):
            el = rd.find(tag)
            if el is not None and el.get("elementType") == "road":
                link(rid, el.get("elementId"))
    for j in mp.juncs:
        for c in j.findall("connection"):
            a, b = c.get("incomingRoad"), c.get("connectingRoad")
            link(a, b)
            # 연결로(connectingRoad)의 **반대쪽 끝**도 이어줘야 교차로를 관통한다.
            cr = mp.roads.get(b)
            if cr is not None:
                for tag in ("link/predecessor", "link/successor"):
                    el = cr.find(tag)
                    if el is not None and el.get("elementType") == "road":
                        link(b, el.get("elementId"))
    return adj


def road_speed_limits_net(mp):
    """도로별 제한속도[m/s]. **기본 50, 붉은 노면(보호구역)만 30.**

    ★★★2026-09-04 주최측 공식 답변으로 규칙이 확정됐다(그 전까지는 추정이었다).
      · "대회 전 구간의 기본 제한속도는 **50 km/h** 입니다. 보호구역을 제외한
         모든 구간에 대한 제한속도입니다." [주최측]
      · "보호구역은 **붉은색 노면 구간**이며, 해당 구간의 제한속도는 30km/h" [주최측]
      · "보호구역 제한속도 평가는 **Ego Position(후륜축)이 붉은색 보호구역 위에
         있을 때만** 평가합니다." [주최측]
      · "본 대회에서는 붉은색 노면만 제한속도구간으로 평가합니다.
         (일반 노면에 적힌 30 은) 50kph로 주행하셔도 무방합니다." [주최측]
      · xodr 사용은 공식 허용 — "제공된 파일에서 모두 사용하셔도 됩니다." [주최측]

    그래서 **망 전파를 없앴다.** 예전에는 노면표시를 도로망으로 퍼뜨렸는데, 그러면
    편도 1차로 스쿨존 이면도로(road 2818)의 30 이 300~571m 떨어진 편도 2~3차로
    간선(2815·2814·2813)까지 번졌다. 사용자 지적 2026-09-04:
    "왕복 4차선 도로에서도 30으로 다님". 맞는 지적이고, 주최측 기준으로도 틀렸다.

    붉은 노면 = 이 맵에서 `RM_536`(어린이보호구역)·`RM_518`(보호구역 속도제한)이
    그려진 도로. 거기에 **보호구역 도로끼리를 잇는 교차로 연결로**를 더한다 —
    구역 안 교차로도 붉게 칠해져 있고, 실측상 그 사이 도로는 전부 연결로(12~36m)다.

    ⚠️ `roadmark_speed_30`(일반 노면에 적힌 30, 35개 도로)은 **세지 않는다**.
       위 주최측 답변이 정확히 그 경우다.
    """
    zones = zone_roads(mp)
    slow = set(zones)
    if zones:
        adj = road_graph(mp)
        for rid, rd in mp.roads.items():
            if rd.get("junction", "-1") == "-1":
                continue                       # 연결로만 — 접근 도로까지 번지면 안 된다
            if any(n in zones for n in adj.get(rid, ())):
                slow.add(rid)
    return {r: (30 / 3.6 if r in slow else 50 / 3.6) for r in mp.roads}

def lay_need(pts, cum, a, need, notes, why):
    """진입점 a 앞으로 목표 횡오프셋을 깐다. **차로 통째 이동이면 깔지 않는다.**

    ⚠️ 2026-08-16 9경유지 실측이 이걸 강제했다. 노면 화살표를 지키려고 need -2.85m
       (한 차로)를 깔았더니, 차가 실제로 2.85m 옆으로 간 **직후 VTD 가 경로이탈로
       리스폰**시켰다(s=644m 에서 2건 연속, 리스폰 1 -> 3). VTD 는 시나리오 Path 를
       기준으로 이탈을 보므로, 차로를 통째로 옮기는 건 **오프셋으로 때울 일이 아니라
       경로 자체가 그 차로로 가야 하는 일**이다. 그건 plan_route 의 그래프가 한다
       (화살표가 금지하는 회전에 ARROW_VIOLATION 페널티).
       여기 남는 need 는 차로 안에서의 미세 보정뿐이다. 못 고치는 건 경고로 남긴다.
    """
    if abs(need) > NEED_MAX:
        notes.append(f"  ⚠️ 교차로@s={cum[a]:.0f}m {why} — 횡이동 {need:+.2f}m 는 차로 통째"
                     f" 이동이라 need 로 깔지 않는다(리스폰). 경로 계획이 할 일")
        return False
    for k in range(a, -1, -1):
        if pts[k] is None:
            break
        back = cum[a] - cum[k]
        if back > APPROACH:
            break
        pts[k]["need"] = round(need * ramp(back), 2)
    return True


def ramp(back):
    """교차로까지 back[m] 남았을 때 목표 오프셋의 몇 %를 요구할지.

    ⚠️ 원래는 APPROACH=70m 에서 시작해 **28m 전에야** 100% 였다. 50km/h 면 2초 —
       차가 옆으로 갈 수 있는 속도(LANE_RATE)로는 도저히 못 끝낸다. 그래서 차로가
       끝나는 데까지 끌려가다 마지막에 홱 꺾였다("그 끝에 가서야 차선을 바꿈").
       110m 에서 시작해 **45m 전에 끝낸다** = 65m(50km/h 로 4.7초)의 이동 구간.
    """
    return min(1.0, max(0.0, (APPROACH - back) / (APPROACH - FULL_BY)))


def lane_arrows(mp):
    """도로별 노면 진행방향 화살표 [(s, t, moves)] — xodr object 에서 읽는다.

    ★왜 laneLink 로는 안 되는가(2026-08-16 실측): 교차로 connection 의 laneLink 는
      '물리적으로 이어지는가'만 말한다. road 310 lane -1 은 직진 연결로가 **있어서**
      laneLink 상 직진이 허용되지만, 바닥에는 `RM_537_LT`(좌회전 전용)가 그려져 있다.
      실제로 우리는 거기서 직진했고 화면으로 바로 보였다. 채점자도 사람도 **화살표**를
      본다 — 그래서 화살표가 있으면 화살표가 laneLink 를 이긴다.

    차로 번호로 묶지 않는다. laneSection 마다 번호가 재부여되므로(road 128 은 7구간)
    같은 물리 차로가 다른 번호가 된다. 대신 **도로 t 좌표**로 경로와 직접 맞춘다 —
    화살표는 차로 한가운데 그려지므로 t 가 곧 그 차로의 중심이다.
    """
    out = {}
    for rid, rd in mp.roads.items():
        got = []
        for o in rd.findall("objects/object"):
            mv = ARROW_MOVES.get((o.get("name") or "").split(".")[0])
            if mv is not None:
                got.append((float(o.get("s", 0)), float(o.get("t", 0)), mv))
        if got:
            out[rid] = sorted(got)
    return out


def arrows_near(arrows, pts, cum, a):
    """진입점 a 뒤 ARROW_LOOK 안에서, 같은 도로 화살표를 (내 차로 / 다른 차로)로 가른다.

    반환 (mine, others) — mine 은 moves 합집합, others 는 [(dt_주행좌+, moves)].
    dt 는 **그 화살표 s 위치의 경로점**과의 t 차이라, 도로가 휘어도 어긋나지 않는다.
    """
    rid = pts[a]["_road"]
    got = arrows.get(rid)
    if not got:
        return None, []
    idx = [k for k in range(a, -1, -1)
           if pts[k] is not None and pts[k]["_road"] == rid
           and cum[a] - cum[k] <= ARROW_LOOK]
    if not idx:
        return None, []
    mine, others = set(), []
    for s_a, t_a, mv in got:
        k = min(idx, key=lambda q: abs(pts[q]["_s"] - s_a))
        if abs(pts[k]["_s"] - s_a) > 12.0:
            continue                       # 그 화살표는 이 구간 밖(반대편 진입로 등)
        dt = t_a - pts[k]["_t"]
        if abs(dt) <= ARROW_T:
            mine |= mv
        else:
            others.append((dt * pts[k]["_flip"], mv))
    return (mine or None), others


def route_headings(route):
    h = []
    for i in range(len(route)):
        a = route[max(0, i - 3)]
        b = route[min(len(route) - 1, i + 3)]
        h.append(math.atan2(b[1] - a[1], b[0] - a[0]))
    return h


def analyse_point(mp, px, py, hdg, hint=None):
    """한 점의 차로 상황. 차로 밖이면 None(모름).

    ★hint = (road, lane, s, t). plan_route 가 준다 — 그 점이 어느 도로 어느 차로의
      어디인지 **이미 알고 계산한 값**이다. 이게 있으면 교차로 안에서도 채운다.
      hint 가 없으면(녹화 경로 등) 지도에서 찾고, 교차로는 종전대로 None 이다.

    ⚠️ 교차로를 None 으로 비워두면 그 안에서 앞이 막혔을 때 **옆 차선이 비어 있어도
       비켜갈 후보로조차 안 본다**(실측 2026-08-16 EV_BOTHBLOCK: 오른쪽 3.17m 에
       같은 방향 빈 차로가 있는데 13.6m 앞 정지차 뒤에서 계속 대기). 연결로가 겹쳐
       '지도에서 되찾기'가 안 될 뿐, 우리가 계획할 때는 알고 있었다.
    """
    if hint is not None:
        rid, _lane_id, s, t = hint[:4]       # 5번째(차선변경 표시)는 여기서 안 쓴다
        rd = mp.roads.get(rid)
        if rd is None:
            return None
        try:
            _x, _y, h = road_point(rd, s)
        except Exception:
            return None
        is_junc = False                    # 계획이 준 차로라 교차로여도 믿는다
    else:
        loc = mp.locate(px, py)
        if loc is None:
            return None
        rd, s, t, h = loc
        is_junc = rd.get("junction", "-1") != "-1"
    if is_junc:
        return None                        # 교차로는 연결로가 겹쳐 신뢰 불가
    lanes = mp.lanes(rd, s)
    mine = next((l for l in lanes if l[2] - 1e-6 <= t <= l[3] + 1e-6), None)
    if mine is None or mine[1] not in DRIVABLE:
        return None
    flip = -1.0 if math.cos(hdg - h) < 0 else 1.0

    same = [l for l in lanes if l[1] in DRIVABLE and (l[0] > 0) == (mine[0] > 0)]
    lo, hi = min(l[2] for l in same), max(l[3] for l in same)
    room_p, room_m = hi - t, t - lo
    left, right = (room_p, room_m) if flip > 0 else (room_m, room_p)

    # 중앙선 넘기: 내 차선군과 반대군 사이에 주행차로가 아닌 게 끼어 있으면 불가(중앙분리대).
    inner = min((l for l in same), key=lambda l: abs(l[0]))       # id 절댓값 최소 = 안쪽
    between = [l for l in lanes if l[1] not in DRIVABLE
               and min(abs(l[2]), abs(l[3])) < abs(inner[2] if inner[0] > 0 else inner[3]) + 1e-6]
    opp = [l for l in lanes if l[1] in DRIVABLE and (l[0] > 0) != (mine[0] > 0)]
    cross = 0.0
    if opp and not between and inner[4] in ("broken", "broken broken"):
        olo, ohi = min(l[2] for l in opp), max(l[3] for l in opp)
        cross = (t - olo) if abs(olo - t) > abs(ohi - t) else (ohi - t)
        cross = abs(cross)
    # 반대차선은 '중앙선 쪽' = 안쪽 = t 가 0 에 가까워지는 방향
    to_center = -1.0 if mine[0] > 0 else +1.0          # t 증감 방향
    xl, xr = (cross, 0.0) if (to_center * flip > 0) else (0.0, cross)

    return dict(lane=mine[0], w=round(mine[3] - mine[2], 2),
                l=round(left, 2), r=round(right, 2),
                xl=round(xl, 2), xr=round(xr, 2), need=0.0, sig=0,
                # j=1 이면 교차로 안. 차로는 알지만 **앞지르기는 금지**다(도교법 22조).
                # 같은 방향 차선변경(장애물 회피)은 허용 — drive.py 가 구분해 쓴다.
                j=1 if rd.get("junction", "-1") != "-1" else 0,
                # jx=1 이면 **진짜 교차로**(갈래 3개 이상). 갈래 2개짜리는 모퉁이라
                # 0 이다 — 회전 지시등·양보·우회전 일시정지는 이걸 봐야 한다(Map 주석 참고).
                jx=1 if mp.junc_arms.get(rd.get("junction", "-1"), 0) >= 3 else 0,
                _road=rd.get("id"), _t=t, _s=s, _flip=flip, _lanes=lanes,
                _junc=(rd.get("junction", "-1") != "-1"))


def drivable_at(mp, px, py, reach=30.0):
    """그 점이 **어느 도로든** 주행차로(driving) 위인가.

    `Map.locate` 는 가장 가까운 기준선 **하나**만 보므로 교차로에서 연결로가 겹치면
    다른 연결로의 차로를 못 본다. 여기서는 격자 안 모든 표본을 훑어 하나라도 담으면 참.
    """
    gx, gy = int(px // CELL), int(py // CELL)
    for a in range(gx - 1, gx + 2):
        for b in range(gy - 1, gy + 2):
            for x, y, h, rd, s in mp.grid.get((a, b), ()):
                if math.hypot(px - x, py - y) > reach:
                    continue
                u = (px - x) * math.cos(h) + (py - y) * math.sin(h)
                if abs(u) > 1.5:                      # 표본 간격 2m — 종방향도 맞아야 한다
                    continue
                t = (px - x) * -math.sin(h) + (py - y) * math.cos(h)
                for _lid, typ, lo, hi, _m in mp.lanes(rd, s + u):
                    if typ in DRIVABLE and lo - 1e-6 <= t <= hi + 1e-6:
                        return True
    return False


PHYS_STEP = 0.1       # 물리 폭 탐침 간격[m] — 0.25 면 코스 E 교착점이 4.25 로 나오는데 실제는 4.5 다(0.5m 차이가 탈출 여유를 가른다)
PHYS_MAX = 8.0        # 이보다 멀리는 안 잰다[m]


def phys_room(mp, x, y, hdg, step=PHYS_STEP, maxd=PHYS_MAX):
    """경로점에서 좌/우로 **주행차로가 이어지는 물리적 폭** (pl, pr)[m].

    `l/r/xl/xr` 은 차로 배치(합법 공간)다. 이건 **포장된 차도가 어디까지 있나**다 —
    인도·연석·도로 밖에서 끝난다. 교차로 안에서는 연결로가 한 방향뿐이라 `xl/xr` 이
    언제나 0 인데, 실제로는 반대 방향 연결로가 옆에 있다. 사고현장 우회(2026-09-09 코스 E
    (915,216): 자전거+직각 승용차가 내 차로와 우측 차로를 막아 좌측 대향 연결로만 통로)
    처럼 **죽은 차 앞에서 마지막으로 빠질 곳**을 찾을 때만 쓴다. 합법 판단은 여전히 l/r.
    """
    out = []
    nx, ny = -math.sin(hdg), math.cos(hdg)
    for side in (+1.0, -1.0):
        room, k = 0.0, 1
        while k * step <= maxd + 1e-9:
            d = k * step
            if not drivable_at(mp, x + side * d * nx, y + side * d * ny):
                break
            room, k = d, k + 1
        out.append(round(room, 2))
    return out[0], out[1]


def plan_turn_lanes(mp, route, hdgs, pts, cum, arrows=None):
    """교차로마다 '그 회전을 허용하는 진입 차로'로 미리 옮기는 need 를 깐다.

    허용 차로는 두 군데서 나온다. **노면 화살표가 있으면 그게 최종**이고, 없을 때만
    junction 의 laneLink(물리 연결성)로 판정한다 — 이유는 lane_arrows() 주석 참고.
    """
    arrows = arrows if arrows is not None else {}
    n = len(route)
    # 교차로 구간(pts 가 None 인 연속 덩어리) 찾기
    i = 0
    notes = []
    def unknown(k):
        """교차로 안이거나 아예 모르는 점 — 회전 구간 판정에서 '교차로'로 친다."""
        return pts[k] is None or pts[k].get("_junc")

    while i < n:
        if not unknown(i):
            i += 1
            continue
        j = i
        while j < n and unknown(j):
            j += 1
        a, b = i - 1, j                                  # 진입 직전 / 진출 직후
        i = j
        if a < 0 or unknown(a):
            continue
        # ⚠️ 마지막 회전이 경로 끝에 걸리면 '진출 직후' 점이 없다(실측: tl74 좌회전이
        #    s=854, 경로는 884에서 끝). 그때는 경로의 마지막 방위를 진출 방위로 쓴다.
        b_h = min(b + 3, n - 1)
        dh = math.degrees((hdgs[b_h] - hdgs[max(a - 3, 0)] + math.pi) % (2 * math.pi) - math.pi)
        inc = pts[a]["_road"]
        # ★직진도 검사한다. '좌회전 전용 차로에 있는데 직진'도 똑같이 지정차로 위반이다.
        #   (이 코스에선 직진 교차로 5곳 모두 우리 차로가 직진 허용이라 해당 없지만,
        #    대회 당일 경로가 바뀌면 걸린다 — 2026-08-15 확인)
        want = "left" if dh > TURN_MIN else ("right" if dh < -TURN_MIN else "straight")
        cur = pts[a]["lane"]

        # ── ① 노면 화살표 (있으면 이게 최종) ──────────────────────────────
        mine, others = arrows_near(arrows, pts, cum, a)
        if mine is not None:
            if want in mine:
                continue                                  # 바닥 화살표가 허락한다
            ok = [d for d, mv in others if want in mv]
            if not ok:
                # ⚠️ 편도 1차로면 화살표를 지킬 방법이 애초에 없다. 실측(road 190):
                #    주행차로가 좌우 하나씩인데 바닥엔 좌회전 화살표만 있고, junction 10 은
                #    같은 차로에서 6027(우회전)·4737(좌회전) 둘 다 연결한다 — 맵의 화살표가
                #    빠진 것이지 우리 경로가 틀린 게 아니다. 경고만 남기고 그냥 간다.
                same = [l for l in pts[a]["_lanes"]
                        if l[1] in DRIVABLE and (l[0] > 0) == (cur > 0)]
                why = "편도 1차로 — 지킬 차로가 없다" if len(same) < 2 else "갈 차로가 없다"
                notes.append(f"  ⚠️ 교차로@s={cum[a]:.0f}m {want} — 도로 {inc} 화살표 "
                             f"{sorted(mine)} 인데 {why}")
                continue
            need = min(ok, key=abs)                       # 가장 가까운 허용 차로로
            if lay_need(pts, cum, a, need, notes,
                        f"{want} (화살표 {sorted(mine)} 는 불가)"):
                notes.append(f"  교차로@s={cum[a]:.0f}m {want} — 화살표: 내 차로 "
                             f"{sorted(mine)} 는 불가, 횡이동 {need:+.2f}m")
            continue

        # ── ② 화살표가 없으면 laneLink(물리 연결성)로 ─────────────────────
        allowed = None
        for jc in mp.juncs:
            for c in jc.findall("connection"):
                if c.get("incomingRoad") != inc:
                    continue
                cr = mp.roads.get(c.get("connectingRoad"))
                if cr is None:
                    continue
                try:
                    L = float(cr.get("length"))
                    _, _, h0 = road_point(cr, 0.0)
                    _, _, h1 = road_point(cr, L)
                except Exception:
                    continue
                cdh = math.degrees((h1 - h0 + math.pi) % (2 * math.pi) - math.pi)
                kind = "left" if cdh > TURN_MIN else ("right" if cdh < -TURN_MIN else "straight")
                if kind != want:
                    continue
                # ⚠️ incomingRoad 가 같은 connection 에는 **도로 반대쪽 끝의 진입로**도
                #    섞여 있다. 그걸 다 허용목록에 넣으면, -s 방향(양수 차로)으로 진입할 때
                #    반대편 차로들이 후보가 되어 '12m 횡이동 = 대향차선으로 넘어가라'가
                #    나온다(실측 2026-08-16, 9경유지 코스 s=2435m: 진입차로 3 -> -3).
                #    우리 차로와 **부호가 같은**(= 같은 진행방향) from 만 본다.
                lanes_from = [int(l.get("from")) for l in c.findall("laneLink")
                              if (int(l.get("from")) > 0) == (cur > 0)]
                if lanes_from:
                    allowed = (allowed or []) + lanes_from
        if not allowed:
            continue
        if cur in allowed:
            continue                                      # 이미 맞는 차로
        # 가장 가까운 허용 차로의 중심으로
        tgt = min(allowed, key=lambda x: abs(x - cur))
        lanes = pts[a]["_lanes"]
        tl = next((l for l in lanes if l[0] == tgt), None)
        if tl is None:
            continue
        dt = (tl[2] + tl[3]) / 2.0 - pts[a]["_t"]
        need = dt * pts[a]["_flip"]                       # 주행방향 기준 좌(+)
        if lay_need(pts, cum, a, need, notes,
                    f"{want} (진입차로 {cur} -> {tgt})"):
            notes.append(f"  교차로@s={cum[a]:.0f}m {want} — 진입차로 {cur} -> {tgt} "
                         f"(허용 {sorted(set(allowed))}), 횡이동 {need:+.2f}m")
    return notes


def mark_route_lane_changes(route, pts, cum):
    """**경로 자체에 들어 있는 차선 변경**을 찾아 방향지시등 구간을 표시한다.

    왜 필요한가(2026-08-15 실측): 녹화 경로는 주최측 차량이 실제로 차선을 바꾼 궤적이다
    (s≈95→125 에서 한 차로 왼쪽으로 이동). 우리는 그 선을 그냥 따라가므로 **횡오프셋이 0**
    이고, 그래서 지시등 로직이 '차선 변경 중'인 걸 알지 못해 깜빡이를 안 켰다.

    차로 번호로 판정하면 안 된다 — laneSection 경계에서 번호가 재부여된다
    (실측: 같은 자리에서 lane 2→3→2 인데 차는 똑바로 갔다).
    도로 t 좌표의 실제 이동량으로 본다.
    """
    n = len(route)
    # ★교차로 칸까지의 뒤쪽 거리 — 빠져나온 직후는 판정하지 않으려고 쓴다.
    since = [1e9] * n
    d = 1e9
    for i in range(n):
        q = pts[i]
        if q is not None and q.get("_junc"):
            d = 0.0
        elif i > 0:
            d += cum[i] - cum[i - 1]
        since[i] = d
    for i in range(n):
        p = pts[i]
        # ★★**교차로를 빠져나온 직후는 판정하지 않는다.**
        #   회전을 나오면 **새 도로의 좌표계로 갈아타면서** 도로 t 가 확 움직인다.
        #   차는 그냥 회전을 마무리하고 있을 뿐인데 아래 `dt` 가 그걸 차선변경으로 읽는다.
        #   사용자 지적 2026-08-27(두 번째): "길이 꺾여서 핸들 틀어 가는 데서 왜 좌측
        #   깜빡이를 켜냐". 실측 코스 A — **우회전을 마치자마자 좌측 깜빡이가 켜졌다**:
        #     t=764 (960,-507)  직전 교차로 **2.9m** 뒤 · 그 직전 4초간 방위 -98°(우회전)
        #     t=270 (1099,22)   직전 교차로 **4.3m** 뒤 · 그 직전 4초간 방위 -68°(우회전)
        #     t=576 (1127,-47)  직전 교차로 **1.7m** 뒤 · 방위 -31°(우회전)
        #   교차로 **안**은 이미 제외하고 있었는데(아래 `_junc`) 나온 직후가 안 막혀 있었다.
        if p is None or p.get("_junc") or since[i] < SIG_JUNC_SETTLE:
            # ⚠️ **교차로 안에서는 판정하지 않는다.** 연결로는 기준선이 우리 진로와
            #    다르게 휘어서, 차가 직진해도 도로 t 가 1.6m 씩 움직인다(실측
            #    2026-08-16 road 3195: 방위변화 0° 인데 t -1.61 -> -3.20 -> 우측
            #    깜빡이 10점 오점등). 교차로 회전 지시등은 _upcoming_turn 담당이다.
            continue
        # 앞으로 LOOK[m] 동안 도로 t 가 얼마나 움직이나
        j = i
        while j < n - 1 and cum[j] - cum[i] < SIG_LOOK:
            j += 1
        q = pts[j]
        if q is None or q.get("_junc") or q["_road"] != p["_road"]:
            continue
        dt = q["_t"] - p["_t"]
        if abs(dt) < SIG_MIN:
            continue
        # ★t 이동만으로는 부족하다 — **차로가 실제로 바뀌어야** 진로변경이다.
        #   사용자 지적 2026-08-20: "횡단보도에서 차선 변경". 실측 (1305.5,357)
        #   road 2790: lane 이 -3 으로 계속 같고 need=0.00 인데 t 가 -5.55 -> -4.17 로
        #   움직여 좌측 지시등이 켜졌다. 차로가 하나 늘면서 -3차로 중심이 바깥으로
        #   밀린 것뿐이고, 차는 자기 차로를 그대로 따라간다. 진로변경이 아니다.
        #
        #   반대로 차로 **번호만** 보는 것도 안 된다(laneSection 경계 재부여:
        #   같은 자리에서 2->3->2 인데 직진). 그래서 **둘 다** 만족할 때만 켠다.
        if p.get("lane") is not None and q.get("lane") is not None \
                and q["lane"] == p["lane"]:
            continue
        # ★★그런데 그 둘은 **독립이 아니다.** 차로가 하나 늘거나 줄면 남은 차로의
        #   번호가 재부여되고 중심 t 도 그만큼 밀린다 — 번호도 바뀌고 t 도 움직여서
        #   위 두 조건을 **한꺼번에** 통과한다. 차는 자기 자리를 그대로 가는데.
        #   사용자 지적 2026-08-26: "길이 왼쪽으로 꺾인 2차선 도로인데 왜 깜빡이를 켜".
        #   실측:
        #     코스 A (1266,141)  l 5.24->8.59  r 1.56->2.99   **l+r 6.80 -> 11.58**
        #     코스 G (790,-202)  l 4.41->8.37  r 1.48->2.56   **l+r 5.89 -> 10.93**
        #       둘 다 오른쪽 가장자리와의 거리는 거의 그대로다. 왼쪽으로 도로가
        #       넓어진 것뿐인데 지시등이 켜졌다.
        #     진짜 차선변경 A (1305,357)  l 4.95->2.92  r 1.69->3.75  l+r 6.64 -> 6.67
        #       **폭은 그대로고 좌우 여유가 한 차로만큼 맞바뀐다.**
        #   -> **도로 폭(l+r)이 거의 안 변할 때만** 진로변경으로 본다.
        #   ⚠️ 차로 증감과 진로변경이 같은 자리에서 겹치면 놓친다(드묾). 그때도
        #      우리가 오프셋을 주면 `drive.py` 의 d_off 경로가 지시등을 켠다.
        wid_p = (p.get("l") or 0.0) + (p.get("r") or 0.0)
        wid_q = (q.get("l") or 0.0) + (q.get("r") or 0.0)
        if abs(wid_q - wid_p) >= SIG_MIN:
            continue
        # 주행방향 기준 좌(+)/우(-) 로 환산
        d = 1 if dt * p["_flip"] > 0 else -1
        # ★★**진로변경이 끝날 때까지** 켠다 [시행령 별표2: "그 행위가 끝날 때까지"].
        #   여기서 표시하는 건 '앞 22m 안에 차선변경이 있다'는 **진입 구간**뿐이었다.
        #   그래서 차가 그 구간을 지나면 sig 가 0 이 되는데, **실제 횡이동은 그때부터
        #   한참 더 간다** — 옆으로 옮겨가는 동안 깜빡이가 꺼져 있다.
        #   실측 2026-08-27 코스 B (1460,389):
        #       t=105.8~111.1  도로기준 횡위치 -14.47 -> -11.04 (3.4m 이동)
        #       t=106.0~107.9  깜빡이 ON — **이동의 앞 1/3 뿐**
        #       t=107.9~111.1  꺼진 채로 계속 옮겨감
        #   -> 판정한 구간 `[i, j]` **전체**에 켠다(j = 22m 앞 = 이동이 끝나는 지점).
        #   ⚠️ 교차로 칸과 교차로 직후(SIG_JUNC_SETTLE)는 여기서도 건너뛴다 —
        #      그 구간은 회전 담당이지 진로변경이 아니다.
        # ★시작을 30m 앞당긴다(위 SIG_LEAD 참고). 교차로 칸은 여전히 건너뛴다.
        lead = sig_lead_for(p.get("lim"))
        i0 = i
        while i0 > 0 and cum[i] - cum[i0 - 1] < lead:
            q0 = pts[i0 - 1]
            if q0 is None or q0.get("_junc") or q0["_road"] != p["_road"]:
                break                      # 다른 도로·교차로까지 끌고 가지 않는다
            i0 -= 1
        for k in range(i0, j + 1):
            pk = pts[k]
            if pk is None or pk.get("_junc") or since[k] < SIG_JUNC_SETTLE:
                continue
            if pk["_road"] != p["_road"]:
                break
            pk["sig"] = d


def mark_planner_lane_changes(hints, pts, cum):
    """경로계획기가 **자기가 옮긴다고 적어 둔** 구간에 지시등을 켠다.

    `plan_route.sample_lane` 이 ego_lanes 5번째 칸에 주행기준 좌(+1)/우(-1)/없음(0)
    을 남긴다. 위 `mark_route_lane_changes` 는 지도 기하만 보고 이걸 되짚는데,
    **차로 증감과 차선변경이 같은 자리에서 겹치면 원리상 구분이 안 된다** — 그 위험을
    주석으로 남겨뒀는데 실제로 터졌다.

    실측 2026-08-30 코스 E road 2815 s=58->42 (사용자 지적 "차선변경 깜빡이 안킴"):
      s=58 주행차로 +2[3.27,6.15] +3[6.15,8.86]         우리 t=7.32 (바깥 차로)
      s=42 주행차로 +2[2.02,3.28] +3[3.28,6.27] +4[6.27,8.91]  우리 t=6.09 (한 칸 안쪽)
    안쪽에 차로가 하나 태어나며 바깥 차로가 통째로 재번호돼, **차로 번호는 3 그대로**이고
    **도로 폭은 5.6->8.5m** 로 변했다. 기하 판정의 두 안전장치(번호 변화 요구, 폭 유지
    요구)에 둘 다 걸려 50m 짜리 진짜 차선변경이 지시등 없이 지나갔다 [법 제38조①].

    기하 판정은 그대로 둔다(교차로 진출 정렬 등 계획기가 안 적는 것도 있다).
    여기서는 **덧붙이기만** 한다.
    """
    if not hints or len(hints) != len(pts):
        return
    n = len(pts)
    i = 0
    while i < n:
        h = hints[i]
        d = h[4] if (h and len(h) > 4) else 0        # 옛 4칸 파일은 조용히 건너뛴다
        if not d:
            i += 1
            continue
        j = i
        while j + 1 < n and hints[j + 1] and len(hints[j + 1]) > 4 and hints[j + 1][4] == d:
            j += 1
        # 시작을 앞당긴다 [시행령 별표2 30m · 안내문 항목13 3초].
        # ★★여기서는 **교차로를 넘어서도 끌고 간다.** 기하 판정(mark_route_lane_changes)
        #   은 커브와 차선변경을 구분 못 해서 교차로를 피해야 하지만, 여기는 계획기가
        #   "내가 옮긴다"고 적어 준 구간이라 오점등 위험이 없다.
        #   실측 2026-08-31 코스 A: 차선변경 9곳 중 7곳이 **교차로 직후**에 시작해
        #   선행이 0.0~1.5m 였다(교차로에서 walk-back 이 끊겼다). 대회 기준 3초 미달이다.
        #   ⚠️ 교차로를 지나는 동안 **실제 회전 지시등과 방향이 다를 수 있다.**
        #      그건 drive.py 에서 회전(`_signal_turn`)이 이기게 해 해결한다 —
        #      몸이 도는 방향과 등이 다르면 그게 더 나쁘다.
        lead = sig_lead_for((pts[i] or {}).get("lim"))
        i0 = i
        while i0 > 0 and cum[i] - cum[i0 - 1] < lead and pts[i0 - 1] is not None:
            i0 -= 1
        for k in range(i0, j + 1):
            pk = pts[k]
            if pk is None:
                continue
            if not pk["sig"]:                        # 이미 켜진 건 덮지 않는다
                pk["sig"] = d
        i = j + 1


def main():
    xodr, route_path = sys.argv[1], sys.argv[2]
    out_path = sys.argv[3] if len(sys.argv) > 3 else "lane_plan.json"
    _rj = json.load(open(route_path, encoding="utf-8"))
    route = _rj["ego_route"]
    hints = _rj.get("ego_lanes")           # plan_route 가 남긴 (도로, 차로, s, t)
    hdgs = route_headings(route)
    cum = [0.0]
    for i in range(1, len(route)):
        cum.append(cum[-1] + math.hypot(route[i][0] - route[i-1][0], route[i][1] - route[i-1][1]))

    mp = Map(xodr)
    if hints is not None and len(hints) != len(route):
        print(f"  ⚠️ ego_lanes {len(hints)}개 != 경로 {len(route)}점 — 무시하고 지도에서 찾는다")
        hints = None
    pts = [analyse_point(mp, p[0], p[1], hdgs[i], hints[i] if hints else None)
           for i, p in enumerate(route)]

    # ★경로점별 제한속도 — **도로망을 따라 퍼뜨린** 값을 쓴다(road_speed_limits_net).
    #   경로 순서로 물려주면 같은 도로가 코스마다 다른 제한을 갖는다(그 실측은 함수 주석).
    #   망에서 끊긴 도로에서만 예전처럼 직전 값을 끌고 간다.
    # ★2026-09-05: 도로 단위가 아니라 **(도로, s) 단위**로 본다 — `zone_extent` 주석.
    #   기본 50, 붉은 노면(표시 s 구간 + 거기 붙은 연결로) 위에서만 30 [주최측 답변].
    ext = zone_extent(mp)
    for p in pts:
        if p is None:
            continue
        p["lim"] = round(30 / 3.6 if in_zone(mp, ext, p["_road"], p["_s"],
                                           dir=lane_dir(p.get("lane")),
                                           lead=ZONE_LEAD_IN) else 50 / 3.6, 3)
    # ★물리 폭(pl/pr) — 죽은 차 앞 마지막 탈출용. 합법 공간(l/r/xl/xr)과 별개다(phys_room 주석).
    for i, p in enumerate(pts):
        if p is not None:
            p["pl"], p["pr"] = phys_room(mp, route[i][0], route[i][1], hdgs[i])
    notes = plan_turn_lanes(mp, route, hdgs, pts, cum, lane_arrows(mp))
    mark_route_lane_changes(route, pts, cum)
    mark_planner_lane_changes(hints, pts, cum)

    clean = [None if p is None else {k: v for k, v in p.items() if not k.startswith("_")}
             for p in pts]
    json.dump({"pts": clean}, open(out_path, "w", encoding="utf-8"))

    known = [p for p in clean if p]
    xok = sum(1 for p in known if p["xl"] > 0 or p["xr"] > 0)
    need = sum(1 for p in known if abs(p["need"]) > 0.1)
    sig = sum(1 for p in known if p["sig"])
    print(f"경로 {len(clean)}점 (모름 {len(clean)-len(known)}점 = 교차로/차로밖)")
    print(f"  중앙선 점선(반대차선 진입 가능) 구간: {xok}점 ({xok*100//max(1,len(known))}%)")
    print(f"  지정차로 이동 필요 구간: {need}점")
    print(f"  경로 자체의 차선변경(지시등 필요) 구간: {sig}점")
    for s in notes:
        print(s)
    print(f"  -> {out_path}")


if __name__ == "__main__":
    main()
