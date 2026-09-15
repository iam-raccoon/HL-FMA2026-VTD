#!/usr/bin/env python3
"""출발점 -> 목표점을 **차로 중심선**을 따라 잇는 경로를 xodr 에서 계산한다.

왜 필요한가: 대회 당일 코스가 바뀐다. 연습 주행에서 녹화해두는 건 답이 안 된다
(대회 코스는 또 다르다). 맵은 그대로이므로 **지점 2개만 주면 경로가 나와야** 한다.
신호 정지선 DB(맵 전체 214개)는 경로와 무관하게 재사용된다.

핵심: **차로 단위 그래프**로 탐색한다.
  ⚠️ 처음엔 도로 단위로 짰다가 실패했다(리스폰 81건 · 완주 실패). 이음매마다
     진행 방향을 기하로 추측했는데 교차로에서 연결로가 겹쳐 자꾸 뒤집혔다.
  ★ 추측할 필요가 없었다 — OpenDRIVE 우측통행에서 **차로 번호 부호가 진행 방향**이다.
       lane id < 0  -> 도로 s 가 증가하는 방향으로 주행
       lane id > 0  -> s 가 감소하는 방향으로 주행
     노드를 (road, lane) 로 두면 방향이 부호에서 바로 나오고, 이음매는 추측 없이
     xodr 이 명시한 lane link(<link><successor id>)와 junction 의 laneLink 로만 잇는다.

노드 = (road_id, lane_id, 그 도로에서 이미 한 차선변경 횟수)
간선 = 도로 끝의 <link> (road 면 lane link, junction 이면 connection/laneLink) + 차선변경
비용 = 도로 길이, 차선변경은 **남은 활주로에 반비례**(아래 참고문헌)

참고: Lane-Level Route Planning for Autonomous Vehicles (arXiv:2206.02883)
  차선변경은 '공짜인 결정적 연산'이 아니라 **확률적으로 실패할 수 있는 행동**이라
  MDP 로 모델링해야 하고, 합리적 가정 하에 Dijkstra 유사 방법으로 O(n log n) 에 풀린다.
  우리는 그 취지만 취해 **고정 60m 페널티 -> 활주로 기반 비용**으로 바꿨다:
  같은 차선변경이라도 400m 직선에서 하는 것과 출구 20m 앞에서 하는 것은 위험이 다르다.
  (구조 자체 — 차로를 노드로 두는 라우팅 그래프 — 는 Lanelet2 routing::RoutingGraph 와 같다)

사용: python3 plan_route.py <xodr> --from-csv <주최측경로.csv> [out.json]   ★대회 당일
      python3 plan_route.py <xodr> x1 y1 x2 y2 [x3 y3 ...] [out.json]
        주최측이 **좌/우회전 지점마다 X,Y 를 하나씩** 준다(강의 오후 [103:20]).
        경유지가 몇 개든 순서대로 다 거친다.
      python3 plan_route.py <xodr> --from-scenario <scenario.xml> [out.json]  (연습용)
      python3 plan_route.py <xodr> --like <route.json>      (녹화본과 대조)
★검사: python3 eval/check_route.py <route.json> <xodr>      주행 전에 반드시 돌릴 것

고치면서 걸린 것들 (전부 check_route.py 가 몇 초 만에 잡아줬다 — 주행은 5분 걸린다)
  · 목표 도달 불가  — **차선 변경 간선이 없었다.** road 1928 에 lane -3 으로 도착하는데
                     좌회전 연결로는 -1 에서만 이어진다. 실제 주행은 차선을 바꾼다.
  · 헤딩 66° 반전   — 차선 변경을 계단식으로 붙임 -> smoothstep 보간
  · 3.4m 순간이동   — 차로를 월드좌표로 자유선택 -> 횡위치 t 로 추적(번호 재부여에 강함)
  · 헤딩 77~80° 반전 — 진범은 **차로 폭 계산**이었다. <width> 레코드가 여러 개인데
                     첫 개만 써서 3차식이 유효범위 밖에서 폭발(2.98m 여야 할 폭이 7.71m).
                     build_lane_plan.at_offset() 참고. 한때 '이음매에서 직전 점에 가장
                     가까운 차로로 스냅'해 덮었는데, 그건 그래프가 정한 차로를 지워버려
                     엉뚱한 차로로 달리게 만든다 — 스냅은 번호가 사라졌을 때만 쓴다.
  · 차선변경 누락   — laneSection 이 바뀌면 같은 물리차로의 번호가 바뀐다. thru() 참고.

검증(2026-08-16): 녹화본과 같은 출발/목표로 계획 -> 녹화본 대비 중앙값 0.80m,
  check_route 이상 0곳, **VTD 실주행 완주 141s · 신호위반 0 · 리스폰 0 · 접촉 0 · 속도위반 0**.
  녹화본 주행은 리스폰 1건이었다 — 계획 경로는 좌회전 차로가 경로에 들어 있어 더 낫다.
"""
import csv
import heapq
import json
import math
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from build_tl_map import road_point                              # noqa: E402
from build_lane_plan import Map, CELL, lane_arrows                # noqa: E402

STEP = 1.4
DRIVABLE = {"driving"}
LC_BASE = 60.0      # 활주로가 넉넉할 때 차선변경 1회의 가상 비용[m]
LC_RUNWAY = 45.0    # 차선변경 1회에 필요한 종방향 거리[m]. 30km/h 에서 약 5.4초.
LC_TIGHT_MIN = 0.25  # 활주로 부족 비율의 하한 -> 비용 상한 4배. 막지는 않는다(아래 참고)
LC_MAX = 3          # 한 도로에서 허용할 차선변경 횟수(탐색 폭 제한)
LC_POCKET_MAX = 4   # 차로 소멸+안쪽 회전포켓이 겹친 짧은 구간의 예외 상한
LC_FINISH = 20.0    # 차선변경은 도로 끝을 이만큼 남기고 **끝내 둔다**(끝에서 홱 꺾지 않게)
WP_RADIUS = 8.0     # 경유지 X,Y 에서 이 안에 있는 차로는 모두 '그 경유지'로 친다
#   ★6.0 -> 8.0 (2026-09-06). 주최측 안내 9/3: "경유지는 **특정 차로값으로 제공되는 게
#     아니며**, 좌회전 구간에서 3차로로 제공될 수도 있습니다. 그럴 경우 1차로로 주행하시면
#     됩니다." 차로폭 3.2m 면 3차로 중심에서 1차로 중심까지 **6.4m** 다 — 반경 6.0 으로는
#     안 닿아서 좌회전 차로를 후보에 올리지도 못한다.
#   실측(고치기 전에 재 봤다): 아는 코스 8개를 6 / 8 / 10 으로 각각 다시 계획해 비교.
#     A·D·E·G·H·pretest1·pretest2 = **어긋남 0.00m**(완전히 같은 경로)
#     B 만 한 곳에서 차로를 다르게 골랐다(어긋남 3.05m) — 두 경로 **모두 check_route 이상 0곳**.
#   즉 아는 코스에는 사실상 영향이 없고, 모르는 대회 경유지에 대해서만 폭을 넓힌다.
ARROW_LOOK = 90.0   # 도로 끝에서 이 안의 노면 화살표를 그 진출 차로의 지시로 본다
ARROW_VIOLATION = 500.0  # 화살표가 금지하는 회전의 가상 비용[m]. 금지가 아니라 페널티다
DEBUG_LC = bool(__import__("os").environ.get("PLAN_DEBUG_LC"))
LANE_TRACK_STEP = 0.6  # 차로를 횡위치로 이어 추적할 때 **한 스텝(1.4m)에** 허용할 이동[m].
#   ⚠️ 절대거리(예: '차로폭 절반')로 재면 안 된다. 없어지는 차로는 사라지기 전에
#      테이퍼되며 안쪽으로 밀려서, 옆 차로와의 간격이 이미 좁아져 있다(실측 road 2626:
#      바깥 차로가 7.01 -> 6.03 으로 밀린 뒤 사라졌고 가운데 차로까지가 1.57m 였다).
#      반면 **스텝당** 변화는 정상 테이퍼가 0.04m, 옆 차로로 빼앗기는 순간이 1.57m 로
#      40배 차이가 난다. 0.6 은 그 사이 어디에 둬도 되는 값이다.
JOINT_SNAP = 1.0    # 이음매에서 이보다 벌어지면 블렌딩 대상[m]
JOINT_BLEND = 20.0  # 그 벌어짐을 이 거리에 걸쳐 흡수한다[m]. 3.2m/20m = 헤딩 9°
LANE_PREF = 0.35    # ★지정차로 선호: 바깥 차로 1칸당 주행거리 1m 에 붙는 가상비용.
#   왜 필요한가(실측 2026-08-18): 탐색에 '어느 차로가 옳은가' 기준이 없어서 연결만 되면
#   아무 차로나 골랐고, 검증본(8/14) 대비 **전 구간 한 차로 바깥(3.20m)** 으로 달렸다.
#   그 탓에 v7 연료통을 0.58m 옆으로 스쳐 지나가 VTD 가 무너졌고, 도교법 지정차로도
#   어겼다. 승용차는 왼쪽(안쪽) 차로가 지정차로다.
#   ⚠️ 화살표 위반(500)보다 훨씬 싸게 둔다 — 지정차로 때문에 못 가는 길이 생기면 안 된다.
TURN_MIN = 25.0     # 연결로 방위변화가 이 이상이면 좌/우회전으로 본다

# ★★차선변경은 **횡단보도 위에서 하지 않는다.**
#   사용자 지적 2026-08-28: "횡단보도에서 차선 변경". 실측 코스 A (1306,350) — 차로
#   이동(-3 -> -2)이 s≈808~812 에서 일어나는데 그 자리가 무신호 보호구역 횡단보도에서
#   **3.9m** 다. 지시등은 켜져 있어 [법 제38조] 위반은 아니지만, 보행자를 살펴야 할
#   자리에서 옆으로 옮기는 건 위험하고 사람 눈에도 나쁘다.
#   전 코스 실측: 차선변경 60곳 중 **23곳이 횡단보도 8m 이내**였다(공식 v7 포함).
#   -> 보간 창을 횡단보도 앞에서 끝내거나(①) 지나간 뒤에 시작한다(②).
# ★★여러 칸을 옮길 때는 **한 칸씩 끊어서** 옮긴다. 예전엔 t_blend0->t_target 을
#   smoothstep 하나로 이어 2칸 6m 를 대각선으로 활강했다 — 판례가 과실 100% 를 물린
#   게 정확히 그 대각선형 진로변경이다(한 번에 두 차로 가로지르기). 경찰 공식 입장은
#   '한 차로씩 완결하며 연속으로 바꾸는 것은 단속 근거 없음'이므로, 중간 차로 중심에서
#   LC_SETTLE 만큼 **정착 구간**을 두고 계단식으로 간다. 활주로가 그걸 감당 못 하면
#   (한 칸당 LC_STAGE_MIN 미만) 기존 한 방 보간으로 되돌아간다 — 짧은 활주로에서
#   무리하게 계단을 넣으면 각 계단이 급해져 더 위험하다.
#   실측 2026-08-30 코스 E s=1027~1080: lane 2->4 를 53m 대각 활강(사용자 지적).
LC_SETTLE = 12.0     # 계단 사이 정착 구간[m] — 중간 차로 중심을 이만큼 유지
LC_STAGE_MIN = 25.0  # 계단 하나(한 칸)에 필요한 최소 보간 거리[m]
LC_CW_PAD = 7.0     # 횡단보도 중심에서 이만큼은 떼어 놓는다[m] (표시폭 ~4 + 앞범퍼 여유)
LC_CW_MIN = 30.0    # 그렇게 밀고도 한 차로당 이만큼 활주로가 남아야 민다[m].
#   30m 는 50km/h 에서 2.2초(횡 1.4m/s), 30km/h 에서 3.6초(0.86m/s). 더 줄이면
#   고속 구간에서 홱 꺾는 경로가 된다 — 그때는 차라리 종전대로 둔다.
CW_BY_ROAD = None   # {도로id: [s,...]} — routes/crosswalks.json 에서 읽는다
CW_DODGE = __import__("os").environ.get("PLAN_CW_DODGE", "1") != "0"  # A/B 용 끄개


def load_crosswalks(path=None):
    """횡단보도 위치를 도로/`s` 로 색인. 파일이 없으면 이 규칙만 조용히 꺼진다."""
    global CW_BY_ROAD
    if CW_BY_ROAD is not None:
        return CW_BY_ROAD
    import os
    path = path or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "routes", "crosswalks.json")
    CW_BY_ROAD = {}
    try:
        for c in json.load(open(path, encoding="utf-8"))["crosswalks"]:
            CW_BY_ROAD.setdefault(str(c["road"]), []).append(float(c["s"]))
    except Exception as e:                                   # noqa: BLE001
        print(f"  ⚠️ 횡단보도 DB 를 못 읽었다({e}) — 차선변경 자리 회피는 꺼진다",
              file=sys.stderr)
    return CW_BY_ROAD


def dodge_crosswalks(cws, s_from, s_lim, runway, dirn, n_lc):
    """보간 창 `[s_lim-runway, s_lim-fin]` 이 횡단보도를 물지 않게 민다.

    반환: (새 s_lim, 새 runway). 못 피하면 받은 값을 그대로 돌려준다 —
    **경로가 없어지는 것보다 횡단보도 위 변경이 낫다.**

    ① 첫 횡단보도 **앞에서 끝낸다**(s_lim 을 당긴다). 교차로 직전 횡단보도가 대부분이라
       보통 이쪽이 걸린다 — 미리 자리를 잡고 횡단보도에 들어가니 더 안전하다.
    ② 그럴 활주로가 없으면 마지막 횡단보도 **뒤에서 시작한다**(runway 만 줄인다).
    """
    if not cws or not runway or not CW_DODGE:
        return s_lim, runway
    need = LC_CW_MIN * max(1, abs(n_lc))
    p_lim = (s_lim - s_from) * dirn                  # 진행 방향 기준 거리로 환산
    p_cw = sorted((c - s_from) * dirn for c in cws)
    for _ in range(4):                               # 횡단보도가 여러 개면 되풀이해 좁힌다
        fin = min(LC_FINISH, runway * 0.4)
        lo, hi = p_lim - runway, p_lim - fin
        hit = [c for c in p_cw if lo - LC_CW_PAD < c < hi + LC_CW_PAD]
        if DEBUG_LC:
            print(f"  [CW] s_from={s_from:.1f} 창=[{lo:.1f},{hi:.1f}] "
                  f"횡단보도={[round(c,1) for c in p_cw]} 걸림={[round(c,1) for c in hit]}",
                  file=sys.stderr)
        if not hit:
            break
        a_lim = min(hit) - LC_CW_PAD                 # ① 첫 횡단보도 앞에서 끝내기
        a_run = min(runway, a_lim)                   # 도로 진입점보다 앞설 수는 없다
        b_run = p_lim - (max(hit) + LC_CW_PAD)       # ② 마지막 횡단보도 뒤에서 시작하기
        if DEBUG_LC:
            print(f"       need={need:.0f} ①앞에서끝내기 run={a_run:.1f} "
                  f"②뒤에서시작 run={b_run:.1f}", file=sys.stderr)
        if a_run >= need and a_run >= b_run:
            p_lim, runway = a_lim, a_run
        elif b_run >= need:
            runway = b_run
        else:
            break                                    # 못 피한다 — 종전대로
    return s_from + p_lim * dirn, runway


def travel_dir(lane_id):
    """차로 번호 -> 진행 방향(+1: s 증가, -1: s 감소). 우측통행 규약."""
    return -1 if lane_id > 0 else +1


def lane_change_cost(road_len, k):
    """그 도로에서 k번째 차선변경의 비용[m].

    ★활주로에 반비례한다. 400m 도로에서 한 칸 옮기는 것과 20m 남기고 옮기는 것은
      같은 '차선변경 1회'가 아니다 — 후자는 실제로 못 끝낼 수 있다(arXiv:2206.02883).
      k번을 옮기려면 도로를 k등분해 써야 하므로 1회당 활주로 = road_len / k.

    ⚠️ **막지는 않는다.** 활주로가 모자란다고 간선을 끊으면, 그 길밖에 없는 코스에서
       '경로 없음'이 된다. 비싸게만 만들어 Dijkstra 가 *가능하면* 긴 도로에서
       미리 옮기게 하고, 불가피하면 늦게라도 옮기게 둔다.
    """
    runway = road_len / max(1, k)
    tight = max(LC_TIGHT_MIN, min(1.0, runway / LC_RUNWAY))
    return LC_BASE / tight


class LaneGraph:
    def __init__(self, mp, usable=None, start_s=None):
        self.mp = mp
        # 출발 도로는 중간에서 시작하므로 남은 길이만 활주로다. {road_id: 남은 길이}
        self.usable = usable or {}
        # 출발 도로는 진입 laneSection 도 s0 쪽이다. {road_id: s0}
        self.start_s = start_s or {}
        self.jconn = {}          # incomingRoad -> [(connectingRoad, {from: to})]
        for j in mp.juncs:
            for c in j.findall("connection"):
                inc, cid = c.get("incomingRoad"), c.get("connectingRoad")
                links = {int(l.get("from")): int(l.get("to")) for l in c.findall("laneLink")}
                self.jconn.setdefault(inc, []).append((cid, links))
        self.arrows = lane_arrows(mp)        # 노면 진행방향 화살표
        self._acache = {}
        self._kcache = {}

    def turn_kind(self, cid, to_lane):
        """연결로 cid 를 to_lane 으로 탈 때 **운전자가 겪는** 회전 종류."""
        key = (cid, to_lane > 0)
        if key not in self._kcache:
            rd = self.mp.roads[cid]
            try:
                L = float(rd.get("length"))
                _x, _y, h0 = road_point(rd, 0.0)
                _x, _y, h1 = road_point(rd, L)
            except Exception:
                self._kcache[key] = None
                return None
            dh = math.degrees((h1 - h0 + math.pi) % (2 * math.pi) - math.pi)
            if to_lane > 0:               # 연결로를 -s 로 탄다 -> 겪는 방향이 반대
                dh = -dh
            self._kcache[key] = ("left" if dh > TURN_MIN else
                                 "right" if dh < -TURN_MIN else "straight")
        return self._kcache[key]

    def arrow_moves(self, rid, lex, d):
        """진출 차로 lex 에 그려진 허용 방향. 화살표가 없으면 None(= 제한 없음).

        화살표를 **차로 번호로** 찾지 않는다 — laneSection 마다 번호가 재부여되므로
        (road 128 은 7구간) 화살표가 그려진 지점의 번호와 진출 지점의 번호가 다르다.
        그려진 자리에서 t 로 차로를 찾고, thru() 로 진출 번호까지 걸어와서 맞춘다.
        """
        key = (rid, lex, d)
        if key in self._acache:
            return self._acache[key]
        mv = None
        got = self.arrows.get(rid)
        if got:
            rd = self.mp.roads[rid]
            end = float(rd.get("length")) if d > 0 else 0.0
            for s_a, t_a, m in got:
                if abs(s_a - end) > ARROW_LOOK:
                    continue
                ln = next((l for l in self.mp.lanes(rd, s_a) if l[1] in DRIVABLE
                           and l[2] - 1e-6 <= t_a <= l[3] + 1e-6), None)
                if ln is None or (ln[0] > 0) != (lex > 0):
                    continue
                if self.thru(rid, ln[0], d, s_a) != lex:
                    continue
                mv = (mv or set()) | m
        self._acache[key] = mv
        return mv

    @staticmethod
    def _in_sec(sec, lane_id):
        for side in ("left", "right"):
            for ln in sec.findall(f"{side}/lane"):
                if int(ln.get("id")) == lane_id:
                    return ln
        return None

    def _lane_el(self, rd, lane_id, at_end):
        secs = rd.findall("lanes/laneSection")
        if not secs:
            return None
        sec = max(secs, key=lambda x: float(x.get("s", 0))) if at_end else \
            min(secs, key=lambda x: float(x.get("s", 0)))
        return self._in_sec(sec, lane_id)

    def thru(self, rid, lane_entry, d, from_s=None):
        """진입 laneSection 의 차로 번호 -> **진출 laneSection 에서의 번호**.

        ★같은 도로 안에서도 laneSection 이 바뀌면 번호가 재부여된다. road 128 은
          laneSection 이 7개이고 중간(s=60)에서 차로가 하나 생겨 우리 물리차로가
          **2 -> 3** 이 된다. 노드를 (도로, 번호) 로만 두면 진출 쪽에서 엉뚱한
          차로의 링크를 읽는다 — 실측: v1 코스의 이음매 4곳이 전부 한 차로씩 어긋났고,
          road 1826 에서는 **필요한 차선변경이 통째로 누락**됐다(연결로 1852 는 -1 에만
          붙는데 우리는 물리적으로 -2 에 있었음 -> 이음매에서 3.0m 점프).
          기하는 sample_lane 의 횡위치 추적이 알아서 맞춰줘서 그동안 안 드러났다.

        laneSection 사이는 xodr 이 lane <link> 로 명시한다 — 진행 방향으로 따라간다.
        """
        rd = self.mp.roads[rid]
        secs = sorted(rd.findall("lanes/laneSection"), key=lambda x: float(x.get("s", 0)))
        if from_s is not None:
            # 출발 도로는 중간에서 시작한다 — **진행 방향으로 남은 구간만** 훑는다.
            # ⚠️ 방향을 안 보면(한때 그랬다) -s 로 가는데 s 가 더 큰 구간들을 훑어서,
            #    지나가지도 않을 곳의 링크를 따라가 진출 번호가 틀린다.
            i = max((k for k, x in enumerate(secs)
                     if float(x.get("s", 0)) <= from_s + 1e-6), default=0)
            secs = secs[i:] if d > 0 else secs[:i + 1]
        if d < 0:
            secs = secs[::-1]       # 진행 방향 순서로
        if not secs:
            return None
        # ★진입 구간에 그 번호의 **주행차로**가 없으면 = 도로 중간에서 새로 생기는
        #   포켓 차로다(successors 가 그런 차로로 가는 차선변경 간선을 만들 때
        #   **진출 기준 번호**를 그대로 노드 번호로 쓴다). 이미 진출 기준이라 변환 안 한다.
        #   ⚠️ 'XML 에 그 id 가 있나'로 보면 안 된다. 진입에서는 갓길·경계로 있다가
        #      나중에 주행차로가 되는 번호가 있어서(실측 road 310 의 -2), 링크를 따라가
        #      엉뚱한 번호(-3)를 돌려준다. successors 의 side_lanes 와 **같은 기준**
        #      (type=driving)으로 판정해야 둘이 어긋나지 않는다.
        ln0 = self._in_sec(secs[0], lane_entry)
        if ln0 is None or ln0.get("type") not in DRIVABLE:
            return lane_entry
        tag = "successor" if d > 0 else "predecessor"
        cur = lane_entry
        for sec in secs[:-1]:
            ln = self._in_sec(sec, cur)
            lk = ln.find(f"link/{tag}") if ln is not None else None
            if lk is None:
                return None
            cur = int(lk.get("id"))
        return cur

    def side_lanes(self, rd, s, lid):
        """s 에서 lid 와 **같은 진행방향**인 주행차로 번호들, 안쪽부터."""
        return sorted({l for l, typ, _lo, _hi, _m in self.mp.lanes(rd, s)
                       if typ in DRIVABLE and (l > 0) == (lid > 0)}, key=abs)

    def lane_rank(self, rd, lid):
        """같은 방향 주행차로 중 **안쪽에서 몇 번째**인가(안쪽=0). 지정차로 비용용."""
        d = travel_dir(lid)
        L = float(rd.get("length"))
        s_in = self.start_s.get(rd.get("id"))
        if s_in is None:
            s_in = 1e-3 if d > 0 else L - 1e-3
        same = self.side_lanes(rd, s_in, lid)
        if lid in same:
            return same.index(lid)
        # ★진입 구간에 없는 번호 = 교차로 앞에서 새로 생기는 포켓 차로(successors 참고).
        #   그건 진출 구간 기준으로 세야 한다. 예전엔 0(=최안쪽)으로 떨어져서 우회전
        #   포켓이 '가장 안쪽 차로'로 취급돼 지정차로 비용이 거꾸로 붙었다.
        same_ex = self.side_lanes(rd, (L - 1e-3) if d > 0 else 1e-3, lid)
        return same_ex.index(lid) if lid in same_ex else 0

    def successors(self, node):
        """[(다음 노드, 비용, 차선변경인가)]"""
        rid, lid, k = node[:3]
        # 네 번째 값은 laneSection 재번호 때문에 **진입 차로 번호와 겹치는**
        # 진출 포켓 차로다. 예: road 429 는 진입 +2 가 진출에서 +4 로 이어지는데,
        # 교차로 직전 새 좌회전 포켓도 +2 다. 예전 3튜플 노드로는 둘을 같은 차로로
        # 취급해 포켓에 들어갈 방법이 없었다. override 노드는 진입 물리차로(lid)를
        # 유지하면서, 도로 끝에서는 이 차로(exit_override)에 있다는 뜻이다.
        exit_override = node[3] if len(node) > 3 else None
        rd = self.mp.roads.get(rid)
        if rd is None:
            return []
        d = travel_dir(lid)
        tag = "successor" if d > 0 else "predecessor"     # 진행 방향으로 나가는 쪽
        link = rd.find(f"link/{tag}")
        if link is None:
            return []
        L = float(rd.get("length"))
        cost = L + LANE_PREF * L * self.lane_rank(rd, lid)
        # ★노드의 차로 번호는 **진입 laneSection 기준**이다. 나가는 링크를 읽으려면
        #   진출 laneSection 에서의 번호로 바꿔야 한다(번호 재부여 대응). thru() 참고.
        lex = (exit_override if exit_override is not None else
               self.thru(rid, lid, d, self.start_s.get(rid)))
        # ⚠️ lex 가 None = **그 차로가 도로 끝까지 가지 않는다**(중간에 없어진다).
        #    여기서 끊으면 라우터가 그런 차로를 통째로 회피한다. 그 차로가 유일한
        #    합법 차로일 때 지시위반을 감수하게 된다 — 실측 road 2813: 우회전 합법
        #    차로 +3 이 다음 도로에서 없어져 후속 간선 0개 -> 화살표 {직진} 인 +2 로
        #    우회전(벌점 500). 이 맵의 주행차로 진입 노드 495개 중 32개(6.5%)가 이 경우다.
        # ★한 번 되돌렸다가 **다시 넣었다**(2026-08-19). 처음엔 이 간선을 내주니
        #    WP9·A·B 에 '교차로 안 차선변경'이 새로 생겨(⑧ 0곳 -> 1/1/2곳) 되돌렸는데,
        #    그건 간선 탓이 아니라 **sample_lane 이 죽는 차로에서 못 빠져나온** 탓이었다
        #    (차로가 도로 끝 전에 사라지는데 블렌딩은 도로 끝에 맞춰져 있었다).
        #    그쪽을 고치고(lane_ends_at + lc_by) 간선을 되살렸다.
        #  ⚠️ 이 간선이 **없으면 안 되는** 결정적 증거: 주최측 예시 경로(2026-08-19 공지)가
        #     계산조차 안 됐다. road 30 -> 연결로 550 -> road 72 는 lane -3 으로만
        #     이어지는데 그 차로가 s≈50 에서 사라진다 -> 후속 간선 0개 -> **road 72 에
        #     들어갈 방법이 아예 없다**. 지시위반 몇 곳의 문제가 아니라 '경로 없음'이다.
        out = []
        if lex is None:
            pass                                           # 아래 차선변경 간선만 낸다
        elif link.get("elementType") == "road":
            nid = link.get("elementId")
            ln = self._lane_el(rd, lex, at_end=(d > 0))
            nxt = ln.find(f"link/{tag}") if ln is not None else None
            if nid in self.mp.roads and nxt is not None:
                out.append(((nid, int(nxt.get("id")), 0), cost, False))
        else:                                              # junction
            # ★노면 화살표가 금지하는 회전은 **비싸게** 만든다(끊지는 않는다).
            #   왜 여기서 하는가: 예전엔 계획을 다 짠 뒤 build_lane_plan 이 '옆 차로로
            #   2.85m 비켜라'(need)로 때웠는데, 그러면 VTD 가 **경로이탈로 리스폰**시킨다
            #   (2026-08-16 9경유지 실측: s=644m 화살표 이동 직후 리스폰 2건).
            #   지정차로는 오프셋으로 때울 게 아니라 **경로가 처음부터 옳은 차로로** 가야 한다.
            #   끊지 않는 이유는 차선변경 비용과 같다 — 그 길밖에 없으면 '경로 없음'이 된다.
            allow = self.arrow_moves(rid, lex, d)
            for cid, links in self.jconn.get(rid, ()):
                if lex in links and cid in self.mp.roads:
                    c = cost
                    # ★★**갈래 2개짜리는 교차로가 아니라 모퉁이다** — 화살표를 안 본다.
                    #   주행기(`drive.py` 의 `jx`)·검사기(`check_route.is_real_junction`)와
                    #   **같은 규칙**이다. 셋이 어긋나면 서로 다른 답을 낸다.
                    #   실측 2026-09-04 사전테스트1: junction 91(2814<->2813)은 진입도로
                    #   2개, 연결로도 방향당 1개, 차로 1:1. 반경 60m 에 다른 도로가 없어
                    #   **좌회전이란 게 존재하지 않는다.** 그런데 맵이 거기 좌/직/우
                    #   화살표를 그려 뒀고 연결로 기하가 87° 휘어 `turn_kind` 가 'left' 를
                    #   준다. 그 둘을 맞대면 직진차로(+3)에 500 벌점이 붙어, 라우터가
                    #   굳이 좌회전 포켓(+2)으로 들어갔다가 **연결로 3099 안에서
                    #   -2 -> -3 차선변경**을 하게 된다. 없는 위반을 피하려고 진짜
                    #   교차로 내 차선변경을 만드는 셈이다.
                    if (allow is not None
                            and self.mp.junc_arms.get(
                                self.mp.roads[cid].get("junction", "-1"), 0) >= 3):
                        kind = self.turn_kind(cid, links[lex])
                        if kind is not None and kind not in allow:
                            c += ARROW_VIOLATION
                    out.append(((cid, links[lex], 0), c, False))

        # ★같은 도로 안에서의 차선 변경. 이게 없으면 '그 연결로로 이어지는 차로'에
        #   못 들어가서 목표에 도달 못 한다(실측: road 1928 에 lane -3 으로 도착하는데
        #   좌회전 연결로는 lane -1 에서만 이어져 탐색 실패). 실제 주행은 차선을 바꾼다.
        #   교차로 연결로 안에서는 차선을 바꾸지 않는다(도교법·기하 모두 부적절).
        #   이웃도 **진입 laneSection 기준**으로 센다 — 노드 규약과 맞춰야 한다.
        if rd.get("junction", "-1") == "-1" and k < LC_MAX and exit_override is None:
            s_in = self.start_s.get(rid)
            if s_in is None:
                s_in = 1e-3 if d > 0 else L - 1e-3
            here = set(self.side_lanes(rd, s_in, lid))
            w = lane_change_cost(self.usable.get(rid, L), k + 1)
            for nb in (lid - 1, lid + 1):
                if nb in here and nb != 0:
                    out.append(((rid, nb, k + 1), w, True))
            # ★교차로 **직전에 새로 생기는** 차로(좌/우회전·직진 포켓)도 이웃이다.
            #   진입 laneSection 에만 물어보면 그런 차로는 아예 존재하지 않는 것이 된다
            #   — 그런데 회전차로는 **전부** 그렇게 생긴다. 실측 2026-08-19 road 310:
            #   끝에서 -2(직진 포켓)가 열리고 화살표도 -1={좌회전} / -2={우회전,직진}
            #   인데, -2 로 가는 간선이 없어 라우터가 **좌회전 차로에서 직진**하는
            #   경로를 골랐다(ARROW_VIOLATION 500 을 물고도 그게 유일한 길이었으므로).
            #   WP9 코스에서 이런 지시위반이 3곳 있었다.
            #   ⚠️ 이웃은 번호 뺄셈이 아니라 **진출 구간의 순번**으로 찾는다. 번호는
            #      구간마다 재부여되므로 뺄셈은 엉뚱한 차로를 가리킨다.
            same_ex = self.side_lanes(rd, (L - 1e-3) if d > 0 else 1e-3, lid)
            if lex in same_ex:
                i = same_ex.index(lex)
                for j in (i - 1, i + 1):
                    #   진입에 없는 번호만 추가한다 -> 위 이웃과 겹치지 않고, 번호가
                    #   겹칠 일도 없다(겹치면 진입에 있다는 뜻이라 위에서 이미 처리).
                    if 0 <= j < len(same_ex) and same_ex[j] not in here:
                        out.append(((rid, same_ex[j], k + 1), w, True))

            # ★진입 번호와 **같은 번호로 새로 생기는** 진출 포켓도 별도 상태로 낸다.
            # 위 `not in here`만으로는 번호 충돌 때문에 이 포켓을 영원히 못 고른다.
            # 현재 물리차로가 진출단에서 lex이고, 목표 포켓이 target이면 둘의 실제
            # 횡순위 차이만큼 차선변경해서 `(rid, lid, k+n, target)` 상태로 간다.
            # sample_lane은 이 target이 실제로 나타날 때까지 보간 시작을 미루므로
            # 존재하지 않는 차로로 미리 들어가는 기하도 만들지 않는다.
            # 현재 차로가 진출 전 사라지는 경우도 센다. 그때는 진입단의 살아남는
            # 어느 차로로 먼저 합류한 뒤, 새 포켓으로 가는 최소 변경 횟수를 쓴다.
            # 사전주행 2번 road 418은 바깥차로 소멸과 안쪽 좌회전 포켓 생성이 동시에
            # 일어나 최종 포켓까지 4개 경계를 넘어야 했다(기존 LC_MAX=3이면 400m 우회).
            exits = []                              # (진출 실제차로, 필요한 변경 횟수)
            if lex is not None and lex in same_ex:
                ix = same_ex.index(lex)
                exits = [(target, abs(it - ix)) for it, target in enumerate(same_ex)]
            elif lid in here:
                ih = sorted(here, key=abs).index(lid)
                best = {}
                for jh, nb in enumerate(sorted(here, key=abs)):
                    sx = self.thru(rid, nb, d, self.start_s.get(rid))
                    if sx is None or sx not in same_ex:
                        continue
                    ix = same_ex.index(sx)
                    for it, target in enumerate(same_ex):
                        n = abs(jh - ih) + abs(it - ix)
                        best[target] = min(best.get(target, 999), n)
                exits = list(best.items())
            for target, n in exits:
                if not n or k + n > LC_POCKET_MAX:
                    continue
                # 한 도로에서 여러 칸을 옮기는 비용은 순서대로 누적한다.
                wsum = sum(lane_change_cost(self.usable.get(rid, L), k + j)
                           for j in range(1, n + 1))
                out.append(((rid, lid, k + n, target), wsum, True))

        # 연결로 안에서 현재 차로의 다음 laneLink가 끊겼지만 옆 차로는 이어지는 경우는
        # **강제 합류**로만 살린다. 평소 교차로 차선변경은 계속 금지한다. 사전주행 1번의
        # 3099/-2는 합법 좌회전 포켓에서 들어오지만 도로 끝 successor가 없고, 바로 옆
        # -3만 다음 도로로 이어진다. 이 예외가 없으면 직진 화살표 차로에서 좌회전한다.
        if (rd.get("junction", "-1") != "-1" and exit_override is None
                and k < LC_MAX and not any(v[0][0] != rid for v, _w, _lc in out)):
            s_in = self.start_s.get(rid)
            if s_in is None:
                s_in = 1e-3 if d > 0 else L - 1e-3
            same = self.side_lanes(rd, s_in, lid)

            def has_forward(test_lid):
                test_ex = self.thru(rid, test_lid, d, self.start_s.get(rid))
                if test_ex is None:
                    return False
                if link.get("elementType") == "road":
                    ln = self._lane_el(rd, test_ex, at_end=(d > 0))
                    nxt = ln.find(f"link/{tag}") if ln is not None else None
                    return link.get("elementId") in self.mp.roads and nxt is not None
                return any(test_ex in links and cid in self.mp.roads
                           for cid, links in self.jconn.get(rid, ()))

            if lid in same:
                i0 = same.index(lid)
                for i, nb in enumerate(same):
                    n = abs(i - i0)
                    if not n or k + n > LC_MAX or not has_forward(nb):
                        continue
                    wsum = sum(lane_change_cost(self.usable.get(rid, L), k + j)
                               for j in range(1, n + 1))
                    out.append(((rid, nb, k + n), wsum, True))
        return out

    def search(self, starts, goal_sets):
        """출발 후보들에서 `goal_sets` 를 **순서대로** 다 거치는 최단 경로.

        goal_sets = [{(road,lane), ...}, ...]  각 경유지를 담는 차로 후보 집합.
        (교차로에서 후보가 여럿이라 하나로 못 정한다 — 그래서 집합으로 둔다)

        ★경유지는 **상태에 넣는다**(층 그래프). "구간별로 따로 찾아 이어붙이기"는
          실패한다(실측 2026-08-16): 경유지에 먼저 닿은 차로가 하필 반대 방향이면
          (road 1818 에 lane +1 로 도착) 거기서 다음 경유지로 갈 길이 없어 그냥
          '경로 없음'이 된다. 층을 두면 탐색이 알아서 다른 차로로 도착하는 길을 찾는다.

        노드는 (road, lane, 그 도로에서의 차선변경 횟수). 경유지는 횟수와 무관하다.

        ★비용이 **정확히 같은** 경로가 흔하다 — 차선변경 1회를 어느 도로에서 하든
          거리 합이 같기 때문이다(실측: 두 경로 모두 974.11). 그때 아무거나 고르면
          안 된다. 실제로 힙 순서가 바뀌었다는 이유만으로 600m 를 한 차로 바깥으로
          도는 경로가 튀어나왔고, 그건 VTD 경로 허용오차(1.5m)를 넘어 리스폰을 부른다.
          그래서 동점은 **차선변경을 최대한 늦게**(= 목표에 가깝게) 하는 쪽으로 깬다.
          사람도 내비도 그렇게 한다 — 좌회전 차로에 600m 전부터 들어가 있지 않는다.
          2순위 키라서 활주로 페널티(60->240)를 절대 뒤집지 못한다. 우선순위가 맞다.
        """
        # goal key의 세 번째 값은 '진출단에 새로 생긴 포켓' 여부다. 기존 호출/테스트의
        # (road,lane) 형식도 그대로 받는다.
        goal_sets = [{(x[0], x[1], x[2] if len(x) > 2 else False) for x in g}
                     for g in goal_sets]
        last = len(goal_sets)

        def node_keys(node):
            keys = {(node[0], node[1], False)}
            if len(node) > 3:
                keys.add((node[0], node[3], True))
            return keys

        def adv(node, layer):
            """그 차로가 다음 경유지를 담고 있으면 층을 올린다. 먼저 지나서 손해볼 건 없다."""
            while layer < last and not node_keys(node).isdisjoint(goal_sets[layer]):
                layer += 1
            return layer

        # 키 = (거리, 늦게변경 점수). 뒤 항은 차선변경 지점의 주행거리 합의 부호반전.
        init = (0.0, 0.0)
        dist, prev, pq = {}, {}, []
        for r, l in starts:
            st = ((r, l, 0), adv((r, l, 0), 0))
            dist[st] = init; pq.append((init, st))
        heapq.heapify(pq)
        hit = None
        while pq:
            key, st = heapq.heappop(pq)
            u, layer = st
            if layer == last:
                hit = st
                break
            if key > dist.get(st, (1e18, 0.0)):
                continue
            dcur, late = key
            for v, w, is_lc in self.successors(u):
                sv = (v, adv(v, layer))
                nk = (dcur + w, late - dcur if is_lc else late)
                if nk < dist.get(sv, (1e18, 0.0)):
                    dist[sv] = nk; prev[sv] = st
                    heapq.heappush(pq, (nk, sv))
        if hit is None:
            return None
        path, st = [hit[0]], hit
        while st in prev:
            st = prev[st]
            if st[0] != path[-1]:                  # 같은 차로에서 층만 오른 건 한 번만
                path.append(st[0])
        return path[::-1]


def locate_lane(mp, px, py):
    """그 점을 실제로 담고 있는 주행차로 (road_id, lane_id, s).

    ⚠️ 교차로는 연결로가 서로 겹쳐서 '가장 가까운 기준선' 하나로 고르면 엉뚱한 도로를
       잡는다(실측: 목표가 1955 위인데 1946 으로 잡혀 179m 를 더 갔다). 후보를 다 본다.
    """
    gx, gy = int(px // CELL), int(py // CELL)
    cand = []
    for a in range(gx - 2, gx + 3):
        for b in range(gy - 2, gy + 3):
            cand.extend(mp.grid.get((a, b), ()))
    best = None
    for x, y, h, rd, s in cand:
        if math.hypot(px - x, py - y) > 40.0:
            continue
        u = (px - x) * math.cos(h) + (py - y) * math.sin(h)   # 종방향도 봐야 한다
        if abs(u) > 1.5:
            continue
        t = (px - x) * -math.sin(h) + (py - y) * math.cos(h)
        for lid, typ, lo, hi, _m in mp.lanes(rd, s):
            if typ not in DRIVABLE or not (lo - 1e-6 <= t <= hi + 1e-6):
                continue
            off = abs(t - (lo + hi) / 2.0)
            if best is None or off < best[0]:
                best = (off, rd.get("id"), lid, s + u)
    if best is None:
        raise SystemExit(f"({px:.1f},{py:.1f}) 를 주행차로에서 못 찾음")
    return best[1], best[2], best[3]


def locate_lanes_all(mp, px, py):
    """그 점을 담는 **모든** 주행차로 [(road, lane, s)] — 차로중앙에 가까운 순.

    ⚠️ 교차로는 연결로가 겹쳐서 후보가 여럿이다. 하나만 골라 목표로 삼으면
       '그 연결로로는 갈 수 없어' 경로가 없다고 나온다(실측: 목표를 1964 로 집어
       탐색 실패). 전부 후보로 두고 먼저 닿는 것을 쓴다.
    """
    gx, gy = int(px // CELL), int(py // CELL)
    cand = []
    for a in range(gx - 2, gx + 3):
        for b in range(gy - 2, gy + 3):
            cand.extend(mp.grid.get((a, b), ()))
    found = {}
    for x, y, h, rd, s in cand:
        if math.hypot(px - x, py - y) > 40.0:
            continue
        # ⚠️ **종방향도 봐야 한다.** 횡방향만 보면 직선 도로에서 30m 떨어진 표본도
        #    'lo <= t <= hi' 를 통과해, 그 표본의 s 가 답으로 잡힌다. 실측 2026-08-16:
        #    경로가 첫 경유지에서 **24m 떨어진 곳에서 시작**하고 종점은 28m 모자랐다.
        #    격자 표본이 2m 간격이므로 가장 가까운 표본은 |u| <= 1m 이다.
        u = (px - x) * math.cos(h) + (py - y) * math.sin(h)
        if abs(u) > 1.5:
            continue
        t = (px - x) * -math.sin(h) + (py - y) * math.cos(h)
        for lid, typ, lo, hi, _m in mp.lanes(rd, s):
            if typ not in DRIVABLE or not (lo - 1e-6 <= t <= hi + 1e-6):
                continue
            key = (rd.get("id"), lid)
            off = abs(t - (lo + hi) / 2.0)
            if key not in found or off < found[key][0]:
                found[key] = (off, s + u)      # 표본 s 가 아니라 **실제 투영 s**
    if not found:
        # 출발/종점이 차로 밖으로 살짝 빗나간 경우(사람이 불러준 좌표). 가장 가까운
        # 차로 하나로 붙인다 — 여기서 여러 개를 허용하면 옆 차로에서 출발해버린다.
        near = wp_lanes(mp, px, py)
        print(f"  ⚠️ ({px:.1f},{py:.1f}) 가 차로 안이 아님 -> 가장 가까운 {near[0][0]}/{near[0][1]:+d} 로 붙임")
        return near[:1]
    return [(r, l, v[1]) for (r, l), v in sorted(found.items(), key=lambda kv: kv[1][0])]


def wp_lanes(mp, px, py, radius=WP_RADIUS):
    """경유지 하나가 허용하는 **모든** 주행차로 [(road, lane, s)] — 중심선이 radius 안.

    ★차로도 방향도 못박으면 안 된다. 주최측은 좌/우회전 지점마다 X,Y 를 하나 찍어줄
      뿐이고(강의 [103:20] "이거에 대한 좌표값 X, Y 그거를 저희가 드릴 겁니다"),
      도착 판정도 "포인트 반경으로 10m, 20m 안"이다([103:56]).
      그런데 왕복도로에서 점 하나를 '담는' 차로는 **한쪽 방향뿐**이다. 그걸 강제하면
      반대 방향으로 접근하려고 한 바퀴 돈다 — 실측: 직선 108m 구간이 695m 가 되고
      9경유지 코스 전체가 2km -> 5.4km 로 부풀었다.
    ⚠️ 반대로 너무 크게 잡으면 옆 골목까지 후보가 된다. 교차로에서 교차하는 도로가
       후보에 들어오는 건 무해하다(어차피 그 회전이 경로다).
    """
    gx, gy = int(px // CELL), int(py // CELL)
    span = int(radius // CELL) + 2
    found = {}
    for a in range(gx - span, gx + span + 1):
        for b in range(gy - span, gy + span + 1):
            for x, y, h, rd, s in mp.grid.get((a, b), ()):
                if math.hypot(px - x, py - y) > radius + 20.0:
                    continue
                for lid, typ, lo, hi, _m in mp.lanes(rd, s):
                    if typ not in DRIVABLE:
                        continue
                    t = (lo + hi) / 2.0
                    d = math.hypot(px - (x - math.sin(h) * t), py - (y + math.cos(h) * t))
                    if d > radius:
                        continue
                    key = (rd.get("id"), lid)
                    if key not in found or d < found[key][0]:
                        found[key] = (d, s)
    if not found:
        # ⚠️ 대회 당일 좌표는 사람이 화면에서 읽어 불러준다([103:20]). 조금 빗나갔다고
        #    그냥 죽으면 안 된다 — 반경을 넓혀 보고, 넓혔다는 사실을 반드시 알린다.
        if radius < 25.0:
            print(f"  ⚠️ ({px:.1f},{py:.1f}) 반경 {radius:.0f}m 에 차로 없음 -> {radius*2:.0f}m 로 넓힘")
            return wp_lanes(mp, px, py, radius * 2)
        raise SystemExit(f"({px:.1f},{py:.1f}) 근처 {radius:.0f}m 안에 주행차로가 없다 — 좌표를 확인할 것")
    return [(r, l, v[1]) for (r, l), v in sorted(found.items(), key=lambda kv: kv[1][0])]


def lc_blend(run_len, rem):
    """차선변경 보간 진행도 0..1. run_len = 시작 시점의 남은 거리, rem = 지금 남은 거리.

    ★도로 끝이 아니라 **LC_FINISH 만큼 남기고** 1 이 된다. 예전엔 rem=0 에서 완료되도록
      돼 있어서, 차로가 없어지는 바로 그 지점에서야 옮기는 경로가 나왔다(실측 road 2003:
      마지막 2m 를 도로 끝 9.8m 남기고 이동). 실주행에서 "차선 없어진다고 미리 적혀
      있는데 끝에 가서야 바꾼다"로 보였다. 지금은 최소 20m 전에 자리를 잡는다.

    ⚠️ 활주로가 짧으면 여유도 그만큼만 뗀다. 그냥 LC_FINISH 를 빼면 분모가 0 에 가까워져
       한 스텝에 0 -> 1 로 튄다(실측: v7 경로 s=3509m 에서 헤딩 64° 반전, check_route
       '쓰면 안 된다' 판정). run_len 의 40% 를 상한으로 둔다.
    """
    fin = min(LC_FINISH, run_len * 0.4)
    u = min(1.0, max(0.0, (run_len - rem) / max(run_len - fin, 1e-3)))
    return u * u * (3 - 2 * u)                              # smoothstep


def lc_blend_staged(run_len, rem, fracs):
    """여러 칸 차선변경의 계단식 진행도 0..1.

    fracs = 중간 차로들이 전체 횡이동에서 차지하는 비율(0 과 1 사이, 오름차순).
            예: 2칸 옮기고 중간 차로가 정확히 절반이면 [0.5].
    각 계단은 smoothstep, 계단 사이엔 LC_SETTLE 만큼 평평한 정착 구간.
    활주로가 부족하면(계단당 LC_STAGE_MIN 미만) 호출부가 애초에 fracs=None 으로
    부르므로 여기선 나눗셈만 신경 쓴다.
    """
    fin = min(LC_FINISH, run_len * 0.4)
    win = max(run_len - fin, 1e-3)
    u = min(1.0, max(0.0, (run_len - rem) / win))
    n = len(fracs) + 1                              # 계단 수
    settle_u = min(LC_SETTLE / win, 0.4 / max(1, n - 1))
    ramp_u = (1.0 - settle_u * (n - 1)) / n
    lo = 0.0
    pts = [0.0] + list(fracs) + [1.0]
    for k in range(n):
        a, b = lo, lo + ramp_u
        if u <= b or k == n - 1:
            t = min(1.0, max(0.0, (u - a) / max(ramp_u, 1e-6)))
            t = t * t * (3 - 2 * t)                 # smoothstep
            return pts[k] + (pts[k + 1] - pts[k]) * t
        if u <= b + settle_u:
            return pts[k + 1]                       # 정착: 중간 차로 중심 유지
        lo = b + settle_u
    return 1.0


def lane_ends_at(mp, rid, lid, s_from, s_to):
    """그 차로를 s_from 부터 훑을 때 **어디서 없어지는가**. 끝까지 살아있으면 None.

    차로를 번호가 아니라 **횡위치 t** 로 따라간다(번호는 laneSection 마다 재부여된다).
    한 스텝에 LANE_TRACK_STEP 넘게 움직여야 이어붙일 수 있으면 '없어졌다'로 본다 —
    sample_lane 의 추적 규칙과 같은 기준이라 둘이 어긋나지 않는다.

    ★왜 필요한가: 없어지는 차로에서 빠져나오는 차선변경은 **차로가 살아있는 동안** 끝나야
      한다. 도로 끝에 맞춰 블렌딩하면 이미 사라진 차로 위에서 옮기는 그림이 나온다.
    """
    rd = mp.roads[rid]
    L = float(rd.get("length"))
    s_from = max(0.0, min(L, s_from)); s_to = max(0.0, min(L, s_to))
    step = STEP if s_to >= s_from else -STEP
    want_pos = lid > 0
    cur_t, s = None, s_from
    while (s <= s_to) if step > 0 else (s >= s_to):
        here = [(lo + hi) / 2.0 for l2, typ, lo, hi, _m in mp.lanes(rd, s)
                if typ in DRIVABLE and (l2 > 0) == want_pos]
        if cur_t is None:
            cur_t = next(((lo + hi) / 2.0 for l2, typ, lo, hi, _m in mp.lanes(rd, s)
                          if typ in DRIVABLE and l2 == lid), None)
            if cur_t is None:
                return None                  # 시작부터 없다 -> 여기서 판단하지 않는다
        elif not here:
            return s
        else:
            near = min(here, key=lambda t: abs(t - cur_t))
            if abs(near - cur_t) > LANE_TRACK_STEP:
                return s                     # 이어붙일 게 없다 = 이 차로는 여기서 끝
            cur_t = near
        s += step
    return None


def sample_lane(mp, rid, lid, s_from, s_to, out, n_lc=0, meta=None, lc_by=None):
    """한 차로를 s_from -> s_to 로 훑으며 중심선 점을 찍는다.

    ⚠️ laneSection 이 바뀌면 같은 물리 차로의 번호가 재부여된다(실측: 2->3->2 인데
       차는 똑바로 감). 번호를 고집하지 말고 **직전 점에 가장 가까운 같은 차선군** 차로를
       따라간다. 차선군(부호)은 진행 방향이라 바뀌면 안 되므로 그건 고정한다.

    n_lc = 이 도로에서 옮길 **부호 있는 칸수**(+ 바깥쪽 / − 안쪽). 번호가 아니다.
      ⚠️ 예전엔 진출 기준 차로 **번호**(lane_out)를 받아 블렌딩 목표로 삼았다. 그런데
         블렌딩이 시작되는 지점은 아직 **진입 번호 체계**라, 그 번호가 전혀 다른 차로를
         가리킨다. 실측 2026-08-19 road 128(laneSection 7개, s≈60 에서 안쪽에 차로가
         하나 생겨 번호가 한 칸씩 밀림): 진출기준 lane_out=2 를 진입 구간에서 찾으니
         **하필 지금 내가 달리는 차로**(t=4.95)라 차선변경이 통째로 no-op 이 됐다.
         그 결과 바깥 차로에 남은 채 교차로까지 가서, 교차로 안에서 이음매 블렌딩이
         3.2m 를 끌어당겼다 — 화면에는 '사거리 한복판에서 이유 없는 차선변경'.
    """
    rd = mp.roads[rid]
    L = float(rd.get("length"))
    s_from = max(0.0, min(L, s_from)); s_to = max(0.0, min(L, s_to))
    step = STEP if s_to >= s_from else -STEP
    want_pos = lid > 0
    cur_t = None            # 지금 따라가는 차로의 횡위치. **월드 좌표가 아니라 t 로 추적**한다
    t_blend0 = t_target = None
    run_len = 0.0
    # ★이음매 흡수: 계획 차로가 직전 점에서 멀면 **툭 튀지 말고 몇 미터에 걸쳐** 붙는다.
    #   번호를 그냥 따르면 128->3346 에서 한 점이 3.2m 튀며 헤딩 74° 가 꺾였고,
    #   반대로 근접 스냅으로 막으면 틀린 차로를 끝까지 따라간다(v7 붕괴의 원인).
    #   튐 자체는 '틀린 차로 -> 옳은 차로' 보정이므로 **없애지 말고 펴야** 한다.
    joint_err = joint_left = 0.0
    lc_fracs = None         # 여러 칸 차선변경의 중간 차로 비율(계단식 보간용)
    # t 증가 방향이 주행기준 좌인가 우인가. lane id 부호가 진행방향을 가른다
    # (analyse_point 의 _flip 과 같은 규약: 양수 차로면 t 증가 = 우측).
    flip = -1.0 if want_pos else +1.0
    # ★활주로는 옮길 차로 수에 비례한다. 2칸을 45m 안에 우겨넣으면 7m 를 5초에 옮기는
    #   경로가 나온다 — 비용 모델(lane_change_cost)이 말하는 것과 같은 이야기다.
    # ★차선변경을 **끝내야 하는 지점**. 보통은 도로 끝(s_to)이지만, 지금 차로가 중간에
    #   없어지면 그 전에 끝나야 한다(호출부가 lane_ends_at 으로 찾아 넘겨준다).
    s_lim = s_to if lc_by is None else lc_by
    runway = min(abs(s_lim - s_from), abs(n_lc) * LC_RUNWAY + LC_FINISH) if n_lc else 0.0
    # ★보간 창을 횡단보도 밖으로 민다(위 dodge_crosswalks 참고).
    if n_lc:
        s_lim, runway = dodge_crosswalks(load_crosswalks().get(str(rid)),
                                         s_from, s_lim, runway,
                                         1 if step > 0 else -1, n_lc)
    s = s_from
    while (s <= s_to) if step > 0 else (s >= s_to):
        try:
            x, y, h = road_point(rd, s)
        except Exception:
            break
        here = [(l2, (lo + hi) / 2.0) for l2, typ, lo, hi, _m in mp.lanes(rd, s)
                if typ in DRIVABLE and (l2 > 0) == want_pos]
        if not here:
            s += step
            continue

        if cur_t is None:
            # ★도로 이음매: 번호가 가리키는 차로가 직전 위치와 안 맞을 수 있다
            #   (실측: 128->3346 이음매에서 3.1m 튀며 헤딩 80° 반전).
            #   직전 점이 있으면 **거기에 가장 가까운** 차로로 붙는다.
            # ★계획이 정한 차로 번호를 **먼저** 쓴다. 예전엔 직전 점에 가장 가까운
            #   차로로 붙였는데(이음매 3.1m 튐 대응), 그러면 한 번 어긋난 순간
            #   **끝까지 그 차로를 따라간다**. 실측 2026-08-18: 검증본 대비 전 구간
            #   한 차로 바깥(3.20m)으로 달렸고, 탐색 비용을 고쳐 차로 선택을 바로잡아도
            #   기하는 그대로였다(계획 -1 인데 그려지는 건 -2). v7 연료통을 0.58m 로
            #   스치게 만든 것이 이것이다.
            #   번호가 이 s 에 없을 때만(진출 laneSection 재부여 등) 직전 점 기준으로 붙는다.
            def _near():
                return (min((t for _l, t in here),
                            key=lambda t: math.dist((x - math.sin(h) * t,
                                                     y + math.cos(h) * t), out[-1]))
                        if out else min(here, key=lambda z: abs(z[1]))[1])
            cur_t = next((t for l2, t in here if l2 == lid), None)
            if cur_t is None:
                cur_t = _near()
            elif out:
                t_prev = ((out[-1][0] - x) * -math.sin(h)
                          + (out[-1][1] - y) * math.cos(h))
                if abs(t_prev - cur_t) > JOINT_SNAP:
                    joint_err, joint_left = t_prev - cur_t, JOINT_BLEND
            # ⚠️ 여기에 '직전 점에서 JOINT_JUMP 넘게 튀면 근접 스냅' 검산을 넣어봤다가
            #    **되돌렸다**(2026-08-18). 헤딩 튐(74°, 1점)은 사라졌지만 ⑦ 기준경로
            #    이격이 0.44m -> 3.21m 로 **차로가 도로 밀렸다**. 이음매의 3.2m 튐은
            #    사실 '잘못된 차로 -> 올바른 차로' 보정이라, 그걸 막으면 틀린 차로에
            #    머문다. 차로가 맞는 쪽이 훨씬 중요하다(v7 붕괴의 원인이 그것이었다).
            #    남은 1점 지그재그는 미해결 — README '알려진 흠' 참고.
        else:
            # laneSection 이 바뀌면 번호가 재부여된다 -> 번호가 아니라 **횡위치**로 이어간다
            cur_t = min((t for _l, t in here), key=lambda t: abs(t - cur_t))

        t = cur_t
        # ★차선 변경은 부드럽게. 툭 갈아타면 그 지점에서 헤딩이 66° 튀어
        #   따라갈 수 없는 경로가 된다(실측 2026-08-16).
        # 남은 거리는 **끝내야 하는 지점**까지다. 지나쳤으면 0(블렌딩은 이미 끝났다).
        rem = max(0.0, (s_lim - s) if step > 0 else (s - s_lim))
        if n_lc and rem < runway:
            if t_blend0 is None:
                # ★목표는 **번호가 아니라 자리**로 잡는다 — 안쪽부터 세어 n_lc 칸.
                #   번호로 잡으면 이 구간의 번호 체계로 읽혀 엉뚱한 차로를 가리킨다.
                # ⚠️ 줄 세우는 키는 **차로 번호의 크기**다. `abs(t)` 로 세우면 안 된다 —
                #    기준선이 차도 한가운데인 도로가 있어서 같은 방향 차로의 t 가
                #    -1.5/+1.5/+4.5/+7.5 처럼 나온다. 맨 안쪽 두 개의 |t| 가 같아
                #    순서가 입력 순서로 정해지고 **한 칸이 통째로 날아간다**
                #    (실측 2026-08-19 road 2626: 2칸 지시인데 1칸만 옮겨 우회전
                #    포켓에 못 들어갔고, 남은 3m 를 교차로 연결로 안에서 흡수했다).
                #    호출부가 칸수를 셀 때 쓰는 side_lanes 와 **같은 기준**이어야 한다.
                same = [tt for _l, tt in sorted(here, key=lambda z: abs(z[0]))]
                i0 = min(range(len(same)), key=lambda k: abs(same[k] - cur_t))
                j = i0 + n_lc
                if DEBUG_LC:
                    print(f"  [LC] rid={rid} s={s:.1f} cur_t={cur_t:+.2f} n_lc={n_lc:+d} "
                          f"i0={i0} j={j} same={[round(v,2) for v in same]}", file=sys.stderr)
                if 0 <= j < len(same):
                    # ⚠️ 옮길 자리가 이 s 에 아직 없을 수 있다. 그때는 시작을 미루고,
                    #    **실제로 시작한 지점의 남은 거리**를 활주로로 쓴다.
                    #    안 그러면 u 가 0 이 아니라 0.5 에서 시작해 그 점에서 튄다.
                    t_blend0, t_target, run_len = cur_t, same[j], max(rem, 1e-3)
                    # ★여러 칸이면 중간 차로 중심 비율을 기억해 계단식으로 간다
                    #   (위 LC_SETTLE 주석). 활주로가 모자라면 기존 한 방 보간.
                    lc_fracs = None
                    n_stage = abs(j - i0)
                    span = same[j] - same[i0]
                    fin0 = min(LC_FINISH, run_len * 0.4)
                    if (n_stage >= 2 and abs(span) > 1e-6
                            and run_len - fin0 >= n_stage * LC_STAGE_MIN
                                                 + (n_stage - 1) * LC_SETTLE):
                        stepd = 1 if j > i0 else -1
                        lc_fracs = [(same[k] - same[i0]) / span
                                    for k in range(i0 + stepd, j, stepd)]
            else:
                # 목표 차로도 laneSection 재부여에 강하게 — 번호가 아니라 횡위치로 추적.
                # ⚠️ 다만 **아무 차로에나 다시 붙이면 안 된다.** 옮겨 갈 차로가 중간에
                #    잠깐 끊기는 도로가 있다(실측 road 2626: s=69~64 에서 바깥 차로가
                #    통째로 사라졌다 다시 나타난다). 그때 '가장 가까운 것'을 집으면
                #    목표가 **가운데 차로에 붙잡혀 영영 안 돌아온다** — 2칸 지시가
                #    1칸이 되고, 남은 한 칸은 교차로 연결로 안에서 흡수됐다.
                #    같은 차로로 볼 수 있을 때(한 차로 폭 안)만 갱신하고, 아니면 둔다.
                near = min((tt for _l, tt in here), key=lambda tt: abs(tt - t_target))
                if abs(near - t_target) <= LANE_TRACK_STEP:
                    if DEBUG_LC and abs(near - t_target) > 0.3:
                        print(f"  [LC] rid={rid} s={s:.1f} 목표 {t_target:+.2f} -> {near:+.2f}",
                              file=sys.stderr)
                    t_target = near
                elif DEBUG_LC:
                    print(f"  [LC] rid={rid} s={s:.1f} 목표 {t_target:+.2f} 유지"
                          f" (가장 가까운 게 {near:+.2f}, 너무 멀다)", file=sys.stderr)
            if t_blend0 is not None:
                prog = (lc_blend_staged(run_len, rem, lc_fracs)
                        if lc_fracs else lc_blend(run_len, rem))
                t = t_blend0 + (t_target - t_blend0) * prog

        if joint_left > 0.0:                       # 이음매 벌어짐을 서서히 흡수
            t += joint_err * (joint_left / JOINT_BLEND)
            joint_left = max(0.0, joint_left - abs(step))
        out.append((x - math.sin(h) * t, y + math.cos(h) * t))
        if meta is not None:
            # ★그 점이 **어느 도로 어느 차로의 어디**인지를 같이 남긴다.
            #   지도에서 다시 찾으면 교차로에서 연결로가 겹쳐 못 고른다 —
            #   여기선 이미 알고 있으니 넘겨주면 된다(build_lane_plan 가 쓴다).
            here_id = min(here, key=lambda z: abs(z[1] - t))[0]
            # ★★**차선변경 중인가**(주행기준 좌 +1 / 우 -1, 아니면 0).
            #   왜 여기서 남기나: build_lane_plan 은 지도 기하만 보고 이걸 되짚는데,
            #   **차로 증감과 차선변경이 같은 자리에서 겹치면 원리상 구분이 안 된다.**
            #   실측 2026-08-30 코스 E road 2815 s=58->42 (사용자 지적):
            #     s=58 주행차로 +2[3.27,6.15] +3[6.15,8.86]  우리 t=7.32 (바깥 차로)
            #     s=42 주행차로 +2[2.02,3.28] +3[3.28,6.27] +4[6.27,8.91]  우리 t=6.09
            #   안쪽에 차로가 하나 태어나면서 바깥 차로들이 통째로 재번호된다. 그래서
            #   **차로 번호는 3 그대로**이고 **도로 폭은 2.9m 변한다** — build_lane_plan 의
            #   두 안전장치(번호 변화 요구, 폭 유지 요구)에 **둘 다** 걸려 지시등이 안 켜졌다.
            #   경로를 만든 쪽은 자기가 옮기는 걸 알고 있다. 추측하게 두지 말고 적어 준다.
            lc = 0
            if t_blend0 is not None and t_target is not None:
                moved = abs(t - t_blend0)
                span = abs(t_target - t_blend0)
                if span > 1e-6 and moved < span - 1e-6:      # 아직 옮기는 중
                    lc = 1 if (t_target - t_blend0) * flip > 0 else -1
            meta.append((rid, here_id, round(s, 3), round(t, 3), lc))
        s += step


def entry_lane(mp, rid, lane_at_s, s_at, d):
    """`s_at` 에서의 차로 번호 -> **그 도로 진입 laneSection 기준** 번호. `thru()` 의 역이다.

    ⚠️⚠️ 왜 필요한가(실측 2026-08-19, 주최측 예시 경로): 탐색 노드의 차로 번호는
       **진입 구간 기준**인데, 좌표로 찾은 번호는 **그 s 기준**이다. laneSection 이
       많은 도로에서는 둘이 다르다 — road 72 는 구간이 9개고 s=200 에서 안쪽에 차로가
       생겨 번호가 통째로 밀린다:
           s=0      -3(t=-7.5) -2(-4.5) -1(-1.5)
           s=230.6  -4(t=-7.5) -3(-4.5) -2(-1.5) -1(+1.6)
       주최측 경유지 4번은 t=-7.5 자리라 '진입 -3 / 그 지점 -4' 다. 목표를 -4 로 잡으면
       **그런 노드가 아예 없어서 '경로 없음'** 이 된다. 실제로 주최측 예시 경로가
       우리 플래너에서 계산이 안 됐다. 당일 좌표는 어디든 떨어질 수 있으므로 치명적이다.
    """
    rd = mp.roads[rid]
    secs = sorted(rd.findall("lanes/laneSection"), key=lambda x: float(x.get("s", 0)))
    if not secs:
        return lane_at_s
    i = max((k for k, x in enumerate(secs) if float(x.get("s", 0)) <= s_at + 1e-6), default=0)
    back = secs[:i + 1][::-1] if d > 0 else secs[i:]     # 진행 방향의 **반대**로 거슬러
    tag = "predecessor" if d > 0 else "successor"
    cur = lane_at_s
    for sec in back[:-1]:
        ln = LaneGraph._in_sec(sec, cur)
        lk = ln.find(f"link/{tag}") if ln is not None else None
        if lk is None:
            return None                     # 그 차로는 도로 진입까지 이어지지 않는다
        cur = int(lk.get("id"))
    return cur


def exit_lane(mp, rid, lane_at_s, s_at, d):
    """`s_at` 에서의 차로 번호 -> **그 도로 진출 laneSection 기준** 번호(`thru` 와 같은 규약)."""
    rd = mp.roads[rid]
    secs = sorted(rd.findall("lanes/laneSection"), key=lambda x: float(x.get("s", 0)))
    if not secs:
        return lane_at_s
    i = max((k for k, x in enumerate(secs) if float(x.get("s", 0)) <= s_at + 1e-6), default=0)
    fwd = secs[i:] if d > 0 else secs[:i + 1][::-1]
    tag = "successor" if d > 0 else "predecessor"
    cur = lane_at_s
    for sec in fwd[:-1]:
        ln = LaneGraph._in_sec(sec, cur)
        lk = ln.find(f"link/{tag}") if ln is not None else None
        if lk is None:
            return None
        cur = int(lk.get("id"))
    return cur


def as_entry_numbers(mp, cands):
    """[(road, 그 s 의 차로번호, s)] -> **탐색 노드가 쓰는 번호**로.

    노드 규약과 맞춰야 `adv()` 의 경유지 매칭이 성립한다. 규약이 두 가지다:
      · 도로 진입까지 이어지는 차로 -> **진입 laneSection 기준 번호**
      · 도로 중간에서 생기는 포켓 차로 -> **진출 기준 번호**(successors 가 그렇게 만든다)
    ⚠️ 포켓을 그냥 버리면 안 된다. 실측 2026-08-19 주최측 예시 경로의 4번 지점이
       바로 그 자리였다 — road 72 의 t=-7.5 차로는 s=0 에 있다가 s=50 에서 사라지고
       s=200 에서 다시 생긴다. 버리면 그 경유지에 도달할 방법이 없어져 '경로 없음'이다.
    """
    out, seen = [], set()
    for rid, lid, s in cands:
        d = travel_dir(lid)
        e = entry_lane(mp, rid, lid, s, d)
        pocket = e is None
        if pocket:                          # 진입까지 안 이어진다 -> 포켓. 진출 번호로.
            e = exit_lane(mp, rid, lid, s, d)
        if e is None or (rid, e, pocket) in seen:
            continue
        seen.add((rid, e, pocket))
        out.append((rid, e, s, pocket))
    return out


def from_csv(path):
    """★주최측이 당일 주는 경로 CSV -> [(x,y), ...] (seq 순서).

    형식(2026-08-19 공지 "주행 경로 형식 및 객체 데이터 기준 안내"):
        seq,x,y            VTD 월드 직교좌표계, m. egoX/egoY 와 **같은 좌표계**라 변환 불필요.
        첫 지점 = 출발, 마지막 지점 = 종료.
        그 사이는 **두 개씩 짝** — 각 교차로의 진입·진출 지점이다.
        (seq 2·3 = 교차로1, 4·5 = 교차로2, ...). 짝 사이 구간이 교차로 내부.
        경로는 최단거리 기준으로 선정돼 있고, **seq 순서대로 통과**해야 한다.
        지정 경로를 이탈하면 감점.

    ★우리 탐색은 '주어진 점들을 순서대로 다 거치는 최단 경로'라 이 형식과 정확히 맞는다.
      교차로 진입·진출을 **둘 다** 주므로 어느 연결로를 타야 하는지가 좌표로 못박힌다 —
      화살표/차로 선택의 모호함이 그만큼 줄어든다.
    """
    pts = []
    with open(path, encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            pts.append((float(row["x"]), float(row["y"])))
    if len(pts) < 2:
        raise SystemExit(f"{path}: 지점이 2개 미만이다")
    n_mid = len(pts) - 2
    if n_mid % 2:
        print(f"  ⚠️ 중간 지점이 {n_mid}개(홀수) — 공지 형식은 교차로마다 진입·진출 **짝**이다. "
              f"파일을 확인할 것(계산은 그대로 진행한다)", file=sys.stderr)
    else:
        print(f"  경로 CSV: 지점 {len(pts)}개 = 출발 + 교차로 {n_mid // 2}개(진입·진출) + 종료", file=sys.stderr)
    return pts


def start_dir_ok(mp, rid, lid, s, heading):
    """그 차로를 `heading`[rad] 방향으로 달릴 수 있나(±90°)."""
    _x, _y, h = road_point(mp.roads[rid], s)
    fwd = h if travel_dir(lid) > 0 else h + math.pi   # 그 차로의 실제 진행 방위
    return math.cos(heading - fwd) > 0.0


def plan(mp, *pts, heading=None):
    """지점 2개 이상을 순서대로 지나는 경로. 중간 웨이포인트는 **반드시** 거친다.

    heading[rad] 을 주면 **출발 차로를 그 방향으로 달릴 수 있는 것만** 쓴다.
      ⚠️⚠️ 이게 없어서 새 코스 A 가 통째로 어긋났다(실측 2026-08-19). 주최측이 준
        출발 X,Y 를 담는 차로가 왕복도로에서 **한쪽 방향뿐**일 수 있는데, VTD 는
        코스 진행 방향에 맞춰 **반대 차로에**(2.96m 옆) 차를 놓는다. 그러면
          ego 스폰 헤딩 76.0°  vs  우리 경로 시작 접선 -106.1°  = **178° 반대**
        가 되고, 경유지 직선합 1098m 짜리 코스에 5278m 경로가 나왔다. 실주행에서는
        차가 앞으로 가버려 제어기가 경로 뒤쪽(인덱스 1733)으로 재획득했고 **앞
        2.4km 를 통째로 안 갔다**(커버리지 56%).
      ★대회 당일 절차: 시나리오를 로드한 **뒤** ego 헤딩을 읽어(`vtd/ego_pose.py`)
        그 값을 넣고 경로를 만든다. 좌표만 보고 만들면 방향을 찍는 셈이다.

    ⚠️ 구간마다 따로 계산해 기하를 이어붙이면 안 된다(실측 2026-08-16, 9-웨이포인트 코스):
       이음매에서 **59.6m 벌어지고 169° 유턴**이 생겼다. 웨이포인트는 도로 중간 점이라
       탐색을 새로 시작하면 그 지점에 **반대 방향 차로로 도착/출발**할 수 있기 때문이다.
       ★이어야 할 것은 기하가 아니라 **그래프 노드**다 — 도착한 (도로,차로)에서 그대로
         다음 구간을 시작하면 방향과 횡위치가 자동으로 이어진다. 샘플링은 마지막에 한 번.
    """
    # ★출발만 차로를 **고정**한다. 차는 이미 그 차로에 놓여 있다(시나리오 StartLane).
    #   여기까지 반경으로 열어주면 계획이 옆 차로에서 시작해버려, VTD 가 경로 이탈로
    #   보고 리스폰을 건다(실측: 검증본 대비 중앙값 3.3m = 한 차로 통째로 밀림).
    #   중간 경유지와 종점은 반대로 열어야 한다 — 주최측도 "반경 10~20m"로 본다.
    starts = locate_lanes_all(mp, *pts[0])
    # ★실측 헤딩(--heading, ego 에서 읽은 값)은 **못박는다**. 다음 지점에서 추정한 건
    #   어디까지나 힌트라, 맞는 차로가 없으면 경고만 하고 원래 후보로 진행한다.
    #   ⚠️ 힌트로도 죽게 했다가 코스 D·E·G 가 '경로 없음'이 됐다 — 직선 방위는 굽은
    #      도로나 교차로 건너 지점에서 실제 차로 방향과 크게 어긋날 수 있다.
    strict = heading is not None
    if heading is None and len(pts) > 1:
        # ★출발 방향은 **다음 지점 쪽**이다. 주최측 형식이 확정되면서(2026-08-19 공지)
        #   첫 지점=출발, 그다음이 첫 교차로 진입점이므로 그 방위가 곧 진행 방향이다.
        #   이게 없으면 왕복도로에서 반대 차로를 잡는다(코스 A 에서 178° 반대로 잡았다).
        #   ⚠️ 어디까지나 **직선 방위**다. 실제 스폰 헤딩이 있으면(--heading) 그걸 쓴다.
        heading = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])
        print(f"  출발 방향을 다음 지점에서 추정: {math.degrees(heading):.1f}° "
              f"(--heading 으로 덮어쓸 수 있다)", file=sys.stderr)
    if heading is not None:
        ok = [c for c in starts if start_dir_ok(mp, c[0], c[1], c[2], heading)]
        if not ok:
            # 그 점을 담는 차로가 전부 반대 방향이다 = **차가 옆 차로에 서 있다**
            # (왕복도로에서 흔하다. VTD 는 코스 방향에 맞춰 반대 차로에 놓는다).
            # 반경을 열어 같은 방향으로 달릴 수 있는 가장 가까운 차로를 잡는다.
            ok = [c for c in wp_lanes(mp, *pts[0], radius=6.0)
                  if start_dir_ok(mp, c[0], c[1], c[2], heading)][:1]
            if ok:
                print(f"  ⚠️ 출발 좌표를 담는 차로가 진행방향과 반대다 -> "
                      f"{ok[0][0]}/{ok[0][1]:+d} 로 옮겨 잡음(ego 헤딩 "
                      f"{math.degrees(heading):.0f}°)", file=sys.stderr)
        if not ok:
            if strict:
                raise SystemExit(f"출발점에서 헤딩 {math.degrees(heading):.0f}° 로 "
                                 f"달릴 수 있는 차로가 없다 — 좌표/헤딩을 확인할 것")
            print(f"  ⚠️ 추정 방향 {math.degrees(heading):.0f}° 로 달릴 수 있는 출발 차로가 "
                  f"없다 — 추정을 버리고 좌표가 담긴 차로로 진행한다. "
                  f"**실제 스폰 헤딩을 --heading 으로 주는 게 안전하다**", file=sys.stderr)
            ok = starts
        starts = ok
    # ⚠️ 경유지·종점 후보는 **진입 기준 번호**로 바꿔서 넣는다(entry_lane 주석 참고).
    #    출발점만 예외다 — 출발 도로는 s0 부터 훑으므로 그 s 의 번호가 곧 진입 번호이고,
    #    start_s/thru(from_s=s0) 가 전부 그 규약으로 돌아간다.
    cand = [starts] \
        + [as_entry_numbers(mp, wp_lanes(mp, x, y)) for x, y in pts[1:-1]] \
        + ([as_entry_numbers(mp, locate_lanes_all(mp, *pts[-1]))] if len(pts) > 1 else [])
    for i, c in enumerate(cand):
        if not c:
            raise SystemExit(f"경유지 {i + 1}번 {pts[i]} 에 도달 가능한 차로가 없다")
    # ★★**종점은 차로를 못박지 않는다 — 못박으면 계획 자체가 실패한다.**
    #   예전엔 출발점과 종점을 같이 고정했는데, 그 근거(중앙값 3.3m 밀림, `StartLane`)는
    #   **출발점 얘기**였다. 종점엔 아무도 차를 놓지 않고, 주최측 완주 판정도
    #   "뒷바퀴 축이 종료 지점을 통과"(반경 10~20m)다.
    #   실측 2026-09-03 사전테스트 경로 2 (`route_pretest_2.csv`):
    #     종점 (349.4,-102.8) 를 담는 차로는 `589/-1` **하나뿐** -> 9번 경유지(road 418)
    #     에서 그리로 갈 수 없어 **'경유지 10개를 순서대로 잇는 차로 경로 없음'** 으로
    #     통째로 실패했다. 반경 6m 면 후보가 9개고 그중 `418/-1` 로 붙는다.
    #   주최측도 2026-09-03 안내에서 못박았다: "경유지의 좌표는 **대략적인 위치**로
    #   주어집니다", "제공드린 경유지를 **무조건 밟아야하는건 아닙니다**".
    #   ⚠️ 그래도 **엄격 판정을 먼저** 쓴다 — 지금 도는 코스들은 다 그걸로 계획되고,
    #      느슨하게 열면 종점 차로가 바뀔 수 있다. 실패할 때만 반경으로 넓힌다.
    goal_relaxed = False
    if len(pts) > 1:
        strict_goal = cand[-1]
        wide_goal = as_entry_numbers(mp, wp_lanes(mp, *pts[-1]))
        goal_tries = [strict_goal]
        if set(map(tuple, wide_goal)) - set(map(tuple, strict_goal)):
            goal_tries.append(wide_goal)
    else:
        goal_tries = [cand[-1]] if len(cand) > 1 else []
    # 출발 도로는 중간에서 시작한다 — 활주로는 도로 전체가 아니라 **남은 만큼**이다.
    usable, start_s = {}, {}
    for r, l, s in cand[0]:
        L = float(mp.roads[r].get("length"))
        rest = (L - s) if travel_dir(l) > 0 else s
        usable[r] = max(usable.get(r, 0.0), rest)
        start_s[r] = s

    g = LaneGraph(mp, usable, start_s)
    path = None
    for _t, goal in enumerate(goal_tries or [None]):
        if goal is not None:
            cand[-1] = goal
        path = g.search(
            [(r, l) for r, l, _ in cand[0]],
            [[(x[0], x[1], x[3] if len(x) > 3 else False) for x in c]
             for c in cand[1:]])
        if path is not None:
            if _t > 0:
                goal_relaxed = True
                print(f"  ⚠️ 종점 차로를 못박으면 경로가 없어 **반경 {WP_RADIUS:.0f}m 로 넓혔다** "
                      f"— 주최측 좌표는 대략적이고 완주는 뒷바퀴축 통과로 본다"
                      f"(후보 {len(goal)}개)", file=sys.stderr)
            break
    if path is None:
        raise SystemExit(f"경유지 {len(cand)}개를 순서대로 잇는 차로 경로 없음")
    s0 = {(r, l): s for r, l, s in cand[0]}[path[0][:2]]
    last_keys = {(path[-1][0], path[-1][1], False)}
    if len(path[-1]) > 3:
        last_keys.add((path[-1][0], path[-1][3], True))
    s1 = next(x[2] for x in cand[-1]
              if (x[0], x[1], x[3] if len(x) > 3 else False) in last_keys)
    # 같은 도로가 연속으로 나오면(=차선 변경) 한 구간으로 합친다.
    segs = []                                   # (road, 진입차로, 진출차로)
    for node in path:
        rid, lid, _k = node[:3]
        target = node[3] if len(node) > 3 else lid
        target_is_exit = len(node) > 3
        if segs and segs[-1][0] == rid:
            segs[-1][2] = target
            segs[-1][3] = target_is_exit
        else:
            segs.append([rid, lid, target, target_is_exit])

    out, meta = [], []
    for k, (rid, l_in, l_out, l_out_is_exit) in enumerate(segs):
        L = float(mp.roads[rid].get("length"))
        d = travel_dir(l_in)
        a = s0 if k == 0 else (0.0 if d > 0 else L)
        b = s1 if k == len(segs) - 1 else (L if d > 0 else 0.0)
        # 차선변경은 **부호 있는 칸수**로 넘긴다(+ 바깥 / − 안쪽). 번호로 넘기면
        # 블렌딩 시작 지점의 번호 체계로 읽혀 엉뚱한 차로를 가리킨다(sample_lane 주석).
        #  ⚠️ 칸수도 번호 뺄셈(lo_ex - li_ex)으로 세면 안 된다. 주행차로 사이에 주행이
        #     아닌 차로가 끼면 번호 간격 ≠ 칸 간격이다. **진출 구간의 주행차로만**
        #     안쪽부터 줄 세워 그 순번 차이를 쓴다.
        fs = start_s.get(rid) if k == 0 else None
        li_ex = g.thru(rid, l_in, d, fs)
        lo_ex = (l_out if l_out_is_exit else
                 (g.thru(rid, l_out, d, fs) if l_out != l_in else li_ex))
        def rank_step(s_ref, a_ref, b_ref):
            """진입/진출 어느 한쪽 구간에서 a_ref -> b_ref 가 **몇 칸**인가(+바깥/−안쪽).

            ⚠️ 번호 뺄셈으로 세면 안 된다. 주행차로 사이에 주행이 아닌 차로가 끼면
               번호 간격 ≠ 칸 간격이다. 그 구간의 **주행차로만** 안쪽부터 줄 세운다.
            """
            same = sorted({l for l, typ, _lo, _hi, _m in mp.lanes(mp.roads[rid], s_ref)
                           if typ in DRIVABLE and (l > 0) == (a_ref > 0)}, key=abs)
            if a_ref in same and b_ref in same:
                return same.index(b_ref) - same.index(a_ref), same
            return 0, same

        s_in = fs if fs is not None else (1e-3 if d > 0 else L - 1e-3)
        # ★지금 차로가 도로 끝 전에 없어지면(li_ex is None) 구간을 **두 토막**으로 나눈다.
        #   ① 차로가 살아있는 동안 옆 차로로 피한다  ② 거기서 진출 차로까지
        #   ⚠️ 한 번에 처리하려 했다가 실패했다(실측 2026-08-19 주최측 예시 경로):
        #      road 72 는 `-3 -> -2[lc1] -> -4[lc2]` 로 **한 도로에서 두 번** 옮겨야 하는데
        #      (죽는 바깥 차로에서 안으로 피했다가, 끝에서 다시 바깥 포켓으로),
        #      sample_lane 은 도로당 블렌딩 창이 하나뿐이라 둘 다 놓치고 3m 를 툭 튀었다
        #      — 헤딩 65° 스파이크. 게다가 진입기준(-3,-2)과 진출기준(-4)이 한 구간에
        #      섞여 칸수 계산도 0 이 나왔다.
        changed = l_out_is_exit or l_out != l_in
        if changed and li_ex is None:
            die = lane_ends_at(mp, rid, l_in, a, b)
            _n0, same_in = rank_step(s_in, l_in, l_in)
            i0 = same_in.index(l_in) if l_in in same_in else -1
            nb = (same_in[i0 - 1] if i0 > 0 else
                  (same_in[i0 + 1] if 0 <= i0 + 1 < len(same_in) else None))
            if die is not None and nb is not None:
                mid = die - LC_FINISH * (1 if d > 0 else -1)
                mid = max(min(mid, max(a, b)), min(a, b))
                n0 = -1 if i0 > 0 else +1
                # 명시적인 진출 포켓으로 가는 경우에는 '죽는 차로의 바로 옆'만
                # 고집하지 않는다. 소멸 전/후 활주로에 변경을 고르게 나눠 급조향을
                # 줄인다. 일반적인 차로 소멸 경로는 기존 한 칸 합류를 그대로 둔다.
                if l_out_is_exit:
                    choices = []
                    for cand_nb in same_in:
                        if cand_nb == l_in:
                            continue
                        cand_ex = g.thru(rid, cand_nb, d, fs)
                        if cand_ex is None or lo_ex is None:
                            continue
                        cn0, _ = rank_step(s_in, l_in, cand_nb)
                        cn2, _ = rank_step((L - 1e-3) if d > 0 else 1e-3,
                                           cand_ex, lo_ex)
                        if abs(cn0) + abs(cn2) > LC_POCKET_MAX:
                            continue
                        r0 = max(abs(mid - a), 1.0)
                        r2 = max(abs(b - mid), 1.0)
                        choices.append((max(abs(cn0) / r0, abs(cn2) / r2),
                                        abs(cn0) + abs(cn2), cand_nb, cn0))
                    if choices:
                        _score, _total, nb, n0 = min(choices)
                sample_lane(mp, rid, l_in, a, mid, out, n0, meta, mid)
                lo2 = g.thru(rid, nb, d, None)
                n2 = 0
                if lo2 is not None and lo_ex is not None:
                    n2, _s = rank_step((L - 1e-3) if d > 0 else 1e-3, lo2, lo_ex)
                sample_lane(mp, rid, nb, mid, b, out, n2, meta)
                continue
        n, lc_by = 0, None
        if changed:
            if li_ex is not None and lo_ex is not None:
                n, _s = rank_step((L - 1e-3) if d > 0 else 1e-3, li_ex, lo_ex)
            else:
                n, _s = rank_step(s_in, l_in, l_out)
                die = lane_ends_at(mp, rid, l_in, a, b)
                if die is not None:
                    lc_by = die - LC_FINISH * (1 if d > 0 else -1)
        sample_lane(mp, rid, l_in, a, b, out, n, meta, lc_by)

    # 이음매에서 같은 점이 두 번 찍힐 수 있다 -> 너무 가까운 점 제거(meta 도 같이)
    dedup, dmeta = ([out[0]], [meta[0]]) if out else ([], [])
    for q, m in zip(out[1:], meta[1:]):
        if math.dist(q, dedup[-1]) >= 0.2:
            dedup.append(q); dmeta.append(m)
    return dedup, path, dmeta


def from_scenario(mp, xml_path, player="Ego"):
    """시나리오 XML 에서 출발/목표를 뽑는다 — 사람이 좌표를 찾아 넣을 필요 없게.

    XML 이 주는 것(실측 2026-08-16):
        <Path><Waypoint TrackId=".." s=".."/> ... </Path>      경유지
        <PathRef StartS=".." TargetS=".." StartLane="2"/>       경로상 시작/끝, 차로
    ⚠️ 웨이포인트 사이의 도로 순서는 안 준다(PathOption="shortest" 로 알아서 가라는 뜻).
       그래서 경로 계산은 우리가 한다 — XML 은 **거쳐야 할 점들**만 준다.
    ★ **웨이포인트가 2개라는 보장이 없다.** v1~v7 은 2개지만 기본 시나리오
       HL_FMA_VTD_LivingLab.xml 은 **9개**다. 첫점·끝점만 쓰면 북쪽 구간이 통째로
       빠져 819m 가 나온다(실제 코스는 2084m). 전부 순서대로 거쳐야 한다.
    ⚠️ StartLane 은 waypoint1 에서만 유효하다. 나머지 차로는 그때그때 다르므로
       '그 지점을 담는 주행차로 아무거나'로 두고 탐색에 맡긴다.
    ⚠️ v1~v7 은 TargetS - StartS 가 waypoint1->waypoint2 거리와 거의 같다(838.6 vs 838.4).
       즉 path 는 마지막 웨이포인트에서 끝난다. 녹화본이 그보다 42m 더 간 것은
       EndAction="continue" 로 교차로를 더 지나간 구간이다.
    """
    import xml.etree.ElementTree as ET
    root = ET.parse(xml_path).getroot()

    # ★**그 플레이어의 Path 만** 쓴다. 시나리오에 Path 가 여러 개일 수 있다.
    #   ⚠️ 예전엔 root.iter("Path") 를 통째로 이어붙였다. v1~v6 은 Path 가 하나뿐이라
    #      멀쩡했는데 **v7 은 2개**다 — Path01(Ego: 128·1928·1926) + Path02(AI 차량
    #      New Player01: 1602·2011·2004). 그래서 에고를 **남의 차 경로까지 순회시키는**
    #      6경유지 3818m 짜리 가짜 코스가 나왔다(실제 에고 코스는 3경유지).
    #      Ego 의 <PathRef PathId="1"> 이 어느 Path 인지 명시하고 있었는데 안 봤다.
    lane0 = path_id = None
    for pl in root.iter("Player"):
        d = pl.find("Description")
        if d is not None and d.get("Name") == player:
            pr = pl.find("Init/PathRef") or pl.find(".//PathRef")
            if pr is not None:
                if pr.get("StartLane"):
                    lane0 = int(pr.get("StartLane"))
                path_id = pr.get("PathId")
            ps = pl.find(".//PathShapeRef")
            if path_id is None and ps is not None:
                path_id = ps.get("PathShapeId")
            break

    paths = [pa for pa in root.iter("Path")]
    mine = [pa for pa in paths if path_id is not None and pa.get("PathId") == path_id]
    if not mine:
        if len(paths) > 1:
            print(f"  ⚠️ Path 가 {len(paths)}개인데 {player} 의 PathRef 를 못 찾았다 "
                  f"— 전부 이어붙인다(경로가 엉뚱할 수 있음)")
        mine = paths
    wps = [(w.get("TrackId"), float(w.get("s")))
           for pa in mine for w in pa.findall("Waypoint")]
    if len(wps) < 2:
        raise SystemExit(f"{xml_path}: Waypoint 가 2개 미만이라 출발/목표를 못 뽑는다")

    def world(track, s_at, want_lane=None):
        rd = mp.roads.get(track)
        if rd is None:
            raise SystemExit(f"road {track} 없음")
        x, y, h = road_point(rd, s_at)
        cands = [(lid, (lo + hi) / 2.0) for lid, typ, lo, hi, _m in mp.lanes(rd, s_at)
                 if typ in DRIVABLE]
        if not cands:
            raise SystemExit(f"road {track} s={s_at} 에 주행차로 없음")
        pick = next((t for lid, t in cands if want_lane is not None and abs(lid) == abs(want_lane)),
                    min((t for _l, t in cands), key=abs))
        return (x - math.sin(h) * pick, y + math.cos(h) * pick)

    out = [world(t, s, lane0 if i == 0 else None) for i, (t, s) in enumerate(wps)]
    print(f"XML에서 추출: 웨이포인트 {len(out)}개 [StartLane={lane0}]")
    for (t, s), (x, y) in zip(wps, out):
        print(f"    road {t:>6} s={s:7.2f}  ->  ({x:8.1f},{y:9.1f})")
    return out


def save(pts, path, meta, out_path):
    total = sum(math.dist(pts[i], pts[i-1]) for i in range(1, len(pts)))
    json.dump({"name": "planned", "ego_start": [pts[0][0], pts[0][1], 0.0],
               "ego_goal": list(pts[-1]), "speed_limit": 8.33,
               "duration": max(400, int(total / 4)),
               "ego_route": [[round(x, 3), round(y, 3)] for x, y in pts],
               # ★경로점별 (도로, 차로, s, t). 교차로 안에서도 차로계획을 채우려면 필요하다.
               "ego_lanes": [[r, l, sv, tv, lc] for r, l, sv, tv, lc in meta]},
              open(out_path, "w", encoding="utf-8"), ensure_ascii=False)
    print(f"차로 {len(path)}개  경로 {len(pts)}점 {total:.0f}m -> {out_path}")


def main():
    xodr = sys.argv[1]
    mp = Map(xodr)
    # ★--heading <도> : ego 가 실제로 바라보는 방향. 출발 차로를 이 방향으로 달릴 수
    #   있는 것만 쓴다. 대회 당일엔 **반드시** 준다(plan() 주석 참고).
    heading = None
    if "--heading" in sys.argv:
        i = sys.argv.index("--heading")
        heading = math.radians(float(sys.argv[i + 1]))
        del sys.argv[i:i + 2]
    if sys.argv[2] == "--from-csv":
        # ★대회 당일: 주최측 CSV 를 그대로 먹인다.
        wps = from_csv(sys.argv[3])
        out_path = sys.argv[4] if len(sys.argv) > 4 else "route_planned.json"
        save(*plan(mp, *wps, heading=heading), out_path)
        return
    if sys.argv[2] == "--from-scenario":
        wps = from_scenario(mp, sys.argv[3])
        out_path = sys.argv[4] if len(sys.argv) > 4 else "route_planned.json"
        save(*plan(mp, *wps, heading=heading), out_path)
        return
    if sys.argv[2] == "--like":
        rt = json.load(open(sys.argv[3], encoding="utf-8"))["ego_route"]
        pts, path, _m = plan(mp, tuple(rt[0]), tuple(rt[-1]), heading=heading)
        print("차로 경로: " + " -> ".join(
            f"{n[0]}/{n[1]:+d}" + (f"=>{n[3]:+d}" if len(n) > 3 else "")
            for n in path))
        print(f"계획 {len(pts)}점 / 녹화 {len(rt)}점")
        dev = sorted(min(math.dist((px, py), q) for q in rt) for px, py in pts)
        print(f"녹화본과의 편차: 중앙값 {dev[len(dev)//2]:.2f}m  "
              f"90%tile {dev[int(len(dev)*0.9)]:.2f}m  최대 {dev[-1]:.2f}m")
        return
    # 좌표를 짝수개 나열하면 그 순서대로 다 거친다(경유지 지정). 마지막 인자는 출력 파일.
    nums, out_path = sys.argv[2:], "route_planned.json"
    if len(nums) % 2:
        out_path = nums.pop()
    v = list(map(float, nums))
    save(*plan(mp, *[(v[i], v[i+1]) for i in range(0, len(v), 2)], heading=heading),
         out_path)


if __name__ == "__main__":
    main()
