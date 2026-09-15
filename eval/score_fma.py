#!/usr/bin/env python3
"""**대회 안내문 그대로**의 채점기 — 실주행 CSV 한 판을 15개 항목으로 훑는다.

왜 따로 만드나
--------------
`eval/evaluate.py` 는 오프라인 합성 시나리오용이고(직선·가상 좌표), `check_lanes.py`
`check_crosswalks.py` `check_run.py` 는 각각 한 측면만 본다. 대회는 **구간마다 100점**
에서 시작해 **같은 항목을 구간당 한 번만** 깎는 구조라, 흩어진 결과를 눈으로 합치면
매번 다르게 센다. 실제로 그렇게 놓친 게 있다 — 코스 E (1454,940) 의 교차로 내 2.5초
정지는 로그를 눈으로 볼 땐 안 보였고, `check_lanes` 를 **다시 돌려서** 나왔다.

[대회 안내문 2026-08-27]
  · 경로가 여러 구간으로 나뉘고 **구간마다 100점**에서 시작한다.
  · 같은 항목은 **구간당 1회만** 감점. 경미 -3 / 중대 -6.
    경미로 깎인 뒤 같은 항목이 중대로 심화되면 **+3 을 더** 깎는다(합 -6).
  · 구간 점수는 마이너스까지 내려가고 총점에서 그대로 차감된다.
  · 순위 = 완주 그룹 우선 -> 총점 -> 완주 시간 -> 중대 위반 횟수.

⚠️ **구간 경계는 안내문에 없다.** `--sections N` 으로 경로 거리를 N 등분하는 게
   기본값이고, 그건 **추정**이다. 9/3~4 사전테스트에서 채점 UI 로 실제 구간 수를
   확인하면 그 값을 쓸 것. 구간이 많을수록 총점이 낮아진다(항목당 1회 제한이 구간마다
   새로 열리므로) — 그래서 이 값을 모르면 **총점은 비교용이지 예측이 아니다.**

사용
----
    python3 eval/score_fma.py <run.csv> --route routes/HL_FMA_NEW_E.json \\
            --lane routes/HL_FMA_NEW_E_lane.json --xodr <map.xodr> [--sections 4]

`--xodr` 를 빼면 지도가 필요한 항목(3·4·5·6)은 '판정 안 함' 으로 남는다(지도 로딩이
몇 분 걸리므로 빠르게 볼 때 유용하다).
"""
import argparse
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "vtd"))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from check_lanes import (CENTER_M, CENTER_S, LANE_EDGE_M, LANE_EDGE_S,
                         WALK_M, WALK_S, classify, lane_change_events)  # noqa: E402
from run_logger import EGO_FRONT as FRONT, EGO_HALF_W as HALF_W  # noqa: E402

MINOR, MAJOR = 3, 6            # 경미 / 중대 [안내문]
GOAL_R = 15.0                  # 완주 판정 반경(check_run.py 와 같은 값)

# ---- 항목별 문턱. 전부 안내문 문언 그대로다. 바꾸려면 근거를 같이 적을 것.
SPD_TOL = 1 / 3.6              # ① 제한속도 +1km/h 까지 허용
SPD_MAJOR = 20 / 3.6           # ① 20km/h 초과 = 중대
ZONE_TOL = 1 / 3.6             # ② 보호구역 +1 경미
ZONE_MAJOR = 5 / 3.6           # ② +5 중대
ZONE_LIM = 30 / 3.6 + 0.1      # 보호구역으로 볼 제한속도 상한
STOP_NEAR = 2.0                # ⑦⑨ 앞범퍼가 정지선 전 [0, 2m) 안에서
STOP_HOLD = 0.5                # ⑦⑨ 이만큼 서 있어야 인정[s]
STOP_V = 1 / 3.6               # 정지로 볼 속도(안내문 ⑫ 가 1km/h 를 쓴다)
GREEN_NEAR = 30.0              # ⑧ 녹색인데 정지선 30m 안에서
GREEN_MINOR, GREEN_MAJOR = 5.0, 10.0    # ⑧ 무의미 정차[s]
CW_STOP_S = 3.0                # ⑫ 횡단보도 **위**에서 3초 정지 = 경미
PED_L, PED_W = 2.2, 0.8        # 사람·이륜차 판별(VTD 는 둘 다 2.0x0.6x1.7 로 준다)
PED_NEAR = 3.0                 # ⑩ 이 안을 무정차로 지나면 중대
SIG_SEC = 3.0                  # ⑬ 차로변경 3초 전 점등
RESPAWN_JUMP = 8.0             # ⑮ 한 프레임에 이만큼 튀면 리스폰
LC_MARGIN_M = 25.0             # 차로변경 표식 앞뒤 이만큼은 '변경 중'[m·경로점]

# ---- ⑧ 안전 정차 면책 — reason/cap_by 문자열만 보고 믿지 않는다. 그 정차가
#   **실제로 그 사유에 맞는 위치의 객체** 때문이었는지까지 `objs`(fx·fy·치수·dpath)로
#   확인한다. 아래 문턱은 새로 지어낸 값이 아니라 src/behavior.py · src/drive.py 가
#   같은 판단에 **실제로 쓰는** 반경·치수를 그대로 옮긴 것이다(줄 번호는 2026-09-04
#   `rg` 확인 기준 — 코드가 바뀌면 같이 확인할 것).
SAFE_OBJ_MIN_H = 0.25          # ROAD_SURFACE 문턱 [src/vtd_io.py:140] — 노면물은 면책 근거가 아니다
PED_MIN_H = 0.85               # VRU 최저 키 [src/vtd_io.py VRU_MIN_H=0.85] — 낮은 라바콘과 사람을 가른다
AHEAD_LANE_HALF = 2.0          # 내 차로로 볼 반폭 [drive.py JAM_LANE_HALF=2.0 · behavior.py lane_half=1.75 를 포함하도록 여유]
AHEAD_FAR = 60.0               # 전방 탐색 상한 — obstacle_gap(15)·follow_gap(10)·
                                #   JAM_LEN+JAM_MARGIN(≈6.8)보다 넉넉히 크게(behavior.py·drive.py)
PED_LAT = 6.0                  # 보행자 감시 폭 [behavior.py ped_watch=4.5] + 여유 1.5
PED_FAR = GREEN_NEAR            # 이보다 멀면 애초에 이 판정 구간(정지선 30m) 밖이다
NEARBY_R = 30.0                 # 대향·교차·충돌코스 교통 반경 [behavior.py oncoming_radius=25,
                                 #   cpa_horizon(2.5s)×도심 속도(~12m/s)] 보다 여유
REAR_R = 35.0                   # 차로변경 후방 접근 반경 [drive.py LC_REAR_WATCH=35.0]
REAR_MIN_H = 0.8                # 뒤차로 볼 최저 높이 [drive.py:1054 "노면물·연석은 뒤차가 아니다"]
MOVING_V = 0.3                  # 이 이상이면 '멈춰 선 물체가 아니다'(behavior.py 여러 곳의 0.3~0.5)
SAFE_MAJORITY = 0.5             # 정차 span 중 이 비율 넘게 근거가 있어야 면책 —
                                 #   프레임 하나 누락으로 span 전체가 깎이지 않게

# 항목 8 면책 reason/cap_by. 2026-09-04 rg 로 실제 코드가 내는 문자열을 다시 확인했다 —
#   예전 목록에 있던 PED_STOP·PED_YIELD 는 어느 코드에서도 낸 적이 없어(git log 에도
#   없다) 뺐다. 아래는 **주최측 공식 답변(2026-09-04)으로 면책이 확정된** "차량·
#   보행자·장애물 대응" 정차만 담는다 — 위치 판정 방식이 reason 마다 달라 넷으로 나눈다.
#   ⚠️ `NARROW_BLOCK` 은 요구사항이 준 목록엔 없지만 그대로 두면 **조용한 회귀**가
#      된다 — 옛 `LAWFUL_WAIT` 에 있어 무조건 면책이었는데, 빼기만 하면 실제 장애물
#      (behavior.py:1170, 그 트리거 자체가 물체 존재를 전제한다)때문에 선 정차가
#      갑자기 감점된다. 그건 이번 답변("장애물 대응 정차 면책")의 정반대다 —
#      여기 넣어 **객체 근거가 있을 때만** 면책되도록 오히려 더 정확하게 만든다.
PED_SAFE = ("YIELD_PED",)                       # 횡단 보행자 양보 [behavior.py:689]
AHEAD_SAFE = ("FOLLOW", "FOLLOW_BRAKE",          # 앞차 추종/제동 [behavior.py:755,757]
              "OBSTACLE_STOP", "HEADON_STOP",    # 정지 장애물 · 정면접근 정지 [behavior.py:734]
              "JUNCTION_JAM",                    # 교차로 꼬리물기 방지(앞선 정지차) [drive.py:1426]
              "NARROW_BLOCK")                    # 좁은 통로를 막은 장애물 [behavior.py:1170]
# ★비보호좌회전 대향차 양보는 **멀리서부터** 한다 — 간격을 보고 기다리는 기동이라
#   30m 반경으로는 근거를 놓친다. 실측(로그 39개, 양보 프레임 4297개, 2026-09-05):
#     YIELD_CROSS 95% 20.6m · RTOR_YIELD 22.7m · CPA_BRAKE 26.1m  -> 30m 로 충분
#     **YIELD_ONCOMING 중앙값 38.1m · 95% 67.8m**  -> 별도로 크게 잡는다
#   실제 오탐: 코스 E t=843.3~855.5, 90° 로 교차 접근하는 차(65.6m -> 28.4m)를
#   양보했는데 30m 반경으로는 12.2초 중 2.5초만 근거로 인정돼 -6 이 붙었다.
ONCOMING_R = 70.0
ONCOMING_SAFE = ("YIELD_ONCOMING",)
NEARBY_SAFE = (ONCOMING_SAFE[0],                 # 비보호좌회전 대향 직진차 양보 [behavior.py:912]
               "YIELD_CROSS",                    # 무신호 교차로 좌회전 교차차 양보 [behavior.py:988]
               "RTOR_YIELD",                     # 적신호 우회전 대기(뒤따르는 신호차) [behavior.py:867]
               "CPA_BRAKE")                      # 충돌코스 물체 제동 [behavior.py:1184]
REAR_SAFE = ("LC_REAR_YIELD",)                   # 차로변경 목표차로 후방 접근차 양보 [drive.py:1561]
OBJECT_SAFE = PED_SAFE + AHEAD_SAFE + NEARBY_SAFE + REAR_SAFE

# 신호·법규상 대기 — 객체가 아니라 신호·법규 그 자체가 근거라 이번 공식 답변
#   범위 **밖**이다(주최측은 "차량·보행자·장애물" 대응만 면책을 확정했다). 기존처럼
#   감점만 빼고 '판정 불명' 으로 남긴다 — 지우지 않는다.
LAWFUL_WAIT = ("WAIT_LEFT_ARROW", "WAIT_LEFT_BLIND", "CROSSWALK_STOP")

ITEMS = {
    1:  "제한속도",         2:  "보호구역 속도",   3:  "차로 유지",
    4:  "중앙선 침범",       5:  "보도 침범",       6:  "실선 차로변경",
    7:  "적색 정지",         8:  "녹색 무의미 정차", 9:  "적색점멸 정지",
    10: "보행자 양보",       11: "장애물 충돌",     12: "횡단보도 위 정지",
    13: "차로변경 지시등",   14: "차량·보행자 접촉", 15: "리스폰",
}
# 지도(xodr) 없이는 판정할 수 없는 항목
NEEDS_MAP = (3, 4, 5, 6)


class Sections:
    """경로를 N 등분해 프레임마다 구간 번호를 준다. 구간 경계는 **추정**이다."""

    def __init__(self, route, n):
        self.route = route
        self.n = max(1, int(n))
        self.cum = [0.0]
        for k in range(1, len(route)):
            self.cum.append(self.cum[-1] + math.hypot(route[k][0] - route[k - 1][0],
                                                      route[k][1] - route[k - 1][1]))
        self.total = self.cum[-1] if self.cum else 0.0

    def index_of(self, x, y):
        return min(range(len(self.route)),
                   key=lambda i: (self.route[i][0] - x) ** 2 + (self.route[i][1] - y) ** 2)

    def of(self, x, y):
        if self.total <= 0:
            return 0
        s = self.cum[self.index_of(x, y)]
        return min(self.n - 1, int(s / self.total * self.n))


class Sheet:
    """구간별 감점표. 같은 항목은 구간당 한 번만 — 심화되면 차액(+3)만 더 깎는다."""

    def __init__(self, n):
        self.n = n
        self.state = [dict() for _ in range(n)]      # sec -> {item: 'minor'|'major'}
        self.why = [dict() for _ in range(n)]        # sec -> {item: 첫 근거}

    def hit(self, sec, item, level, why):
        cur = self.state[sec].get(item)
        if cur == "major" or cur == level:
            return
        self.state[sec][item] = level
        if item not in self.why[sec]:
            self.why[sec][item] = why
        elif level == "major":
            self.why[sec][item] += f" -> 심화: {why}"

    def penalty(self, sec, item):
        lv = self.state[sec].get(item)
        return MAJOR if lv == "major" else (MINOR if lv == "minor" else 0)

    def score(self, sec):
        return 100 - sum(self.penalty(sec, i) for i in ITEMS)

    def majors(self):
        return sum(1 for s in self.state for lv in s.values() if lv == "major")


def load(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            def g(k, d=None):
                v = r.get(k)
                return d if v in (None, "") else v
            rows.append(dict(
                t=float(r["t"]), x=float(r["x"]), y=float(r["y"]),
                h=float(r["heading"]), v=float(r["v"]),
                tl=int(r["tl_state"]), tid=int(r["tl_id"]),
                reason=r.get("reason", ""),
                sig=int(g("sig", 0)),
                clr=float(g("clr", 99.9)),
                d_ego=float(g("d_ego", 0.0)),
                objs=r.get("objs", ""),
                cap_by=r.get("cap_by", "")))     # 옛 로그(컬럼 없음)는 빈 문자열
    return rows


def parse_objs(cell):
    """'fx:fy:len:wid:hgt:spd:clr:...' 묶음 -> [(fx, fy, len, wid, hgt, spd, clr)]."""
    out = []
    for chunk in (cell or "").split("|"):
        p = chunk.split(":")
        if len(p) < 7:
            continue
        try:
            out.append(tuple(float(p[i]) for i in range(7)))
        except ValueError:
            continue
    return out


def parse_objs_detail(cell):
    """`objs` 셀 전체를 dict 로 파싱한다 — ⑧ 안전 정차 근거 판정 전용.

    `parse_objs()` 의 7필드 튜플 규약은 항목 ⑩⑪⑭ 가 그대로 쓰고 있어 못 바꾼다
    (요구사항 2). 이건 그 위에 `dpath`·`route_width`·상대방위(`dh`)까지 마저 읽는
    별도 헬퍼다. 옛 로그(그 세 칸이 아직 없던 버전)도 그대로 돈다 — 없으면 None.
    형식: `fx:fy:length:width:height:speed:clearance:dpath:route_width:relative_heading`
    [src/run_logger.py:15].
    """
    out = []
    for chunk in (cell or "").split("|"):
        if not chunk:
            continue
        p = chunk.split(":")
        if len(p) < 7:
            continue
        try:
            fx, fy, ol, ow, oh, sp, clr = (float(p[i]) for i in range(7))
        except ValueError:
            continue

        def _opt(i):
            if len(p) <= i:
                return None
            try:
                v = float(p[i])
            except ValueError:
                return None
            return None if math.isnan(v) else v

        out.append(dict(fx=fx, fy=fy, len=ol, wid=ow, hgt=oh, spd=sp, clr=clr,
                         dpath=_opt(7), route_w=_opt(8), dh=_opt(9)))
    return out


def _is_ped_shaped(o):
    """사람·이륜차 치수인가(항목 ⑩ 과 같은 PED_L/PED_W) + 라바콘 등 저신고물 제외."""
    return o["len"] <= PED_L and o["wid"] <= PED_W and o["hgt"] >= PED_MIN_H


def _object_backs_reason(tag, o):
    """`tag`(reason 또는 cap_by) 가 가리키는 사유와 이 객체의 위치가 실제로 맞는가.

    문턱 근거는 파일 위쪽 SAFE_* 상수 선언부의 주석 — 전부 src/behavior.py ·
    src/drive.py 가 같은 판단에 실제로 쓰는 값을 옮긴 것이다.
    """
    if o["hgt"] < SAFE_OBJ_MIN_H:
        return False                              # ROAD_SURFACE — 면책 근거가 아니다
    if tag in PED_SAFE:
        return (_is_ped_shaped(o)
                and -2.0 <= o["fx"] <= PED_FAR and abs(o["fy"]) <= PED_LAT)
    if _is_ped_shaped(o):
        return False       # 아래 세 부류는 전부 차량·장애물 대응이라 사람은 근거가 아니다
    if tag in AHEAD_SAFE:
        # `dpath` 는 이미 `d_ego` 를 더해 만든 **경로기준 절대** 횡위치다
        # [src/run_logger.py:122 `dpath = rf_d + d_ego`] — 그래서 여기서 `d_ego` 를
        # 또 더하지 않는다. drive.py 의 JUNCTION_JAM 판정도 같은 값(`abs(d+d_ego)`)을
        # 문턱과 비교한다(drive.py:1418). 옛 로그처럼 `dpath` 가 없으면 `fy`(ego 기준
        # 횡위치, `d_ego` 로 보정 못함)로 근사한다 — 완벽하지 않지만 보수적인 방향이다.
        lat = abs(o["dpath"]) if o["dpath"] is not None else abs(o["fy"])
        return -2.0 <= o["fx"] <= AHEAD_FAR and lat <= AHEAD_LANE_HALF
    if tag in ONCOMING_SAFE:
        return o["spd"] >= MOVING_V and math.hypot(o["fx"], o["fy"]) <= ONCOMING_R
    if tag in NEARBY_SAFE:
        return o["spd"] >= MOVING_V and math.hypot(o["fx"], o["fy"]) <= NEARBY_R
    if tag in REAR_SAFE:
        return o["hgt"] >= REAR_MIN_H and -REAR_R <= o["fx"] < 0.0
    return False


def _safe_object_cause(row):
    """이 프레임의 reason·cap_by 가 안전 대응 사유이고, 그걸 뒷받침하는 객체가 실제로 있나.

    요구사항 1: reason **또는** cap_by 어느 쪽이든 안전 사유면 되지만(추월·회피 중엔
    reason 이 `OVT:*`/`*+N` 으로 덮이고 cap_by 만 진짜 제약을 남긴다 — drive.py:1613·1644),
    둘 다 아니면 객체가 있어도 면책하지 않는다(요구사항 4의 '무관한 객체' 방지).
    """
    reason, cap_by = row.get("reason", ""), row.get("cap_by", "")
    tag = reason if reason in OBJECT_SAFE else (cap_by if cap_by in OBJECT_SAFE else None)
    if tag is None:
        return False
    return any(_object_backs_reason(tag, o) for o in parse_objs_detail(row.get("objs", "")))


def spans(rows, pred, min_sec):
    """`pred` 가 참인 연속 구간 중 `min_sec` 이상인 것 [(t0, t1, 대표행)]."""
    out, start, last = [], None, None
    for r in rows:
        if pred(r):
            if start is None:
                start = r
            last = r
        else:
            if start is not None and last["t"] - start["t"] >= min_sec - 1e-9:
                out.append((start["t"], last["t"], start))
            start = last = None
    if start is not None and last["t"] - start["t"] >= min_sec - 1e-9:
        out.append((start["t"], last["t"], start))
    return out


# ------------------------------------------------------------------ 항목 구현
def item_speed(rows, sheet, secs, lim_at):
    """① 제한속도 · ② 보호구역 속도. 안내문은 **후륜축** 기준이라 기록 좌표 그대로다."""
    for r in rows:
        lim = lim_at(r["x"], r["y"])
        if lim is None:
            continue
        over = r["v"] - lim
        sec = secs.of(r["x"], r["y"])
        zone = lim <= ZONE_LIM
        if zone:
            if over > ZONE_MAJOR:
                sheet.hit(sec, 2, "major",
                          f"t={r['t']:.1f} {r['v']*3.6:.0f}>{lim*3.6:.0f}km/h")
            elif over > ZONE_TOL:
                sheet.hit(sec, 2, "minor",
                          f"t={r['t']:.1f} {r['v']*3.6:.0f}>{lim*3.6:.0f}km/h")
        else:
            if over > SPD_MAJOR:
                sheet.hit(sec, 1, "major",
                          f"t={r['t']:.1f} {r['v']*3.6:.0f}>{lim*3.6:.0f}km/h")
            elif over > SPD_TOL:
                sheet.hit(sec, 1, "minor",
                          f"t={r['t']:.1f} {r['v']*3.6:.0f}>{lim*3.6:.0f}km/h")


def item_lane_geometry(rows, sheet, secs, lc_at=None):
    """③ 차로 유지 · ④ 중앙선 · ⑤ 보도 · ⑥ 실선 차로변경 — `check_lanes.classify` 결과 사용.

    ⚠️ 출발 직후는 뺀다. ego 는 차선을 물고 스폰되고(실측 2026-08-27 주변교통 6판 전부
       t=0~2.5s 가 보도침범으로 찍혔다) 그건 우리가 한 게 아니다.
    """
    #   ⚠️ 거리 8m 로는 모자랐다 — 실측 코스 E t=2.9 에 아직 `d_ego=+1.05` 였다(스폰이
    #      경로에서 1m 옆이라 수렴 중). **경로에 붙을 때까지** 뺀다.
    sx, sy = rows[0]["x"], rows[0]["y"]
    t_go = None
    for r in rows:
        if (math.hypot(r["x"] - sx, r["y"] - sy) > 8.0
                and abs(r.get("d_ego") or 0.0) < 0.5):
            t_go = r["t"]
            break
    live = [r for r in rows if t_go is not None and r["t"] >= t_go]
    if not live:
        return

    # 차로변경 후보는 공용 helper가 고른다. lane 번호 변화가 아니라 기존 driving-lane
    # 경계에 OBB가 실제로 진입했는지를 보며, 후륜 lane 변화는 완료 정보로만 쓴다.
    LC_WIN = 2.5
    changes = lane_change_events(live)

    def during_change(r):
        if lc_at is not None and lc_at(r["x"], r["y"]):
            return True                       # 계획기가 여기서 차로를 바꾸라고 적었다
        return any(abs(r["t"] - c["t"]) < LC_WIN for c, _, _ in changes)

    # ③ 차로 유지 — 차로를 **바꾸는 중**에 선을 넘는 건 정상이라 뺀다.
    #   ★얼마나 물었는지를 같이 남긴다. "물었다/안 물었다" 만으로는 지도 해상도 문제인지
    #     경로가 실제로 치우친 것인지 다음 사람이 또 처음부터 파야 한다.
    #     실측 코스 E: 차로폭 3.1~3.2m 에서 뒷축~경계가 0.07~0.88m 였다(차체 반폭 0.943)
    #     — d_ego 는 내내 0.2~0.3 이라 **경로 자체가 차로 중앙이 아니었다.**
    # 주최측은 단일 프레임 접촉은 즉시 감점하지 않고 '유의미한 이탈'만 본다고 답했다.
    # 내부 문턱은 비공개이므로 공용 잠정값(LANE_EDGE_M/S)으로 깊이와 시간을 모두 거른다.
    # 횟수와 심화 여부는 전체 주행이 아니라 **구간마다** 독립적으로 센다.
    n_edge = {}
    for t0, t1, r0 in spans(live,
                            lambda r: (r.get("lane_intrusion") is not None
                                       and r["lane_intrusion"] + 1e-9 >= LANE_EDGE_M
                                       and not during_change(r)), LANE_EDGE_S):
        span_rows = [r for r in live if t0 <= r["t"] <= t1]
        first_by_sec = {}
        for r in span_rows:
            first_by_sec.setdefault(secs.of(r["x"], r["y"]), r)
        # 하나의 유의미한 이탈이 구간 경계를 걸치면 위반 상태가 존재한 각 구간에
        # 독립적으로 한 번씩 반영한다.
        for sec, first_r in first_by_sec.items():
            n_edge[sec] = n_edge.get(sec, 0) + 1
            deep = max(r["lane_intrusion"] for r in span_rows
                       if secs.of(r["x"], r["y"]) == sec
                       and r.get("lane_intrusion") is not None)
            sheet.hit(sec, 3, "major" if n_edge[sec] >= 2 else "minor",
                      f"t={t0:.1f}~{t1:.1f} ({first_r['x']:.0f},{first_r['y']:.0f})"
                      f" 물린 깊이 {deep:.2f}m"
                      f" d_ego={first_r.get('d_ego', 0.0):+.2f}"
                      f" [잠정 문턱 {LANE_EDGE_M:.2f}m·{LANE_EDGE_S:.2f}s]")

    # ④ 중앙선 — OBB가 정상 차로군 안쪽 경계를 0.6m 이상 넘은 상태가 0.6초 이상.
    for t0, t1, r0 in spans(
            live,
            lambda r: (r.get("center_intrusion") is not None
                       and r["center_intrusion"] + 1e-9 >= CENTER_M),
            CENTER_S):
        deep = max(r["center_intrusion"] for r in live
                   if t0 <= r["t"] <= t1 and r.get("center_intrusion") is not None)
        sheet.hit(secs.of(r0["x"], r0["y"]), 4, "major",
                  f"t={t0:.1f}~{t1:.1f} ({r0['x']:.0f},{r0['y']:.0f}) "
                  f"깊이 {deep:.2f}m·{t1-t0:.1f}초")

    # ⑤ 보도 — OBB와 OpenDRIVE sidewalk의 겹침 깊이 0.5m·0.3초.
    for t0, t1, r0 in spans(
            live,
            lambda r: (r.get("sidewalk_intrusion") is not None
                       and r["sidewalk_intrusion"] + 1e-9 >= WALK_M),
            WALK_S):
        deep = max(r["sidewalk_intrusion"] for r in live
                   if t0 <= r["t"] <= t1 and r.get("sidewalk_intrusion") is not None)
        sheet.hit(secs.of(r0["x"], r0["y"]), 5, "major",
                  f"t={t0:.1f}~{t1:.1f} ({r0['x']:.0f},{r0['y']:.0f}) "
                  f"깊이 {deep:.2f}m·{t1-t0:.1f}초")

    # ⑥ 실선 차로변경 — 위에서 OBB가 실제 경계에 진입한 `changes` 를 쓴다.
    #   ★넘은 **그 경계**의 표시를 골라야 한다. OpenDRIVE 의 `roadMark` 는 그 차로의
    #     **바깥쪽(기준선에서 먼) 경계**를 가리킨다. 그래서 3->4 든 4->3 이든 넘은 선은
    #     `|id|` 가 **작은 쪽 차로**의 표시다. 직전 차로의 표시를 그냥 쓰면 절반이 틀린다.
    #   ★★**주행차로끼리의 이동만** 차로변경이다. 실측 코스 A 두 곳(t=215.6 road 2574
    #     lane 1->2, t=291.7 road 2626 lane 2->3)이 전부 `border`(연석)로 들어간 것이었다
    #     — 그건 차로변경이 아니라 바깥으로 밀린 것이고, 항목 5 가 볼 일이다.
    #     그 `solid` 는 차로 사이 선이 아니라 **길가장자리 구역선**이다.
    #   ★roadMark 규약은 xodr 로 확인했다(2026-09-01): road 2626 s=68 에서
    #     lane 1=broken · lane 2(맨 바깥 주행차로)=solid · `<center>`=solid.
    #     즉 **차로의 roadMark = 그 차로의 바깥쪽 경계**이고 중앙선은 `<center>` 가 따로 있다.
    #     그러니 넘은 선은 `|id|` 가 작은 쪽 차로의 표시가 맞다.
    #   ★넘은 선의 표시는 `lane_change_events`가 OBB가 가로지른 **그 경계의 것**을
    #     직접 준다. 예전처럼 직전/현재 차로의 mark를 간접 선택하지 않는다.
    n_solid = {}
    for r, p, mk in changes:
        if p.get("off") or r.get("off"):
            continue                          # 주행차로 -> 연석/보도는 차로변경이 아니다
        if "solid" not in str(mk or ""):
            continue
        sec = secs.of(r["x"], r["y"])
        n_solid[sec] = n_solid.get(sec, 0) + 1
        src = r.get("lane_change_from_lane", p.get("lane"))
        dst = r.get("lane_change_to_lane")
        dst_text = str(dst) if dst is not None else "경계 반대편(후륜 미전환)"
        sheet.hit(sec, 6, "major" if n_solid[sec] >= 2 else "minor",
                  f"t={r['t']:.1f} 차로 {src}->{dst_text} (OBB가 넘은 선 = {mk})")


def item_traffic_light(rows, sheet, secs, tl_stops, notes):
    """⑦ 적색 정지 · ⑧ 녹색 무의미 정차 · ⑨ 적색점멸.

    정지선은 `routes/tl_map_livinglab.json`(= 주행 스택이 쓰는 것과 **같은 DB**)이다.
    다른 DB 를 쓰면 채점기와 주행기가 서로 다른 선을 보고 영원히 안 맞는다.
    """
    TL_RED, TL_FLASH = 1, 6
    # 신호 id 별로 '정지선 앞 STOP_NEAR 미만에서 STOP_HOLD 이상 섰나'와
    # '선을 넘었나'를 본다. STOP_NEAR 이상에서의 정지는 정상은 아니지만
    # 무정차 통과와 구분해 공식 기준대로 경미 위반으로 기록한다.
    held, far_held, crossed, first, ahead = {}, {}, {}, {}, {}
    stopped_since, far_stopped_since = {}, {}
    for r in rows:
        tid = r["tid"]
        p = tl_stops.get(str(tid)) or tl_stops.get(tid)
        if tid < 0 or p is None:
            continue
        dx, dy = p[0] - r["x"], p[1] - r["y"]
        fwd = dx * math.cos(-r["h"]) - dy * math.sin(-r["h"]) - FRONT   # 앞범퍼 기준
        first.setdefault((tid, r["tl"]), r)
        key = (tid, r["tl"])
        # ★그 신호에 **다가간 적**이 있어야 '넘었다'가 된다(2026-09-06). 실측 코스 B t=0.0:
        #   출발 위치가 신호147 정지선 94m **뒤**인데 VTD 가 그 신호를 보고해 첫 프레임부터
        #   fwd<0 -> '정지 없이 통과' 로 -6 을 줬다. 다가간 적 없는 선은 넘은 게 아니다.
        if fwd > 0.0:
            ahead[key] = True
        if r["tl"] in (TL_RED, TL_FLASH):
            if 0.0 <= fwd < STOP_NEAR and r["v"] <= STOP_V:
                far_stopped_since.pop(key, None)
                t0 = stopped_since.setdefault(key, r["t"])
                if r["t"] - t0 >= STOP_HOLD:
                    held[key] = True
            elif fwd >= STOP_NEAR and r["v"] <= STOP_V:
                stopped_since.pop(key, None)
                t0 = far_stopped_since.setdefault(key, r["t"])
                if r["t"] - t0 >= STOP_HOLD:
                    far_held[key] = True
            else:
                stopped_since.pop(key, None)
                far_stopped_since.pop(key, None)
            if fwd < -1.0 and ahead.get(key):
                crossed.setdefault(key, r)
    for (tid, st), r in crossed.items():
        key = (tid, st)
        if held.get(key):
            continue
        item = 7 if st == TL_RED else 9
        if far_held.get(key):
            sheet.hit(secs.of(r["x"], r["y"]), item, "minor",
                      f"t={r['t']:.1f} 신호{tid} 정지 위치가 앞범퍼 기준 "
                      f"정지선 {STOP_NEAR}m 이상")
        else:
            sheet.hit(secs.of(r["x"], r["y"]), item, "major",
                      f"t={r['t']:.1f} 신호{tid} 정지선 앞 "
                      f"0~{STOP_NEAR}m·{STOP_HOLD}초 정지 없이 통과")

    # ⑧ 녹색인데 정지선 30m 안에서 **무의미하게** 서 있음
    #   ⚠️ '무의미' 가 관건이다. 실측 코스 E t=718.5~730.9 는 비보호 좌회전에서
    #      대향 직진차를 보내는 중이었다(YIELD_ONCOMING). 그건 법이 시키는 것이고
    #      [제26조] 무의미한 정차가 아니다.
    #   ★★2026-09-04 주최측 공식 답변: 제한속도 이하 감속·저속 주행은 감점 대상이
    #      아니고, **차량·보행자·장애물** 때문에 정지선 30m 안에서 서는 것도 이 항목
    #      면책이 확정됐다. 그래서 두 갈래로 가른다 —
    #        ① `OBJECT_SAFE`(reason 또는 cap_by) + 그걸 뒷받침하는 실제 객체
    #           (`_safe_object_cause`) -> 감점도 '판정 불명' 경고도 없다.
    #        ② `LAWFUL_WAIT` — 신호·법규 그 자체가 근거라 이번 답변 범위 밖인 것만
    #           **감점만** 빼고 '판정 불명' 으로 남긴다(기존 정책 그대로, 안 지웠다).
    #      단순 reason 문자열 대조가 아니라 그 프레임(또는 span)에 실제로 그 사유에
    #      맞는 위치의 객체가 있었는지까지 보는 이유: reason 하나만 믿으면 옆 차로의
    #      무관한 차, 혹은 로그 조작으로도 항목 8 전체가 빠져나간다(요구사항 4).
    TL_GREEN, TL_LEFT, TL_GREEN_LEFT = 3, 4, 5

    def green_idle(r):
        p = tl_stops.get(str(r["tid"])) or tl_stops.get(r["tid"])
        if p is None or r["tl"] not in (TL_GREEN, TL_LEFT, TL_GREEN_LEFT):
            return False
        dx, dy = p[0] - r["x"], p[1] - r["y"]
        fwd = dx * math.cos(-r["h"]) - dy * math.sin(-r["h"])
        return 0.0 < fwd < GREEN_NEAR and r["v"] <= STOP_V

    for t0, t1, r0 in spans(rows, green_idle, GREEN_MINOR):
        held_s = t1 - t0
        why = f"t={t0:.1f}~{t1:.1f} 신호{r0['tid']} 녹색에 {held_s:.1f}초 정차"
        span_rows = [r for r in rows if t0 <= r["t"] <= t1]

        # ① 객체 대응 안전 정차 — 공식 답변으로 면책 확정. span 전체에서 절반 넘게
        #   근거가 있으면 된다(요구사항 4: 프레임 하나 누락으로 전체가 깎이면 안 된다).
        obj_ok = sum(1 for r in span_rows if _safe_object_cause(r))
        if obj_ok > len(span_rows) * SAFE_MAJORITY:
            continue

        # ② 신호·법규상 대기 — 아직 확정 안 됨. 감점만 빼고 확인 목록으로.
        lawful = [r["reason"] for r in span_rows if r["reason"] in LAWFUL_WAIT]
        if len(lawful) > len(span_rows) * SAFE_MAJORITY:
            notes.append(f"[⑧ 판정 불명] {why} — 대부분 {max(set(lawful), key=lawful.count)} "
                         f"(법이 시키는 대기라 무의미 정차가 아니다). 채점 UI 확인 필요")
            continue

        sheet.hit(secs.of(r0["x"], r0["y"]), 8,
                  "major" if held_s >= GREEN_MAJOR else "minor", why)


def item_contact(rows, sheet, secs):
    """⑪ 장애물 충돌(경미) · ⑭ 차량·보행자 접촉(중대).

    `clr` 은 run_logger 가 매 프레임 남기는 **차체 대 차체 최소 실여유**(SAT)다.
    0 미만이면 겹쳤다는 뜻이다. 어느 쪽에 걸리는지는 상대의 치수로 가른다.
    """
    for r in rows:
        if r["clr"] >= 0.0:
            continue
        sec = secs.of(r["x"], r["y"])
        near = min(parse_objs(r["objs"]), key=lambda o: o[6], default=None)
        if near is None:
            sheet.hit(sec, 11, "minor", f"t={r['t']:.1f} 여유 {r['clr']:.2f}m")
            continue
        _fx, _fy, ol, ow, oh, _sp, _clr = near
        # 사람·이륜차는 2.0x0.6x1.7 로 온다. **높이**가 없으면 라바콘·기름통이다
        #  (실측 치수: 라바콘 0.15x0.46x0.61). 그건 항목 11(장애물, 경미)이다.
        person = ol <= PED_L and ow <= PED_W and oh >= 1.2
        vehicle = ol > PED_L
        item = 14 if (person or vehicle) else 11
        sheet.hit(sec, item, "major" if item == 14 else "minor",
                  f"t={r['t']:.1f} 여유 {r['clr']:.2f}m (상대 {ol:.1f}x{ow:.1f}m)")


def item_pedestrian(rows, sheet, secs):
    """⑩ 보행자 양보 — 사람 반경 3m 를 **안 서고** 지났나.

    ⚠️ 안내문 문언('무정차 통과 / 횡단 완료 전 미정차')만으로는 **서 있는 사람**까지
       대상인지 알 수 없다. 여기서는 **움직이는 사람**만 센다(속도 0.3m/s 이상).
       사전테스트에서 채점 UI 로 확인할 것 — 그때까지 이 항목은 참고값이다.
    """
    # ★★안내문 문언은 "**무정차** 통과 / 횡단 완료 전 미정차" 다 — **한 번도 안 섰나**를
    #   묻는 것이지 '가까울 때 속도가 0 이 아니었나' 가 아니다. 실측 코스 A t=368.0:
    #   YIELD_PED 로 감속하다 t=368.5 에 **0.7km/h** 까지 떨어뜨렸는데, 2.1km/h 이던
    #   프레임 하나로 '무정차 통과' 가 찍혔다. 접근 구간 어디서든 섰으면 뺀다.
    PED_STOP_WIN = 6.0                                 # 이 시간 안에 섰으면 '섰다'[s]

    def stopped_near(t_at):
        return any(r["v"] <= STOP_V for r in rows
                   if t_at - PED_STOP_WIN <= r["t"] <= t_at + 1.0)

    for r in rows:
        if r["v"] <= STOP_V:
            continue                                   # 서 있었으면 위반이 아니다
        for fx, fy, ol, ow, _oh, sp, _clr in parse_objs(r["objs"]):
            if not (ol <= PED_L and ow <= PED_W):
                continue                               # 사람·이륜차가 아니다
            # ★★**앞쪽만** 본다. `fx` 는 뒷축 기준이라 앞범퍼는 +3.81m 다 —
            #   fx 가 음수면 이미 **내 옆이나 뒤**고, 그건 지나쳐 간 게 아니라
            #   같이 달리는 교통이다. 실측 코스 E t=306.2: fx=-1.0 fy=2.5 인
            #   **17km/h 자전거**(옆 차로 주행)를 '보행자 무정차 통과' 로 찍었다.
            if not (0.0 <= fx <= FRONT + PED_NEAR):
                continue
            # ★보행 속도만. 2.5m/s(9km/h) 넘게 가는 건 자전거·이륜차의 **주행**이지
            #   횡단이 아니다(VTD 는 사람과 자전거를 같은 상자로 준다).
            if not (0.3 <= sp <= 2.5):
                continue
            if abs(fy) <= PED_NEAR and not stopped_near(r["t"]):
                sheet.hit(secs.of(r["x"], r["y"]), 10, "major",
                          f"t={r['t']:.1f} 보행자 옆 {abs(fy):.1f}m 를 "
                          f"{r['v']*3.6:.0f}km/h 로 **안 서고** 통과")
                break


def item_crosswalk_stop(rows, sheet, secs, cws):
    """⑫ **횡단보도 위**에서 1km/h 이하로 3초 이상 정지 = 경미.

    횡단보도 **앞**에 서는 건 의무다(제27조⑦). 위에 서 있는 게 위반이다.
    """
    if not cws:
        return

    def on_cw(r):
        fx = r["x"] + FRONT / 2 * math.cos(r["h"])     # 차체 중앙쯤
        fy = r["y"] + FRONT / 2 * math.sin(r["h"])
        return (r["v"] <= STOP_V
                and any(math.hypot(fx - c["x"], fy - c["y"]) < 3.0 for c in cws))

    for t0, t1, r0 in spans(rows, on_cw, CW_STOP_S):
        sheet.hit(secs.of(r0["x"], r0["y"]), 12, "minor",
                  f"t={t0:.1f}~{t1:.1f} 횡단보도 위 {t1-t0:.1f}초 정지")


def item_turn_signal(rows, sheet, secs):
    """⑬ 차로변경 **3초 전** 점등 [안내문] / 30m 전 [법 제38조①·시행령 별표2].

    '차로를 바꿨다' 는 **경로기준 횡위치**(`d_ego`)가 아니라 회전으로도 움직이므로,
    한 차로 폭(3.0m)만큼 **한 방향으로** 움직인 구간의 시작점을 기동 개시로 본다.
    """
    LANE_W, WIN = 3.0, 8.0
    # ⚠️ **출발 직후는 뺀다.** ego 는 차로 중앙이 아닌 곳에 스폰되고 경로로 수렴한다 —
    #    실측 코스 E: t=0 에 d_ego=3.20 -> t=4 에 -0.08. 그건 차로변경이 아니다.
    sx, sy = rows[0]["x"], rows[0]["y"]
    rows = [r for r in rows if math.hypot(r["x"] - sx, r["y"] - sy) > 8.0]
    i = 0
    while i < len(rows):
        r0 = rows[i]
        j, moved, k0 = i, 0.0, i
        while j + 1 < len(rows) and rows[j + 1]["t"] - r0["t"] < WIN:
            j += 1
            moved = rows[j]["d_ego"] - r0["d_ego"]
            if abs(moved) >= LANE_W:
                break
        if abs(moved) >= LANE_W:
            # ★★**기동이 시작된 프레임**을 찾는다. 창 안에서 완료됐다는 이유로 창의
            #   맨 앞을 시작점으로 삼으면, 아직 가만히 있던 시각을 기준으로 선행점등을
            #   재게 된다 — 4초 전에 켜 놓고도 '0초' 로 찍힌다.
            for k in range(i, j + 1):
                if abs(rows[k]["d_ego"] - r0["d_ego"]) > 0.3:
                    k0 = k
                    break
            r0 = rows[k0]
            i = k0
            want = 1 if moved > 0 else 2               # TS_LEFT / TS_RIGHT
            lead = None
            for k in range(i, -1, -1):
                if rows[k]["sig"] != want:
                    break
                lead = r0["t"] - rows[k]["t"]
            if lead is None or lead < SIG_SEC:
                sheet.hit(secs.of(r0["x"], r0["y"]), 13, "minor",
                          f"t={r0['t']:.1f} 차로변경 선행점등 "
                          f"{'0.0' if lead is None else f'{lead:.1f}'}초 < {SIG_SEC}초")
            i = j
        i += 1


def item_respawn(rows, sheet, secs):
    """⑮ 리스폰 — 구간별 1회는 무료, 그 뒤로는 매번 -6 누적.

    ⚠️ 이 항목만 '구간당 1회' 규칙의 예외다(안내문이 누적이라고 못박았다).
       그래서 Sheet 를 안 쓰고 따로 센다.
    """
    per = {}
    for a, b in zip(rows, rows[1:]):
        d = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        dt = b["t"] - a["t"]
        if d > RESPAWN_JUMP and dt < 1.0:
            sec = secs.of(b["x"], b["y"])
            per.setdefault(sec, []).append(b["t"])
    return per


# ------------------------------------------------------------------ 실행
def main():
    ap = argparse.ArgumentParser(description="HL-FMA 2026 대회 기준 채점기")
    ap.add_argument("csv")
    ap.add_argument("--route", required=True, help="routes/<이름>.json")
    ap.add_argument("--lane", help="routes/<이름>_lane.json (제한속도)")
    ap.add_argument("--xodr", help="지도. 없으면 항목 3·4·5·6 은 판정 안 함")
    ap.add_argument("--sections", type=int, default=1,
                    help="구간 수(안내문에 없음 — 추정). 기본 1")
    ap.add_argument("--tl-map", default=os.path.join(HERE, "..", "routes",
                                                     "tl_map_livinglab.json"))
    ap.add_argument("--crosswalks", default=os.path.join(HERE, "..", "routes",
                                                         "crosswalks.json"))
    a = ap.parse_args()

    rows = load(a.csv)
    if not rows:
        print("빈 CSV"); return 1
    route = json.load(open(a.route, encoding="utf-8"))["ego_route"]
    secs = Sections(route, a.sections)
    sheet = Sheet(secs.n)

    lims = None
    if a.lane:
        lims = [p.get("lim") for p in json.load(open(a.lane, encoding="utf-8"))["pts"]]

    def lim_at(x, y):
        if not lims:
            return None
        i = secs.index_of(x, y)
        return lims[i] if i < len(lims) else None

    # ★계획기가 '여기서 차로를 바꾼다' 고 적어 둔 구간(`ego_lanes[i][4]`).
    #   차로변경 중에 선을 넘는 건 위반이 아니다 — 항목 3 판정에서 뺀다.
    ego_lanes = json.load(open(a.route, encoding="utf-8")).get("ego_lanes") or []
    lc_flag = [bool(p[4]) if len(p) > 4 else False for p in ego_lanes]

    def lc_at(x, y):
        if not lc_flag:
            return False
        i = secs.index_of(x, y)
        lo = max(0, i - int(LC_MARGIN_M))
        hi = min(len(lc_flag), i + int(LC_MARGIN_M) + 1)
        return any(lc_flag[lo:hi])

    tl_stops = {}
    if os.path.exists(a.tl_map):
        tl_stops = json.load(open(a.tl_map, encoding="utf-8"))
    cws = []
    if os.path.exists(a.crosswalks):
        cws = json.load(open(a.crosswalks, encoding="utf-8"))["crosswalks"]
        cws = [c for c in cws
               if min((math.hypot(p[0] - c["x"], p[1] - c["y"]) for p in route),
                      default=99) < 6.0]

    mapped = False
    if a.xodr:
        from build_lane_plan import Map                       # noqa: E402
        print(f"[채점] 지도 읽는 중 {a.xodr} (몇 분 걸린다)", flush=True)
        classify(rows, Map(a.xodr))
        mapped = True

    notes = []
    item_speed(rows, sheet, secs, lim_at)
    if mapped:
        item_lane_geometry(rows, sheet, secs, lc_at)
    item_traffic_light(rows, sheet, secs, tl_stops, notes)
    item_contact(rows, sheet, secs)
    item_pedestrian(rows, sheet, secs)
    item_crosswalk_stop(rows, sheet, secs, cws)
    item_turn_signal(rows, sheet, secs)
    respawns = item_respawn(rows, sheet, secs)

    # ---- 완주 판정: 뒷축이 종점을 지났나 [안내문]
    gx, gy = route[-1]
    dmin = min(math.hypot(r["x"] - gx, r["y"] - gy) for r in rows)
    finished = dmin <= GOAL_R
    elapsed = rows[-1]["t"] - rows[0]["t"]

    # ---- 출력
    W = 74
    print("=" * W)
    print(f"  HL-FMA 2026 채점  |  {os.path.basename(a.csv)}  |  {secs.n}구간")
    print("=" * W)
    total = 0
    for k in range(secs.n):
        pen_r = 0
        rs = respawns.get(k, [])
        if len(rs) > 1:
            pen_r = MAJOR * (len(rs) - 1)              # 구간당 1회 무료
        sc = sheet.score(k) - pen_r
        total += sc
        head = f"  구간 {k+1}/{secs.n}   {sc:>4d} / 100"
        print(f"\n{head}")
        rows_out = [(i, sheet.penalty(k, i), sheet.why[k].get(i))
                    for i in ITEMS if sheet.penalty(k, i)]
        if pen_r:
            rows_out.append((15, pen_r, f"리스폰 {len(rs)}회 (1회 무료)"))
        if not rows_out:
            print("       감점 없음")
        for i, p, why in rows_out:
            print(f"       -{p:<2d} [{i:>2d}] {ITEMS[i]:<12s} {why}")

    if notes:
        print("\n  ⚠️ 확인 필요 — 감점에서 뺐지만 심판이 어떻게 볼지 모르는 것")
        for nt in notes:
            print(f"       {nt}")

    print("\n" + "-" * W)
    print(f"  총점        {total:>6d} / {secs.n * 100}")
    print(f"  완주        {'✅ 완주' if finished else f'❌ 미완주 (종점까지 {dmin:.0f}m)'}"
          f"   |  소요 {elapsed/60:.1f}분  |  중대 위반 {sheet.majors()}회")
    if not mapped:
        print(f"  ⚠️ --xodr 없음 -> 항목 {NEEDS_MAP} 는 **판정하지 않았다**")
    print(f"  ⚠️ 구간 수 {secs.n} 은 추정이다 — 안내문에 경계가 없다. "
          f"사전테스트에서 채점 UI 로 확인할 것")
    print("-" * W)
    return 0


if __name__ == "__main__":
    sys.exit(main())
