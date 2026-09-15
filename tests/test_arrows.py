"""노면 진행방향 화살표(지정차로) 유닛테스트.

배경(2026-08-16 실주행 지적): "방금 좌회전 차선에서 직진함". 확인해 보니 우리는
교차로 `connection/laneLink` 만 보고 있었다. laneLink 는 **물리적으로 이어지는가**만
말한다 — road 310 lane -1 은 직진 연결로가 있어서 laneLink 상 직진이 되지만,
바닥에는 `RM_537_LT`(좌회전 전용)가 그려져 있다. 채점자도 사람도 바닥을 본다.

이 테스트가 지키는 것:
  ① 이름 -> 방향 해석이 맞는가(좌우가 뒤집히면 정확히 반대로 달린다)
  ② 화살표가 laneLink 를 **이긴다**
  ③ 내 차로 화살표와 옆 차로 화살표를 t 로 제대로 가르는가
  ④ 편도 1차로처럼 지킬 방법이 없을 때 need 를 만들지 않는가

①은 실지도로, 나머지는 가짜 pts 로 돈다.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "vtd"))

import build_lane_plan as B                                    # noqa: E402

XODR = os.path.expanduser("~/hlfma2026_map/HL_FMA_VTD_LivingLab.xodr")

ok = fail = 0


def check(name, cond, note=""):
    global ok, fail
    if cond:
        ok += 1; print(f"  ✅ {name}")
    else:
        fail += 1; print(f"  ❌ {name}  {note}")


print("[이름 해석] 화살표 코드가 뜻하는 방향")
check("537_LT 는 좌회전", B.ARROW_MOVES["RM_537_LT"] == {"left"})
check("537_RT 는 우회전", B.ARROW_MOVES["RM_537_RT"] == {"right"})
check("538_SLT 는 직진+좌회전", B.ARROW_MOVES["RM_538_SLT"] == {"straight", "left"})
check("538_SRT 는 직진+우회전", B.ARROW_MOVES["RM_538_SRT"] == {"straight", "right"})


def fake_pt(road, s, t, flip=1.0, lanes=None):
    return dict(lane=-1, w=3.0, l=0.0, r=0.0, xl=0.0, xr=0.0, need=0.0, sig=0, j=0,
                _road=road, _s=s, _t=t, _flip=flip,
                _lanes=lanes if lanes is not None else
                [(-1, "driving", -3.2, 0.0), (-2, "driving", -6.4, -3.2)],
                _junc=False)


print()
print("[이동 구간] 차선변경을 끝낼 시간을 주는가")
check("멀면 0%", B.ramp(B.APPROACH + 1) == 0.0)
check("FULL_BY 지점에서 100%", B.ramp(B.FULL_BY) == 1.0)
check("그보다 가까워도 100%", B.ramp(10.0) == 1.0)
check("중간은 중간", 0.4 < B.ramp((B.APPROACH + B.FULL_BY) / 2) < 0.6)
# 한 차로(3.2m)를 LANE_RATE 로 옮기는 데 걸리는 거리 < 계획이 주는 이동 구간
LANE_RATE = 0.9                     # drive.py Driver.LANE_RATE 와 같은 값
need_m = (3.2 / LANE_RATE) * (50 / 3.6)
check("50km/h 에서 한 차로 옮길 거리가 이동 구간 안에 든다",
      need_m <= B.APPROACH - B.FULL_BY,
      f"필요 {need_m:.0f}m > 주어진 {B.APPROACH - B.FULL_BY:.0f}m")

print()
print("[내 차로 / 옆 차로] t 로 가른다")
# 도로 R: 30m 구간, 우리 t=-1.6(1차로). 옆 차로 중심 t=-4.8.
arrows = {"R": [(20.0, -1.6, {"left"}), (20.0, -4.8, {"straight", "right"})]}
pts = [fake_pt("R", float(s), -1.6) for s in range(0, 31)]
cum = [float(i) for i in range(31)]
mine, others = B.arrows_near(arrows, pts, cum, 30)
check("내 차로 화살표만 mine 에", mine == {"left"}, str(mine))
check("옆 차로는 others 로", len(others) == 1 and others[0][1] == {"straight", "right"})
check("옆 차로 방향이 주행기준 우(-)", others[0][0] < 0, str(others[0][0]))

mine_r, others_r = B.arrows_near(arrows, [fake_pt("R", float(s), -1.6, flip=-1.0)
                                          for s in range(31)], cum, 30)
check("반대방향 주행이면 좌우 부호가 뒤집힌다", others_r[0][0] > 0, str(others_r[0][0]))

far = {"R": [(20.0, -12.0, {"left"})]}
mine_f, others_f = B.arrows_near(far, pts, cum, 30)
check("너무 먼 t 도 others 에는 들어간다", mine_f is None and len(others_f) == 1)

behind = {"R": [(500.0, -1.6, {"left"})]}
check("구간 밖 s 는 무시", B.arrows_near(behind, pts, cum, 30) == (None, []))

check("화살표 없는 도로는 (None, [])", B.arrows_near({}, pts, cum, 30) == (None, []))


print()
print("[우선순위] 화살표가 laneLink 를 이긴다")


class FakeJunc:
    """laneLink 상으로는 직진이 되는 교차로(연결로가 곧다)."""
    def findall(self, tag):
        if tag != "connection":
            return []
        c = type("C", (), {})()
        c.get = lambda k, d=None: {"incomingRoad": "R", "connectingRoad": "C"}.get(k, d)
        ll = type("L", (), {})()
        ll.get = lambda k, d=None: {"from": "-1", "to": "-1"}.get(k, d)
        c.findall = lambda t: [ll] if t == "laneLink" else []
        return [c]


class FakeMap:
    juncs = [FakeJunc()]
    roads = {}


def run_case(arrow_map, hdg_out_deg):
    """R 도로 30m 직진 후 교차로 1점. hdg_out 으로 회전 종류가 정해진다."""
    n = 34
    pts = [fake_pt("R", float(i), -1.6) for i in range(31)] + [None, None] + \
          [fake_pt("X", 0.0, -1.6)]
    route = [(float(i), 0.0) for i in range(n)]
    cum = [float(i) for i in range(n)]
    hdgs = [0.0] * 31 + [0.0, 0.0] + [math.radians(hdg_out_deg)]
    notes = B.plan_turn_lanes(FakeMap(), route, hdgs, pts, cum, arrow_map)
    return pts, notes


# 내 차로가 좌회전 전용인데 직진하려 한다 -> 차로 통째 이동은 need 로 때우지 않는다.
# (그렇게 했더니 VTD 가 경로이탈로 리스폰시켰다 — lay_need() 주석 참고. 경로 계획 몫이다.)
pts_a, notes_a = run_case({"R": [(28.0, -1.6, {"left"}),
                                 (28.0, -4.8, {"straight", "right"})]}, 0)
check("차로 통째 이동은 need 를 안 깐다(리스폰)", pts_a[30]["need"] == 0.0,
      f"need={pts_a[30]['need']}")
check("대신 경고로 남긴다", bool(notes_a) and "차로 통째" in notes_a[0], str(notes_a))

check("화살표를 읽었다는 게 메모에 남는다",
      bool(notes_a) and "화살표" in notes_a[0], str(notes_a))
# 옆 차로 화살표는 항상 ARROW_T(1.75m) 밖이므로, 화살표發 need 는 구조적으로 안 깔린다.
check("ARROW_T 가 NEED_MAX 보다 크다(= 화살표發 need 는 늘 경로 몫)",
      B.ARROW_T > B.NEED_MAX, f"{B.ARROW_T} vs {B.NEED_MAX}")

# lay_need 자체는 차로 안 보정을 그대로 깐다
N = 121
pts_s = [fake_pt("R", float(i), -1.6) for i in range(N)]
cum_s = [float(i) for i in range(N)]
notes_s = []
laid = B.lay_need(pts_s, cum_s, N - 1, -0.8, notes_s, "테스트")
check("차로 안 보정(0.8m)은 깐다", laid and pts_s[N - 1]["need"] == -0.8,
      f"need={pts_s[N - 1]['need']}")
check("FULL_BY 지점엔 이미 다 깔려 있다",
      pts_s[N - 1 - int(B.FULL_BY)]["need"] == -0.8)
check("그보다 뒤는 램프로 줄어든다", abs(pts_s[N - 1 - 80]["need"]) < 0.8)
check("APPROACH 밖은 0", pts_s[0]["need"] == 0.0)
check("깔았으면 경고를 안 남긴다", notes_s == [])

# 내 차로가 직진 허용이면 아무것도 하지 않는다
pts_b, notes_b = run_case({"R": [(28.0, -1.6, {"straight", "right"})]}, 0)
check("직진 허용 차로면 need 0", pts_b[30]["need"] == 0.0, str(pts_b[30]["need"]))
check("메모도 안 남긴다", notes_b == [])

# 편도 1차로 — 화살표를 지킬 방법이 없다
one_lane = [fake_pt("R", float(i), -1.6) for i in range(31)]
for p in one_lane:
    p["_lanes"] = [(-1, "driving", -3.2, 0.0), (1, "driving", 0.0, 3.2)]
pts_c = one_lane + [None, None] + [fake_pt("X", 0.0, -1.6)]
notes_c = B.plan_turn_lanes(FakeMap(), [(float(i), 0.0) for i in range(34)],
                            [0.0] * 34, pts_c, [float(i) for i in range(34)],
                            {"R": [(28.0, -1.6, {"left"})]})
check("편도 1차로면 need 를 만들지 않는다", pts_c[30]["need"] == 0.0)
check("대신 경고를 남긴다", bool(notes_c) and "편도 1차로" in notes_c[0], str(notes_c))


if os.path.exists(XODR):
    print()
    print("[실지도] 실제로 지적된 곳이 잡히는가")
    mp = B.Map(XODR)
    ar = B.lane_arrows(mp)
    a310 = {round(t, 2): sorted(m) for s, t, m in ar.get("310", []) if s > 100}
    check("road 310 1차로는 좌회전 전용", a310.get(-1.65) == ["left"], str(a310))
    check("road 310 2차로는 직진+우회전", a310.get(-4.75) == ["right", "straight"], str(a310))
    check("화살표가 있는 도로가 100개 넘는다", len(ar) > 100, str(len(ar)))

    print()
    print("[보호구역] 536/518 을 30 으로 읽는가")
    lim = B.road_speed_limits(mp)
    prot = set()
    for rid, rd in mp.roads.items():
        for o in rd.findall("objects/object"):
            if (o.get("name") or "").split(".")[0] in ("RM_518", "RM_536"):
                prot.add(rid)
    miss = [r for r in prot if lim.get(r, 99) > 30.5 / 3.6]
    check("보호구역 도로는 전부 30", not miss, f"빠진 도로 {sorted(miss)[:6]}")
    check("보호구역 표시가 실제로 있다(테스트가 자명하지 않음)", len(prot) > 20, str(len(prot)))
    # 이게 실주행 지적("어린이보호구역에서 50")의 진짜 원인이었다: 536 이 그려진
    # 도로 17개 중 16개에 roadmark_speed_30 이 없다.
    only536 = [r for r in prot
               if not any((o.get("name") or "").startswith("roadmark_speed_30")
                          for o in mp.roads[r].findall("objects/object"))]
    check("30 표시 없이 536/518 만 있는 도로가 대부분", len(only536) >= 20, str(len(only536)))
    check("그 도로들도 30 으로 나온다",
          all(lim.get(r, 99) <= 30.5 / 3.6 for r in only536))
else:
    print(f"\n(실지도 테스트 건너뜀 — {XODR} 없음)")


print()
print(f"통과 {ok} / 실패 {fail}")
sys.exit(1 if fail else 0)
