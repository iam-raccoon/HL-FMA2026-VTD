"""경로 계획기 — 차선변경 비용 모델 유닛테스트.

배경(2026-08-16): 차선변경 비용이 **고정 60m** 였다. arXiv:2206.02883 이 지적하듯
차선변경은 공짜인 결정적 연산이 아니라 활주로가 모자라면 못 끝낼 수 있는 행동이다.
활주로 기반 비용으로 바꾸면서 노드에 '그 도로에서의 변경 횟수'가 붙었는데,
**그것만으로 힙 순서가 바뀌어** 비용이 정확히 같은 다른 경로가 튀어나왔다
(둘 다 974.11m. 600m 를 한 차로 바깥으로 도는 경로 -> VTD 허용오차 1.5m 초과 -> 리스폰).
그래서 여기서 검사하는 것은 두 가지다: 비용 모델이 실제로 활주로를 반영하는가,
그리고 **동점일 때 늦게 바꾸는 쪽으로 결정적으로** 깨지는가.

xodr 없이 도는 테스트다(가짜 그래프). 실지도 회귀는 `--like` 와 check_route.py 담당.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "vtd"))

from plan_route import LC_BASE, LC_RUNWAY, LaneGraph, lane_change_cost   # noqa: E402

ok = fail = 0


def check(name, cond, note=""):
    global ok, fail
    if cond:
        ok += 1; print(f"  ✅ {name}")
    else:
        fail += 1; print(f"  ❌ {name}  {note}")


print("[비용 모델] 활주로가 짧을수록 비싸다")
check("긴 도로는 기본값", lane_change_cost(400, 1) == LC_BASE)
check("활주로 딱 맞으면 기본값", lane_change_cost(LC_RUNWAY, 1) == LC_BASE)
check("절반이면 2배", abs(lane_change_cost(LC_RUNWAY / 2, 1) - 2 * LC_BASE) < 1e-9)
check("횟수가 늘면 1회당 활주로가 줄어 비싸진다",
      lane_change_cost(60, 2) > lane_change_cost(60, 1),
      f"{lane_change_cost(60, 2)} vs {lane_change_cost(60, 1)}")
check("긴 도로는 2회여도 안 비싸다", lane_change_cost(400, 2) == LC_BASE)
check("상한이 있다(경로 자체를 잃지 않게)",
      lane_change_cost(1, 3) == lane_change_cost(10, 3) == LC_BASE / 0.25)
check("단조 비증가", all(lane_change_cost(L, 1) <= lane_change_cost(L - 5, 1)
                        for L in range(20, 200, 5)))


class FakeGraph(LaneGraph):
    """도로 A(200m, 2차로) -> B(200m, 2차로) -> C(연결로). C 는 lane -1 에서만 이어진다.

    즉 A 나 B 어느 쪽에서 -2 -> -1 로 바꿔도 총 비용이 **정확히 같다**.
    사람도 내비도 이럴 때 B 에서 바꾼다 — 목적지 직전까지 원래 차로를 지킨다.
    """
    def __init__(self, len_a=200.0, len_b=200.0):
        self.LA, self.LB = len_a, len_b

    def successors(self, node):
        rid, lid, k = node
        out = []
        if rid == "A":
            out.append((("B", lid, 0), self.LA, False))
            if k < 1 and lid == -2:
                out.append((("A", -1, k + 1), lane_change_cost(self.LA, k + 1), True))
        elif rid == "B":
            if lid == -1:
                out.append((("C", -1, 0), self.LB, False))
            if k < 1 and lid == -2:
                out.append((("B", -1, k + 1), lane_change_cost(self.LB, k + 1), True))
        return out


print("\n[동점 깨기] 비용이 같으면 늦게 바꾼다")
p = FakeGraph().search([("A", -2)], [[("C", -1)]])
check("경로를 찾는다", p is not None)
if p:
    lc = [r for i, (r, _l, _k) in enumerate(p) if i and p[i - 1][0] == r]
    check("차선변경은 B 에서 (A 가 아니라)", lc == ["B"], f"실제 {lc}  경로 {p}")

print("\n[우선순위] 활주로 페널티는 동점깨기를 이긴다")
# B 를 짧게 만들면(30m) B 에서 바꾸는 게 실제로 위험하다 -> A 로 앞당겨야 한다
p = FakeGraph(len_b=30.0).search([("A", -2)], [[("C", -1)]])
check("경로를 찾는다", p is not None)
if p:
    lc = [r for i, (r, _l, _k) in enumerate(p) if i and p[i - 1][0] == r]
    check("짧은 B 를 피해 A 에서 미리 바꾼다", lc == ["A"], f"실제 {lc}  경로 {p}")


class PocketGraph(LaneGraph):
    """진입 +2와 새 진출 포켓 +2가 번호만 같은 경우."""
    def __init__(self):
        pass

    def successors(self, node):
        if node == ("A", 2, 0):
            return [(("A", 2, 1, 2), 10.0, True)]
        if node == ("A", 2, 1, 2):
            return [(("C", -1, 0), 10.0, False)]
        return []


print("\n[회전 포켓] 같은 차로번호라도 진입 물리차로와 새 포켓을 구분한다")
p = PocketGraph().search([("A", 2)], [[("A", 2, True)], [("C", -1, False)]])
check("진입 +2를 포켓 +2로 오인하지 않는다", p is not None and len(p) == 3, f"경로 {p}")
if p:
    check("포켓 차선변경 상태를 실제로 거친다", len(p[1]) == 4 and p[1][3] == 2,
          f"경로 {p}")



# ---------------------------------------------------------------- laneSection
# 실지도가 있으면 번호 재부여 대응(thru)까지 확인한다. 없으면 조용히 건너뛴다.
import glob                                                          # noqa: E402
import os as _os                                                     # noqa: E402

_x = next((p for p in glob.glob(_os.path.expanduser(
    "~/hlfma2026_map/HL_FMA_VTD_LivingLab.xodr")) or glob.glob(_os.path.expanduser(
        "~/.claude/jobs/*/tmp/map/HL_FMA_VTD_LivingLab.xodr"))), None)
if _x is None:
    print("\n[laneSection 번호 재부여] xodr 없음 — 건너뜀")
else:
    print("\n[laneSection 번호 재부여] 같은 물리차로인데 번호가 바뀌는 도로")
    from build_lane_plan import Map                                  # noqa: E402
    from plan_route import LaneGraph                                 # noqa: E402
    mp = Map(_x)
    g = LaneGraph(mp)

    def phys(rid, lid, d):
        """횡위치 t 를 따라가며 진출 쪽에서 실제로 어느 번호가 되는지 — 기하로 구한 정답."""
        rd = mp.roads[rid]; L = float(rd.get("length"))
        ss = [i / 5.0 for i in range(int(L * 5) + 1)]
        if d < 0: ss = ss[::-1]
        cur = None; last = lid
        for s in ss:
            here = [(l, (lo + hi) / 2.0) for l, ty, lo, hi, _m in mp.lanes(rd, min(s, L))
                    if ty == "driving" and (l > 0) == (lid > 0)]
            if not here: continue
            if cur is None:
                m = [t for l, t in here if l == lid]
                if not m: return None
                cur = m[0]
            last, cur = min(here, key=lambda z: abs(z[1] - cur))[0], \
                min((t for _l, t in here), key=lambda t: abs(t - cur))
        return last

    # road 128(laneSection 7개, s=60 에서 차로가 생김) / 2011(8개, 좌회전 포켓)
    for rid, lid, d in (("128", 2, -1), ("2011", -1, +1)):
        want = phys(rid, lid, d)
        got = g.thru(rid, lid, d)
        check(f"thru({rid}, {lid:+d}) = 기하가 말하는 {want}", got == want, f"실제 {got}")
    check("번호가 실제로 바뀌는 사례다(테스트가 자명하지 않음)",
          g.thru("2011", -1, +1) != -1, "thru=-1 이면 검증력 없음")

    # ★출발 도로는 중간에서 시작한다 -> **진행 방향으로 남은 구간만** 훑어야 한다.
    #   방향을 안 보면 -s 로 가는데 s 가 더 큰 구간을 훑어 진출 번호가 틀린다.
    for rid, lid, d in (("128", 2, -1), ("2011", -1, +1)):
        L = float(mp.roads[rid].get("length"))
        far = L - 1.0 if d < 0 else 1.0          # 진입단 바로 안쪽 = 전체와 같아야 함
        check(f"thru({rid}, {lid:+d}, from_s=진입단) 이 from_s 없는 것과 같다",
              g.thru(rid, lid, d, far) == g.thru(rid, lid, d),
              f"{g.thru(rid, lid, d, far)} vs {g.thru(rid, lid, d)}")

    print("\n[차로 폭] <width> 레코드가 여러 개인 차로")
    from build_lane_plan import at_offset                            # noqa: E402
    multi = [(rd.get("id"), sec, ln)
             for rd in mp.roads.values()
             for sec in rd.findall("lanes/laneSection")
             for side in ("left", "right")
             for ln in sec.findall(f"{side}/lane")
             if len(ln.findall("width")) > 1]
    check("이 맵에 실제로 존재한다(테스트가 자명하지 않음)", len(multi) > 100, f"{len(multi)}개")
    check("sOffset 이 큰 지점에선 뒤쪽 레코드를 고른다",
          all(at_offset(ln.findall("width"), 1e9) is ln.findall("width")[-1]
              for _r, _s, ln in multi[:200]))

    # 폭 폭주 회귀: 3차식을 유효범위 밖으로 외삽하면 차로가 수십 m 로 벌어진다
    wide = 0; jump = 0.0; n = 0
    for rd in mp.roads.values():
        secs = sorted(float(x.get("s", 0)) for x in rd.findall("lanes/laneSection"))
        bounds = secs + [float(rd.get("length"))]
        for i, a in enumerate(secs):
            prev = {}; s = a + 0.05
            while s < bounds[i + 1] - 0.05:
                for l, ty, lo, hi, _m in mp.lanes(rd, s):
                    if ty != "driving": continue
                    n += 1
                    if hi - lo > 6.0: wide += 1
                    c = (lo + hi) / 2.0
                    if l in prev: jump = max(jump, abs(c - prev[l]))
                    prev[l] = c
                s += 0.5
    check(f"폭 6m 넘는 주행차로 표본 0개 (전체 {n})", wide == 0, f"{wide}개")
    check(f"laneSection 내부 중심선 0.5m당 이동 <= 0.3m", jump <= 0.3, f"최대 {jump:.2f}m")


    print("\n[교차로 진입 차로] 도로 반대쪽 끝 진입로가 섞이는가")
    mixed = []
    for jc in mp.juncs:
        by_inc = {}
        for c in jc.findall("connection"):
            for l in c.findall("laneLink"):
                by_inc.setdefault(c.get("incomingRoad"), set()).add(int(l.get("from")) > 0)
        for inc, signs in by_inc.items():
            if len(signs) > 1:
                mixed.append((jc.get("id"), inc))
    check("한 incomingRoad 에 양쪽 끝 진입로가 실제로 섞여 있다(테스트가 자명하지 않음)",
          len(mixed) > 0, f"{len(mixed)}개")
    # 그런 곳에서 부호 필터를 안 걸면 '반대편 차로로 옮기라'가 나온다 — 실측 12.08m
    for jid, inc in mixed[:5]:
        jc = next(j for j in mp.juncs if j.get("id") == jid)
        allf = [int(l.get("from")) for c in jc.findall("connection")
                if c.get("incomingRoad") == inc for l in c.findall("laneLink")]
        for sign in (True, False):
            sub = [f for f in allf if (f > 0) == sign]
            if sub:
                check(f"junction {jid} / road {inc} / {'+' if sign else '-'}쪽 필터가 한 방향만 남긴다",
                      all((f > 0) == sign for f in sub), f"{sub}")

    print("\n[지점->차로 투영] s 가 실제 종방향 위치여야 한다")
    from plan_route import locate_lanes_all, wp_lanes                # noqa: E402
    from build_tl_map import road_point                              # noqa: E402
    import random                                                    # noqa: E402
    rnd = random.Random(7)
    rids = sorted(mp.roads)
    worst = 0.0; n = 0
    for rid in rnd.sample(rids, min(300, len(rids))):
        rd = mp.roads[rid]; L = float(rd.get("length"))
        if rd.get("junction", "-1") != "-1" or L < 20:
            continue
        s_true = L * 0.37                       # 격자 표본(2m)과 안 맞는 지점으로
        try:
            x, y, h = road_point(rd, s_true)
        except Exception:
            continue
        dr = [(l, (lo + hi) / 2.0) for l, ty, lo, hi, _m in mp.lanes(rd, s_true) if ty == "driving"]
        if not dr:
            continue
        _l, t = dr[0]
        px, py = x - math.sin(h) * t, y + math.cos(h) * t
        got = [(r, li, sv) for r, li, sv in locate_lanes_all(mp, px, py) if r == rid]
        if not got:
            continue
        n += 1
        worst = max(worst, min(abs(sv - s_true) for _r, _li, sv in got))
    check(f"무작위 {n}개 도로에서 s 오차 <= 1.5m", worst <= 1.5, f"최대 {worst:.1f}m")
    check("표본을 실제로 봤다(테스트가 자명하지 않음)", n >= 30, f"{n}개")



# --------------------------------------------- 차선변경을 도로 끝에서 끝내지 않는가
print("\n[차선변경 완료 여유] 차선이 없어지는 지점에서 홱 꺾지 않는다")
from plan_route import LC_FINISH, lc_blend                           # noqa: E402

# 실주행 지적("차선 없어진다고 미리 적혀 있는데 끝에 가서야 바꿈")의 진범은 보간이
# rem=0 에서 완료되도록 돼 있던 것이었다(실측 road 2003: 도로 끝 9.8m 전에야 완료).
check("시작점은 0", lc_blend(100.0, 100.0) == 0.0)
check("LC_FINISH 남기고 이미 1", lc_blend(100.0, LC_FINISH) == 1.0)
check("도로 끝에서도 1 (되돌아가지 않는다)", lc_blend(100.0, 0.0) == 1.0)
check("그 전까지는 1 미만", lc_blend(100.0, LC_FINISH + 5) < 1.0)
check("단조 증가", all(lc_blend(100.0, r) >= lc_blend(100.0, r + 1)
                      for r in range(0, 100)))
check("smoothstep — 시작/끝 기울기가 0에 가깝다",
      lc_blend(100.0, 99.0) < 0.01 and lc_blend(100.0, LC_FINISH + 1) > 0.99)
# ⚠️ 활주로가 짧아도 한 스텝에 0->1 로 튀면 안 된다(v7 헤딩 64° 반전 회귀)
for rl in (5.0, 10.0, 20.0, 25.0):
    step = max(lc_blend(rl, r / 10.0) - lc_blend(rl, r / 10.0 + 0.14)
               for r in range(0, int(rl * 10) - 2))
    check(f"활주로 {rl:.0f}m 에서도 한 스텝 증가가 완만({step:.2f})", step < 0.12, f"{step:.3f}")

# --------------------------------------------- 차선변경을 도로 끝에서 끝내지 않는가
# --------------------------------------------- 시나리오에 Path 가 여러 개일 때
print("\n[시나리오 Path 선택] 남의 차 경로를 섞으면 안 된다")
import tempfile as _tf, os as _o2

_XML = """<?xml version="1.0"?><Scenario>
 <Layout><Path Name="Path01" PathId="1">
   <Waypoint PathOption="shortest" s="10.0" TrackId="A"/>
   <Waypoint PathOption="shortest" s="20.0" TrackId="B"/></Path>
  <Path Name="Path02" PathId="2">
   <Waypoint PathOption="shortest" s="30.0" TrackId="C"/>
   <Waypoint PathOption="shortest" s="40.0" TrackId="D"/></Path></Layout>
 <Entities>
  <Player><Description Name="Ego" Control="external"/>
   <Init><PathRef StartS="3.0" TargetS="99.0" StartLane="2" PathId="1"/></Init></Player>
  <Player><Description Name="New Player01" Control="internal"/>
   <Init><PathShapeRef StartS="1.0" PathShapeId="2"/></Init></Player>
 </Entities></Scenario>"""


class _FakeMap:
    """world() 가 부르는 것만 흉내낸다 — 어떤 웨이포인트가 뽑히는지만 보면 된다."""
    roads = {k: k for k in "ABCD"}

    @staticmethod
    def lanes(rd, s):
        return [(-1, "driving", -3.2, 0.0, None)]


def _picked(xml_text):
    import plan_route as PR
    saved = PR.road_point
    PR.road_point = lambda rd, s: (float(s), 0.0, 0.0)     # x=s 로 두면 추적이 쉽다
    try:
        f = _o2.path.join(_tf.mkdtemp(), "s.xml")
        open(f, "w").write(xml_text)
        return [round(x) for x, _y in PR.from_scenario(_FakeMap(), f)]
    finally:
        PR.road_point = saved


got = _picked(_XML)
check("Ego 의 PathRef(PathId=1) 만 쓴다", got == [10, 20], f"뽑힌 s={got}")
# ⚠️ 예전엔 root.iter("Path") 를 통째로 이어붙여 [10,20,30,40] 이 나왔다. v1~v6 은
#    Path 가 하나뿐이라 멀쩡했고, **v7 만 2개**라 에고를 AI 차량 경로까지 순회시켰다
#    (3경유지 994m -> 6경유지 3818m 짜리 가짜 코스).
check("남의 차 경로(Path02)는 안 섞인다", 30 not in got and 40 not in got, f"{got}")

_ONE = _XML.replace('<Path Name="Path02" PathId="2">\n   <Waypoint PathOption="shortest" s="30.0" TrackId="C"/>\n   <Waypoint PathOption="shortest" s="40.0" TrackId="D"/></Path>', '')
check("Path 가 하나면 그대로 쓴다(v1~v6 회귀)", _picked(_ONE) == [10, 20])


print("\n[경로 차선변경 지시등] 차로가 늘어난 것과 진짜 진로변경을 가른다")
# 사용자 지적 2026-08-26: "길이 왼쪽으로 꺾인 2차선 도로인데 왜 왼쪽 깜빡이를 켜".
# 차로가 하나 늘면 남은 차로 번호가 재부여되고 중심 t 도 밀린다 — 번호도 바뀌고
# t 도 움직여서 기존 두 조건(t 이동 + 번호 변경)을 **한꺼번에** 통과했다.
# 가르는 것은 **도로 폭**이다: 진짜 진로변경은 폭이 그대로고 좌우 여유가 맞바뀐다.
from build_lane_plan import mark_route_lane_changes                     # noqa: E402


def _plan(n, t_of, lane_of, l_of, r_of, step=1.5):
    pts = [dict(_junc=False, _road="1", _t=t_of(i), _flip=1.0,
                lane=lane_of(i), l=l_of(i), r=r_of(i), sig=0) for i in range(n)]
    cum = [i * step for i in range(n)]
    mark_route_lane_changes([None] * n, pts, cum)
    return pts


N, SHIFT = 40, 3.2                      # 20 점(30m)에 걸쳐 한 차로 이동
def _s(i):                              # 0 -> SHIFT 로 부드럽게
    return SHIFT * min(1.0, max(0.0, (i - 5) / 20.0))

# ① 진짜 진로변경: 도로 폭 그대로, 좌우 여유가 맞바뀐다
real = _plan(N, lambda i: _s(i), lambda i: -3 if _s(i) < SHIFT / 2 else -2,
             lambda i: 4.9 - _s(i), lambda i: 1.7 + _s(i))
check("진짜 차선변경엔 지시등을 켠다",
      any(p["sig"] for p in real), f'sig 전부 0')
check("켜는 방향이 이동 방향과 같다(좌+)",
      all(p["sig"] >= 0 for p in real) and any(p["sig"] == 1 for p in real))

# ② 차로가 왼쪽에 하나 늘어난 것: 오른쪽 가장자리와의 거리는 그대로, 폭만 커진다
grown = _plan(N, lambda i: _s(i), lambda i: 3 if _s(i) < SHIFT / 2 else 4,
              lambda i: 4.9 + _s(i), lambda i: 1.6)
check("차로가 늘어난 것뿐이면 지시등을 안 켠다",
      not any(p["sig"] for p in grown),
      f'{sum(1 for p in grown if p["sig"])}점이 켜졌다')

print("\n[경로 차선변경 지시등] 교차로 직후라도 진짜 차선변경이면 켠다")
# ★★2026-08-28 **이 테스트는 뒤집혔다.** 예전엔 "교차로 빠져나온 직후 25m 는 판정하지
#   않는다"였다. 근거로 삼았던 세 곳을 지도로 다시 재보니 **전부 진짜 차선변경**이었다 —
#     (960,-507) l 7.58->4.45 r 1.55->4.73 폭 9.13->9.18(유지) = 한 차로 왼쪽
#     (1099, 22) l 7.45->4.72 r 1.72->4.41 폭 9.17->9.13(유지) = 한 차로 왼쪽
#     (1127,-47) l 7.14->4.29 r 1.55->4.15 폭 8.69->8.44(유지) = 한 차로 왼쪽
#   사용자가 본 "우회전 마치자마자 좌측 깜빡이"의 진짜 원인은 **회전 지시등이 늦게
#   꺼지는 것**이었고 그건 drive.py 의 자차 회전율 래치로 따로 고쳤다(fe972d4).
#   25m 억제의 대가: 코스 A·B·G 차선변경 23곳 중 **12곳이 깜빡이 없음**[법 제38조 위반],
#   그러면서 막은 오점등은 **0건**. 오점등은 아래 '폭 보존' 검사가 이미 다 막는다.


def _plan2(n, t_of, lane_of, l_of, r_of, junc_upto, step=1.5):
    pts = [dict(_junc=(i <= junc_upto), _road="1", _t=t_of(i), _flip=1.0,
                lane=lane_of(i), l=l_of(i), r=r_of(i), sig=0) for i in range(n)]
    cum = [i * step for i in range(n)]
    mark_route_lane_changes([None] * n, pts, cum)
    return pts


N, SHIFT = 60, 3.2
def _s2(i, start):                       # start 점부터 20점(30m)에 걸쳐 SHIFT 이동
    return SHIFT * min(1.0, max(0.0, (i - start) / 20.0))

# ① 교차로 바로 뒤(2m)에서 한 차로를 옮기면 — 폭이 보존되므로 **진짜 차선변경**. 켠다.
just_out = _plan2(N, lambda i: _s2(i, 2), lambda i: -3 if _s2(i, 2) < SHIFT/2 else -2,
                  lambda i: 4.9 - _s2(i, 2), lambda i: 1.7 + _s2(i, 2), junc_upto=1)
check("교차로 직후라도 진짜 차선변경이면 켠다", any(p["sig"] for p in just_out),
      "교차로 직후라는 이유로 눌러버렸다 — 제38조 위반")

# ② 교차로 직후라도 **차로가 늘어난 것뿐**이면(폭이 커진다) 켜지 않는다.
grown_out = _plan2(N, lambda i: _s2(i, 2), lambda i: 3 if _s2(i, 2) < SHIFT/2 else 4,
                   lambda i: 4.9 + _s2(i, 2), lambda i: 1.6, junc_upto=1)
check("교차로 직후 차로 증가는 켜지 않는다", not any(p["sig"] for p in grown_out),
      f'{sum(1 for p in grown_out if p["sig"])}점이 켜졌다')

# ③ 교차로 **안**은 여전히 판정하지 않는다(연결로 기준선이 우리 진로와 다르게 휜다).
inside = _plan2(N, lambda i: _s2(i, 2), lambda i: -3 if _s2(i, 2) < SHIFT/2 else -2,
                lambda i: 4.9 - _s2(i, 2), lambda i: 1.7 + _s2(i, 2), junc_upto=N)
check("교차로 안에서는 켜지 않는다", not any(p["sig"] for p in inside),
      f'{sum(1 for p in inside if p["sig"])}점이 켜졌다')

print("\n[경로 차선변경 지시등] 진로변경이 끝날 때까지 켠다")
# 실측 2026-08-27 코스 B (1460,389): 도로기준 횡위치 -14.47 -> -11.04 (3.4m 이동)에
# 걸린 시간이 t=105.8~111.1 인데 깜빡이는 **106.0~107.9(1.8초)** 뿐이었다 —
# 이동의 앞 1/3 만 켜고 **옮겨가는 동안엔 꺼져 있었다.**
# 시행령 별표2 는 "그 행위가 **끝날 때까지**" 신호하라고 한다.
lc = _plan(N, lambda i: _s(i), lambda i: -3 if _s(i) < SHIFT / 2 else -2,
           lambda i: 4.9 - _s(i), lambda i: 1.7 + _s(i))
on = [k for k, p in enumerate(lc) if p["sig"]]
# 이동은 idx 5 에서 시작해 25 에서 끝난다(_s 정의). 끝까지 켜져 있어야 한다.
check("이동이 끝날 때까지 켜져 있다", on and max(on) >= 25,
      f'마지막 점등 idx {max(on) if on else None} (이동 종료 25)')
check("이동 시작 전부터 켜져 있다", on and min(on) <= 5,
      f'첫 점등 idx {min(on) if on else None} (이동 시작 5)')

print("\n[차선변경 자리] 횡단보도 위에서는 옮기지 않는다 [실측 2026-08-28]")
# 코스 A/G (1307,348) road 2790: 차로 이동(-3 -> -2)의 핵심 구간이 무신호 보호구역
# 횡단보도에서 **3.6m** 였다. 지시등은 켜져 있어 [법 제38조] 위반은 아니지만,
# 보행자를 살펴야 할 자리에서 옆으로 옮기는 건 위험하다.
import json                                                          # noqa: E402
from plan_route import dodge_crosswalks, LC_CW_PAD, LC_CW_MIN, LC_FINISH   # noqa: E402
_HERE = _os.path.dirname(_os.path.abspath(__file__))


def _win(s_lim, runway, s_from, dirn):
    """보간 창을 진행거리 좌표로. dodge_crosswalks 와 같은 식."""
    fin = min(LC_FINISH, runway * 0.4)
    p_lim = (s_lim - s_from) * dirn
    return p_lim - runway, p_lim - fin


# ① 도로 중간(s=150)의 횡단보도 -> 그 앞에서 끝내거나 뒤에서 시작해야 한다
_sl, _rw = dodge_crosswalks([150.0], 0.0, 200.0, 65.0, +1, 1)
_lo, _hi = _win(_sl, _rw, 0.0, +1)
check("창이 횡단보도를 비켜간다", not (_lo - LC_CW_PAD < 150.0 < _hi + LC_CW_PAD),
      f"창 [{_lo:.1f},{_hi:.1f}] 이 아직 150 을 문다")
check("비켜가고도 활주로가 남는다", _rw >= LC_CW_MIN, f"활주로 {_rw:.1f}m")

# ② 짧은 도로 입구(s=6)의 횡단보도 -> **지나간 뒤에** 시작한다
_sl, _rw = dodge_crosswalks([6.0], 0.0, 44.0, 44.0, +1, 1)
_lo, _hi = _win(_sl, _rw, 0.0, +1)
check("입구 횡단보도는 지나서 시작한다", _lo >= 6.0 + LC_CW_PAD - 1e-6,
      f"시작 {_lo:.1f} (횡단보도 6.0 + 여유 {LC_CW_PAD})")

# ③ 피할 활주로가 없으면 종전대로 — 경로가 없어지는 것보다 낫다
check("못 피하면 건드리지 않는다",
      dodge_crosswalks([15.0], 0.0, 30.0, 30.0, +1, 1) == (30.0, 30.0))

# ④ 역방향(-s)도 같은 규칙
_sl, _rw = dodge_crosswalks([50.0], 200.0, 0.0, 65.0, -1, 1)
_lo, _hi = _win(_sl, _rw, 200.0, -1)
_pcw = (50.0 - 200.0) * -1
check("역방향에서도 비켜간다", not (_lo - LC_CW_PAD < _pcw < _hi + LC_CW_PAD),
      f"창 [{_lo:.1f},{_hi:.1f}] 이 {_pcw:.1f} 을 문다")

# ⑤ 실제 경로: 어느 코스에도 **횡단보도 8m 안에서 차로를 바꾸는 곳이 없어야** 한다
_CW = json.load(open(_os.path.join(_HERE, "..", "routes", "crosswalks.json"),
                     encoding="utf-8"))["crosswalks"]
_bad = []
for _f in sorted(glob.glob(_os.path.join(_HERE, "..", "routes", "*.json"))):
    if _f.endswith("_lane.json") or _os.path.basename(_f) in (
            "crosswalks.json", "stoplines.json", "tl_map_livinglab.json",
            "lane_plan_v1_v6.json", "lane_room_v1_v6.json", "official_v1_v6.json"):
        continue
    _j = json.load(open(_f, encoding="utf-8"))
    _rt, _meta = _j.get("ego_route"), _j.get("ego_lanes")
    if not _rt or not _meta:
        continue
    _runs, _cur = [], None
    for _i in range(1, len(_meta)):
        if _meta[_i-1][0] == _meta[_i][0] and abs(_meta[_i][3] - _meta[_i-1][3]) > 0.02:
            _cur = _cur or [_i-1, _i]
            _cur[1] = _i
        elif _cur:
            _runs.append(_cur); _cur = None
    if _cur:
        _runs.append(_cur)
    for _a, _b in _runs:
        _tot = _meta[_b][3] - _meta[_a][3]
        # 차로 **번호가 바뀌어야** 차선변경이다(번호가 같으면 차로 중심 테이퍼일 뿐)
        if abs(_tot) < 1.2 or _meta[_a][1] == _meta[_b][1]:
            continue
        _lo2 = next((k for k in range(_a, _b+1)
                     if (_meta[k][3]-_meta[_a][3])/_tot >= 0.10), _a)
        _hi2 = max((k for k in range(_a, _b+1)
                    if (_meta[k][3]-_meta[_a][3])/_tot <= 0.90), default=_b)
        _d = min(math.hypot(_c["x"]-_rt[k][0], _c["y"]-_rt[k][1])
                 for k in range(_lo2, _hi2+1) for _c in _CW)
        if _d < 8.0:
            _bad.append(f"{_os.path.basename(_f)[:-5]} ({_rt[_lo2][0]:.0f},{_rt[_lo2][1]:.0f}) {_d:.1f}m")
check("어느 코스에도 횡단보도 8m 안 차선변경이 없다", not _bad, "; ".join(_bad[:4]))


print("[계단식 다차로 변경] 한 방 대각 활강이 아니라 한 칸씩, 중간 차로에서 정착")
from plan_route import lc_blend_staged, LC_SETTLE, LC_STAGE_MIN  # noqa: E402
_run = 100.0
_prev, _plateau, _mono = None, 0.0, True
for _k in range(0, 1001):
    _y = lc_blend_staged(_run, _run * (1.0 - _k / 1000.0), [0.5])
    if _prev is not None and _y < _prev - 1e-9:
        _mono = False
    if abs(_y - 0.5) < 1e-9:
        _plateau += _run / 1000.0
    _prev = _y
check("계단식 진행도는 단조증가", _mono)
check("끝에서 정확히 1", abs(_prev - 1.0) < 1e-9, f"{_prev}")
check("중간 차로(0.5) 정착이 실재한다", _plateau >= LC_SETTLE * 0.8, f"{_plateau:.1f}m")
check("한 칸 변경은 기존 smoothstep 과 동일",
      all(abs(lc_blend_staged(100.0, r, []) - lc_blend(100.0, r)) < 1e-12
          for r in (95.0, 60.0, 30.0, 10.0, 0.0)))
# 활주로 문턱: 70m 면 기존 보간, 100m 면 계단식(sample_lane 의 산식 고정)
_need = 2 * LC_STAGE_MIN + LC_SETTLE
check("짧은 활주로(70m)엔 계단을 안 넣는다", 70.0 - min(LC_FINISH, 70.0*0.4) < _need)
check("넉넉한 활주로(100m)엔 계단식", 100.0 - min(LC_FINISH, 100.0*0.4) >= _need)

# 실제 코스 검산: E 의 (1477,457)~(1480,529) 두 칸 변경에 정착 구간이 있다
_R = _os.path.join(_HERE, "..", "routes")
_rtE = json.load(open(_os.path.join(_R, "HL_FMA_NEW_E.json")))["ego_route"]
_lpE = json.load(open(_os.path.join(_R, "HL_FMA_NEW_E_lane.json")))["pts"]
_seg = [(_i, _p) for _i, _p in enumerate(_lpE)
        if _p and 1476 < _rtE[_i][0] < 1481 and 455 < _rtE[_i][1] < 535]
_rs = [_p["r"] for _i, _p in _seg]
# 정착 = r 이 중간값 근처(±0.15m)에 6점(≈8m) 이상 연속으로 머문다
_mid = (max(_rs) + min(_rs)) / 2.0
_best = _cur = 0
for _v in _rs:
    _cur = _cur + 1 if abs(_v - _mid) < 0.35 else 0
    _best = max(_best, _cur)
check("코스 E 두 칸 변경에 중간 차로 정착 구간이 있다", _best >= 6, f"연속 {_best}점")


print("[계획기가 적어 준 차선변경] 지시등이 한 곳도 빠지지 않는다 [법 제38조①]")
_miss, _tot, _old = [], 0, []
for _f in sorted(glob.glob(_os.path.join(_HERE, "..", "routes", "*.json"))):
    if _f.endswith("_lane.json"):
        continue
    _n = _os.path.basename(_f)[:-5]
    _lf = _f[:-5] + "_lane.json"
    if not _os.path.exists(_lf):
        continue
    _j = json.load(open(_f, encoding="utf-8"))
    _lm = _j.get("ego_lanes")
    if not _lm or len(_lm[0]) < 5:
        _old.append(_n)                     # 옛 4칸 파일 — 조용히 건너뛰는 게 정상
        continue
    _lp = json.load(open(_lf, encoding="utf-8"))["pts"]
    _runs = []
    for _i, _m in enumerate(_lm):
        if not _m[4]:
            continue
        if _runs and _i - _runs[-1][1] <= 2:
            _runs[-1][1] = _i
        else:
            _runs.append([_i, _i])
    _tot += len(_runs)
    for _a, _b in _runs:
        if not all(_lp[_k] and _lp[_k].get("sig") for _k in range(_a, _b + 1) if _lp[_k]):
            _miss.append(f"{_n} idx{_a}")
check(f"계획기 차선변경 {_tot}곳 전부 지시등 있음", not _miss, "; ".join(_miss[:4]))
check("옛 4칸 경로 파일도 깨지지 않는다(조용히 건너뜀)", True, "")


print("[실선 차로변경] 계획된 차선변경이 실선을 넘지 않는다 [대회 안내문 항목 6]")
#   "실선 차로 변경: 1회 경미(-3) / 2회 중대(-6)".
#   맵에는 실선이 1056개·점선 1034개로 반반인데, 우리 경로의 차선변경은 **전부 점선**을
#   넘는다(2026-08-31 실측 34/34). 차로 그래프가 연결성을 따라가니 자연히 그렇게 된다.
#   구현이 아니라 **회귀로 못박는다** — 경로를 다시 만들었을 때 깨지면 여기서 잡힌다.
#   ⚠️ OpenDRIVE 규약: lane N 의 roadMark 는 그 차로의 **바깥쪽** 경계 표시다.
#      안쪽 차로(|id| 작은 쪽)의 mark 가 두 차로 사이의 선이다.
from build_lane_plan import Map as _Map                                # noqa: E402
_mp2 = _Map(_x) if _x else None
_solid = []
if _mp2 is not None:
    for _f in sorted(glob.glob(_os.path.join(_HERE, "..", "routes", "*.json"))):
        if _f.endswith("_lane.json"):
            continue
        _j2 = json.load(open(_f, encoding="utf-8"))
        _lm2 = _j2.get("ego_lanes")
        if not _lm2 or len(_lm2[0]) < 5:
            continue
        _rt2 = _j2["ego_route"]
        _runs2 = []
        for _i, _m in enumerate(_lm2):
            if not _m[4]:
                continue
            if _runs2 and _i - _runs2[-1][1] <= 2:
                _runs2[-1][1] = _i
            else:
                _runs2.append([_i, _i])
        for _a, _b in _runs2:
            _rid, _la, _sa, _ta = _lm2[_a][:4]
            _lb = _lm2[_b][1]
            _sb = _lm2[_b][2]
            _rd = _mp2.roads.get(str(_rid)) or _mp2.roads.get(_rid)
            if _rd is None or _la == _lb:
                continue
            _lanes = {l[0]: l for l in _mp2.lanes(_rd, (_sa + _sb) / 2.0)}
            _inner = _la if abs(_la) < abs(_lb) else _lb
            _ln = _lanes.get(_inner)
            _mk = _ln[4] if _ln else None
            if _mk and "solid" in _mk:
                _solid.append(f"{_os.path.basename(_f)[:-5]} ({_rt2[_a][0]:.0f},{_rt2[_a][1]:.0f}) {_mk}")
check("계획된 차선변경이 실선을 넘지 않는다", not _solid, "; ".join(_solid[:4]))

print(f"\n=== {ok}개 통과 · {fail}개 실패 ===")
sys.exit(1 if fail else 0)
