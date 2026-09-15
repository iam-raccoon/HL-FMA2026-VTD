#!/usr/bin/env python3
"""아무 코스에나 **돌발상황**을 심는다 -> `scenarios/<코스>_HZ.xml`.

사용자 요청 2026-08-22: 사람이 튀어나옴 / 트럭·다른 차의 신호위반·역주행 / 다른 차량 운행.
오프라인 판(eval/scenarios/hz_*.json)으로 먼저 만들어 **실제 결함 두 개**를 잡았고
(경로투영 clamp, 정면 접근을 앞차로 오인), 이건 그걸 VTD 에서 다시 보기 위한 것이다.

심는 것(코스 길이 비율로 자리를 잡아 서로 간섭하지 않게 한다):
    0.15  Lead      앞차 — 제한속도로 정상 주행(헛브레이크·헛추월 검사)
    0.22  Rear      뒤차 — **경로 시작 뒤쪽이 아니라 진행 중** 뒤에 둔다
    0.32  WrongWay  역주행 — 내 차로 중심, 마주보는 방향으로 접근
    0.55  RedRun    신호위반 — 교차로 옆에서 대기하다 내가 접근하면 가로지른다
    0.70  Oncoming  대향차 — 반대 차로 정상 주행(무시해야 한다)
    0.85  Jay       무단횡단 보행자 — 차로 가장자리(DummyPerson)

    python3 vtd/add_hazards.py <xodr> <코스이름>
      예: python3 vtd/add_hazards.py $X HL_FMA_NEW_A  -> scenarios/HL_FMA_NEW_A_HZ.xml

⚠️⚠️ **트리거 거리를 크게 잡으면 안 된다.** 실측 2026-08-24 첫 VTD 판(781s 완주):
   Lead·Rear 를 `trig=400m` 로 뒀더니 시나리오 초반에 트리거가 걸려 **그대로 달려가
   사라졌다** — 자차가 그 자리에 도착했을 땐 obj=0 이었다. VTD 액터는 Control="internal"
   + DefaultDriver 라 경로를 안 주면 **도로망을 따라 제멋대로 간다**. 트리거는 자차가
   충분히 가까울 때만 걸리도록 짧게(45~90m) 잡아야 마주친다.
   (`_EV` 가 잘 되던 건 거기 정지차가 speed=0 이라 아예 안 움직였기 때문이다.)
⚠️ 역주행차는 `AlignToRoad="false"` 로 놓는다. true 면 VTD 가 **도로 방향으로 정렬**해
   우리가 준 반대 방위를 되돌려버린다.

⚠️ add_events.py 에서 배운 것 두 가지를 그대로 지킨다:
   · 대향/역주행차는 **차로 중심**에 놓는다(경로점 위에 역방향으로 놓으면 VTD 플레이어가
     충돌을 무시하고 뚫고 지나간다 — 판이 무의미해진다).
   · 막는 차는 **교차로 밖 + 옆 여유가 있는 곳**에 놓는다(교차로 안이면 자차가 갇힌다).
⚠️ 보행자는 Player 로 스폰되지 않는다(Type="Hannah" 실패). MovingObjectsControl 오브젝트로
   넣는다. 그래서 **움직이지 않는다** — '뛰어드는' 검증은 오프라인 판이 담당한다.
"""
import json
import math
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from add_oncoming import add                                        # noqa: E402
from build_lane_plan import Map                                     # noqa: E402
from plan_route import locate_lanes_all, sample_lane                # noqa: E402

HERE = __file__.rsplit("/", 1)[0]
PED_DEF = "DummyPerson"
# ⚠️ elevationProfile 로 낸 노면 고도에 이만큼 올려야 오브젝트가 지면에 앉는다.
#    검증(2026-08-22): 공식 v1 s=120m 의 계산 고도 42.09 인데, 그 자리에서 실제로
#    보이던 기름통의 Z 가 42.83 이었다(make_event_scenarios 의 GROUND_Z 실측값).
#    Z 를 그냥 노면으로 주면 반쯤 묻혀 안 보인다.
OBJ_Z_LIFT = 0.74
NEED_ROOM = 3.2


def cumdist(rt):
    c = [0.0]
    for i in range(1, len(rt)):
        c.append(c[-1] + math.dist(rt[i], rt[i - 1]))
    return c


def idx_at(cum, frac):
    tgt = cum[-1] * frac
    return min(range(len(cum)), key=lambda i: abs(cum[i] - tgt))


def hdg(rt, i):
    j = min(i + 3, len(rt) - 1)
    return math.atan2(rt[j][1] - rt[i][1], rt[j][0] - rt[i][0])


def elevation(rd, s):
    """도로 rd 의 s 지점 고도. 없으면 0."""
    best = None
    for e in rd.findall("elevationProfile/elevation"):
        es = float(e.get("s", 0))
        if es <= s + 1e-6 and (best is None or es > float(best.get("s", 0))):
            best = e
    if best is None:
        return 0.0
    ds = s - float(best.get("s", 0))
    return (float(best.get("a", 0)) + float(best.get("b", 0)) * ds
            + float(best.get("c", 0)) * ds ** 2 + float(best.get("d", 0)) * ds ** 3)


def opposite_lane_point(mp, x, y):
    """그 자리의 **반대 차로 중심** 좌표. 없으면 None."""
    c = locate_lanes_all(mp, x, y)
    if not c:
        return None
    rid, lid, s = c[0]
    rd = mp.roads[rid]
    lanes = set()
    for sec in rd.iter("laneSection"):
        for side in ("left", "right"):
            el = sec.find(side)
            if el is None:
                continue
            for ln in el.iter("lane"):
                if ln.get("type") == "driving":
                    lanes.add(int(ln.get("id")))
    opp = [l for l in lanes if (l > 0) != (lid > 0)]
    if not opp:
        return None
    out = []
    L = float(rd.get("length"))
    sample_lane(mp, rid, opp[0], max(0.0, s - 1), min(L, s + 1), out)
    return out[len(out) // 2] if out else None


def junction_runs(lp):
    """경로 위 교차로 구간 [시작idx, 끝idx] 목록."""
    runs = []
    for i, p in enumerate(lp):
        if p and p.get("j"):
            if runs and i - runs[-1][1] <= 4:
                runs[-1][1] = i
            else:
                runs.append([i, i])
    return runs


def pick_cross(mp, rt, lp, cum, frac):
    """frac 이후 교차로들을 훑어 **교차 도로가 실제로 있는** 첫 곳을 고른다.

    ⚠️ 교차로라고 다 십자로가 아니다 — 이 맵의 junction 대부분은 단순 연결로다.
       실측 2026-08-22 코스 A: 경로 위 교차로 30곳 중 교차 도로를 찾은 건 일부뿐이라,
       frac 한 곳만 보고 포기하면 신호위반차를 못 심는다.
    """
    import contextlib
    import io
    start = idx_at(cum, frac)
    for a, b in junction_runs(lp):
        m = (a + b) // 2
        if m < start:
            continue
        with contextlib.redirect_stderr(io.StringIO()):     # locate_lanes_all 경고 억제
            cp = cross_road_point(mp, rt[m][0], rt[m][1], hdg(rt, m))
        if cp:
            return m, cp
    return None


def lane_point_back(mp, x, y, back):
    """(x,y) 가 속한 차로를 따라 **back[m] 뒤**의 차로 중심. 없으면 None.

    ⚠️ 헤딩으로 직선 외삽하면 안 된다 — 도로가 굽으면 차로 밖으로 나간다
       (실측 2026-08-22: 경로 시작에서 35m 직선 외삽 -> 차로에서 12m 벗어남).
    """
    c = locate_lanes_all(mp, x, y)
    if not c:
        return None
    rid, lid, s0 = c[0]
    rd = mp.roads[rid]
    L = float(rd.get("length"))
    s = s0 - back if lid < 0 else s0 + back      # 우측차로(-)는 s 증가방향 주행
    if not (0.0 <= s <= L):
        return None
    out = []
    sample_lane(mp, rid, lid, max(0.0, s - 1.0), min(L, s + 1.0), out)
    return out[len(out) // 2] if out else None


def cross_road_point(mp, jx, jy, h, rmin=12.0, rmax=30.0):
    """교차로 (jx,jy) 로 **가로질러 들어오는** 차로 위의 한 점과 진입 방위.

    교차로 옆 20m 를 그냥 찍으면 도로 밖이기 십상이다(실측: 차로까지 24m).
    반경을 넓혀 가며 차로를 찾고, **우리 진행방향과 60도 이상 어긋난** 차로만 쓴다.
    """
    best = None
    for r in (rmin, rmin + 6, rmin + 12, rmax):
        for a in range(0, 360, 15):
            th = math.radians(a)
            px, py = jx + r * math.cos(th), jy + r * math.sin(th)
            c = locate_lanes_all(mp, px, py)
            if not c:
                continue
            rid, lid, s = c[0]
            rd = mp.roads[rid]
            L = float(rd.get("length"))
            out = []
            sample_lane(mp, rid, lid, max(0.0, s - 1.0), min(L, s + 1.0), out)
            if len(out) < 2:
                continue
            q = out[len(out) // 2]
            if math.hypot(q[0] - px, q[1] - py) > 3.0:
                continue                     # 차로 중심에서 먼 자리 = 도로 밖
            # 그 차로에서 교차로 쪽을 향하는 방위
            toward = math.atan2(jy - q[1], jx - q[0])
            if abs(math.cos(toward - h)) > 0.5:
                continue                     # 우리와 나란한 도로 -> 교차 아님
            best = (q[0], q[1], math.degrees(toward) % 360.0)
            return best
    return best


def add_ped(in_xml, out_xml, mp, x, y, h):
    """MovingObjectsControl 에 보행자 오브젝트 하나를 넣는다."""
    c = locate_lanes_all(mp, x, y)
    z = (elevation(mp.roads[c[0][0]], c[0][2]) + OBJ_Z_LIFT) if c else 0.0
    src = open(in_xml, encoding="utf-8").read()
    obj = (f'<Object Type="other" Name="Jay" Definition="{PED_DEF}">'
           f'<StartPosAbs X="{x:.16e}" Y="{y:.16e}" Z="{z:.16e}" '
           f'Direction="{(h + math.pi / 2) % (2 * math.pi):.16e}" Pitch="0.0" Roll="0.0"/>'
           f'</Object><ObjectActions Object="Jay"/>')
    if "<MovingObjectsControl />" in src:
        src = src.replace("<MovingObjectsControl />",
                          f"<MovingObjectsControl>{obj}</MovingObjectsControl>", 1)
    elif "</MovingObjectsControl>" in src:
        src = src.replace("</MovingObjectsControl>", obj + "</MovingObjectsControl>", 1)
    else:
        print("⚠️ MovingObjectsControl 앵커가 없다 — 보행자 생략", file=sys.stderr)
    open(out_xml, "w", encoding="utf-8").write(src)
    return z


def main():
    xodr, name = sys.argv[1], sys.argv[2]
    rt = json.load(open(f"{HERE}/../routes/{name}.json", encoding="utf-8"))["ego_route"]
    lp = json.load(open(f"{HERE}/../routes/{name}_lane.json", encoding="utf-8"))["pts"]
    mp = Map(xodr)
    cum = cumdist(rt)
    src = f"{HERE}/../scenarios/{name}.xml"
    out = f"{HERE}/../scenarios/{name}_HZ.xml"
    cur, tmp, n = src, "/tmp/_hz_%d.xml", 0
    made = []

    def place(x, y, hd_deg, speed, trig, nm, align=True):
        nonlocal cur, n
        n += 1
        dst = tmp % n
        add(cur, dst, x, y, hd_deg, speed, trig, name=nm, align=align)
        cur = dst
        made.append(nm)

    # ① 앞차 — 제한속도로 정상 주행
    i = idx_at(cum, 0.15)
    place(rt[i][0], rt[i][1], math.degrees(hdg(rt, i)) % 360, 8.0, 70.0, "Lead")

    # ② 뒤차 — **경로 시작 35m 뒤**에 둔다. 이게 2026-08-22 에 찾은 버그의 재현이다:
    #    경로 밖(시작보다 뒤)의 차가 s=0 d=0 으로 자차와 겹쳐 투영돼 NARROW_BLOCK 이
    #    걸리고 **출발조차 못 했다**. 자차(30~50km/h)보다 느린 7m/s 라 절대 못 따라잡는다.
    rp = lane_point_back(mp, rt[0][0], rt[0][1], 35.0)
    if rp:
        place(rp[0], rp[1], math.degrees(hdg(rt, 0)) % 360, 7.0, 45.0, "Rear")
    else:
        print("⚠️ 경로 시작 뒤쪽 차로를 못 찾았다 — 뒤차 생략", file=sys.stderr)

    # ③ 역주행 — 내 차로 중심에 마주보게. 트리거를 길게 잡아 멀리서부터 다가오게 한다.
    i = idx_at(cum, 0.32)
    place(rt[i][0], rt[i][1], math.degrees(hdg(rt, i) + math.pi) % 360, 6.0, 90.0, "WrongWay",
          align=False)

    # ④ 신호위반 — 교차로 진입점 옆 20m 에서 대기하다 가로지른다.
    jj = pick_cross(mp, rt, lp, cum, 0.45)
    if jj:
        jidx, cp = jj
        place(cp[0], cp[1], cp[2], 8.0, 45.0, "RedRun")
    else:
        print("⚠️ 교차 도로가 있는 교차로를 못 찾았다 — 신호위반차 생략", file=sys.stderr)

    # ⑤ 대향차 — 반대 차로 정상 주행(무시해야 한다)
    i = idx_at(cum, 0.70)
    p = opposite_lane_point(mp, rt[i][0], rt[i][1])
    if p:
        place(p[0], p[1], math.degrees(hdg(rt, i) + math.pi) % 360, 8.0, 120.0, "Oncoming")
    else:
        print("⚠️ 반대 차로를 못 찾았다 — 대향차 생략", file=sys.stderr)

    # ⑥ 무단횡단 보행자 — 차로 가장자리(움직이지 않는다)
    i = idx_at(cum, 0.85)
    h = hdg(rt, i)
    z = add_ped(cur, out, mp,
                rt[i][0] + 1.6 * math.cos(h + math.pi / 2),
                rt[i][1] + 1.6 * math.sin(h + math.pi / 2), h)
    made.append(f"Jay(Z={z:.1f})")
    print(f"-> {out}\n   심은 것: " + " · ".join(made))


if __name__ == "__main__":
    main()
