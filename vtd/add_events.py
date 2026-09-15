#!/usr/bin/env python3
"""아무 코스에나 **동적 이벤트**(내 차로 정지차 + 대향 직진차)를 심는다.

왜 필요한가(2026-08-20): `make_course.py` 로 만든 새 코스에는 **상대차가 하나도 없다.**
그래서 처음 보는 도로에서 검증한 게 경로추종뿐이고 '모르는 길에서 위험 상황'은
한 번도 안 해봤다. 공식 코스에는 `make_event_scenarios.py` 로 이벤트를 심었지만
그건 v5 코스 전용이다.

    python3 vtd/add_events.py <xodr> <코스이름>
      예: python3 vtd/add_events.py $X HL_FMA_NEW_G
      -> scenarios/HL_FMA_NEW_G_EV.xml

⚠️⚠️ 자리를 아무 데나 잡으면 **판이 불공정해진다.** 실측 2026-08-20에 둘 다 겪었다:
  · 대향차를 **경로점 위**(=우리 차로)에 역방향으로 놨더니 정면충돌이 났다. 자차는
    정확히 멈췄는데 VTD 내부 플레이어가 충돌을 무시하고 뚫고 지나간 것이다.
    -> 대향차는 반드시 `sample_lane` 로 **반대 차로 중심**에 놓는다.
  · 정지차를 **교차로 안**(j=1, 좌우 여유 1.42m)에 놨더니, 자차가 갇혔다가
    `stuck_long` 규칙으로 3.5m 를 비켜 나가 리스폰 3건이 났다.
    -> 정지차는 **교차로 밖 + 비킬 여유가 있는 곳**에 놓는다.
"""
import math
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from add_oncoming import add                                        # noqa: E402
from build_lane_plan import Map                                     # noqa: E402
from plan_route import locate_lanes_all, sample_lane                # noqa: E402

BLOCK_AT = 0.20      # 코스의 이 지점 이후에서 정지차 자리를 찾는다(0~1)
ONCOME_AT = 0.45     # 대향차는 이 지점 부근
NEED_ROOM = 3.2      # 정지차를 놓을 곳에 필요한 옆 여유[m] — 이보다 좁으면 추월이 못 한다


def cumdist(rt):
    c = [0.0]
    for i in range(1, len(rt)):
        c.append(c[-1] + math.dist(rt[i], rt[i - 1]))
    return c


def pick_blocker(rt, lp, cum):
    """교차로 밖 + 옆 여유가 있는 자리. 없으면 None."""
    start = min(range(len(rt)), key=lambda i: abs(cum[i] - cum[-1] * BLOCK_AT))
    for i in range(start, len(rt) - 20):
        p = lp[i] if i < len(lp) else {}
        if p.get("j") or max(p.get("l", 0.0), p.get("r", 0.0)) < NEED_ROOM:
            continue
        h = math.atan2(rt[i + 3][1] - rt[i][1], rt[i + 3][0] - rt[i][0])
        return rt[i][0], rt[i][1], math.degrees(h) % 360.0
    return None


def pick_oncoming(mp, rt, cum):
    """반대 차로 중심 + 우리와 마주보는 heading. 없으면 None."""
    start = min(range(len(rt)), key=lambda i: abs(cum[i] - cum[-1] * ONCOME_AT))
    for i in range(start, len(rt) - 20):
        c = locate_lanes_all(mp, rt[i][0], rt[i][1])
        if not c:
            continue
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
            continue
        out = []
        L = float(rd.get("length"))
        sample_lane(mp, rid, opp[0], max(0.0, s - 1), min(L, s + 1), out)
        if not out:
            continue
        p = out[len(out) // 2]
        h = math.atan2(rt[i + 3][1] - rt[i][1], rt[i + 3][0] - rt[i][0])
        return p[0], p[1], math.degrees(h + math.pi) % 360.0
    return None


def main():
    import json
    xodr, name = sys.argv[1], sys.argv[2]
    here = __file__.rsplit("/", 1)[0]
    rt = json.load(open(f"{here}/../routes/{name}.json", encoding="utf-8"))["ego_route"]
    lp = json.load(open(f"{here}/../routes/{name}_lane.json", encoding="utf-8"))["pts"]
    mp = Map(xodr)
    cum = cumdist(rt)

    src = f"{here}/../scenarios/{name}.xml"
    out = f"{here}/../scenarios/{name}_EV.xml"
    tmp = "/tmp/_ev_stage.xml"

    b = pick_blocker(rt, lp, cum)
    if b is None:
        raise SystemExit(f"{name}: 정지차를 놓을 자리(교차로 밖 + 여유 {NEED_ROOM}m)가 없다")
    add(src, tmp, b[0], b[1], b[2], 0.0, 10.0, name="Blocker")

    o = pick_oncoming(mp, rt, cum)
    if o is None:
        print(f"⚠️ {name}: 반대 차로를 못 찾았다 — 정지차만 넣는다", file=sys.stderr)
        import shutil
        shutil.copy(tmp, out)
    else:
        add(tmp, out, o[0], o[1], o[2], 8.0, 60.0, name="Oncoming")
    print(f"-> {out}")


if __name__ == "__main__":
    main()
