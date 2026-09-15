#!/usr/bin/env python3
"""OpenDRIVE(.xodr)에서 **맵 전체 신호등의 정지선 좌표 DB**를 만든다.

★핵심 발견(2026-08-14 실측 검증):
  9910 DataPacket 의 `tl_id` = xodr 의 **controller id** 다. (signal id 아님!)
  controller 하나가 신호기 여러 개(보통 3개: 직진/좌회전/보행 등)를 묶고 있고,
  그 신호들의 기준선 위치가 곧 그 교차로 진입부(=정지선) 근처다.
  실측 정지선과 대조: tl30 1.6m, tl38 1.6m, tl74 0.6m, tl90 2.9m 오차 -> 충분히 정확.

왜 필요한가:
  지금까지 정지선 맵은 '직접 달려서' 만든 5개뿐인데 **대회 당일엔 새 경로가 주어진다.**
  맵(화성 리빙랩)은 고정이므로 전체를 미리 뽑아두면 어떤 경로가 와도 커버된다.
  정지선을 모르면 우리 로직은 안전측으로 '길 한복판 정지'(RED_STOP_BLIND)라 시간 손해가 크다.

★2026-09-04 보정: 신호 기준선 좌표를 **도색 정지선에 진행방향으로 스냅**한다.
  신호 여러 개의 중앙이라 원래도 진행방향 오차는 작았지만(중앙값 -0.15m), tl90 처럼
  2.44m 어긋난 곳이 있었다. 같은 도로 · 같은 t 부호 · |Δs|<=15m 인 도색 정지선을 찾아
  그 진행방향 위치로 옮긴다. **횡방향은 옛 좌표 그대로 둔다** — 넓은 도로는 차로마다
  정지선이 따로 도색돼 있어 평균을 내면 횡으로 10m 까지 밀리는데, 정지거리는
  진행방향 성분만 쓰므로(drive.py `_tl_stop_dist`) 횡을 건드릴 이유가 없다.
  손으로 실측한 5곳 진행방향 |오차| 합 3.14m -> 0.58m.

주의:
  - controller 요소는 정션 안에 '참조용 빈 controller'로도 나타난다(id 중복).
    반드시 <control> 자식이 있는 것만 유효한 정의로 취급할 것.
  - 신호의 횡오프셋 t 는 기둥이 길가에 있다는 뜻이므로 무시하고 기준선 위 점을 쓴다.
  - ⚠️ VTD 가 보고하는 tl_id 중 **xodr 에 controller 가 없는 것**이 있다(80·82 실측).
    이 DB 로는 못 메운다 -> drive.py 가 도색 정지선 DB(stoplines_all.json)로 폴백한다.

사용: python3 build_tl_map.py <xodr> [out.json] [--check route.json]
"""
import json
import math
import sys
import xml.etree.ElementTree as ET


def poly3_u(b, c, d, ds):
    """poly3 의 **호길이** ds -> 다항식 매개변수 u. v(u)=a+bu+cu²+du³ 이다.

    ⚠️ u = ds 로 두면 안 된다(원래 그랬다). OpenDRIVE 의 length 는 호길이인데 u 는
       기하 자기 축 방향 좌표라, 곡선이 휘는 만큼 u 가 앞서 나간다. 그 결과
       **레코드 경계에서 최대 2.83m 어긋났다**(맵 전체 실측 2026-08-16.
       line/arc/spiral 은 0.000m 인데 poly3 571개만 그랬다). 경로에서는 이게
       60~120° 헤딩 반전으로 나타나 '따라갈 수 없는 경로'가 된다.
       v1~v7 코스가 멀쩡했던 건 그 도로들의 휨이 작아 오차가 0.001m 수준이었기 때문.

    ds/du = sqrt(1 + v'(u)²) 를 심프슨으로 적분하고 뉴턴으로 뒤집는다.
    S'(u) >= 1 이라 반복이 안정적이고 4회면 마이크론 수준으로 수렴한다.
    """
    if abs(c) < 1e-12 and abs(d) < 1e-12:              # v 가 직선이면 닫힌 해
        return ds / math.hypot(1.0, b)

    def arclen(u):
        n, tot = 16, 0.0
        h = u / n
        for i in range(n + 1):
            t = i * h
            w = 1 if i in (0, n) else (4 if i % 2 else 2)
            tot += w * math.hypot(1.0, b + 2 * c * t + 3 * d * t * t)
        return tot * h / 3.0

    u = ds
    for _ in range(4):
        u = max(0.0, u + (ds - arclen(u)) / math.hypot(1.0, b + 2 * c * u + 3 * d * u * u))
    return u


def geom_point(g, ds):
    """geometry 레코드 시작에서 ds[m] 진행한 (x, y, hdg). line/arc/spiral/poly3 지원."""
    x0, y0 = float(g.get("x")), float(g.get("y"))
    h0 = float(g.get("hdg"))
    kind = next((c.tag for c in g), None)
    if kind == "arc":
        k = float(g.find("arc").get("curvature"))
        if abs(k) < 1e-9:
            return x0 + ds * math.cos(h0), y0 + ds * math.sin(h0), h0
        h = h0 + k * ds
        return (x0 + (math.sin(h) - math.sin(h0)) / k,
                y0 - (math.cos(h) - math.cos(h0)) / k, h)
    if kind == "spiral":
        sp = g.find("spiral")
        k0, k1 = float(sp.get("curvStart")), float(sp.get("curvEnd"))
        L = float(g.get("length")) or 1e-9
        n = max(8, int(ds / 0.5))                 # 0.5m 스텝 수치적분(클로소이드)
        x, y, h, step = x0, y0, h0, ds / max(1, n)
        for i in range(n):
            k = k0 + (k1 - k0) * ((i + 0.5) * step / L)
            hm = h + k * step * 0.5
            x += step * math.cos(hm)
            y += step * math.sin(hm)
            h += k * step
        return x, y, h
    if kind == "poly3":
        p = g.find("poly3")
        a, b, c, d = (float(p.get(k)) for k in ("a", "b", "c", "d"))
        u = poly3_u(b, c, d, ds)
        v = a + b * u + c * u * u + d * u ** 3
        dv = b + 2 * c * u + 3 * d * u * u
        return (x0 + u * math.cos(h0) - v * math.sin(h0),
                y0 + u * math.sin(h0) + v * math.cos(h0), h0 + math.atan(dv))
    return x0 + ds * math.cos(h0), y0 + ds * math.sin(h0), h0


def road_point(road, s):
    """도로 기준선의 종방향 s[m] 지점 (x, y, hdg)."""
    geoms = road.findall("./planView/geometry")
    if not geoms:
        return None
    g = geoms[0]
    for cand in geoms:
        if float(cand.get("s")) <= s:
            g = cand
        else:
            break
    return geom_point(g, max(0.0, s - float(g.get("s"))))


STOPLINE = "Rm_StopLine_300cm_JPN_01"      # 맵의 도색 정지선 오브젝트 이름
SNAP_DS = 15.0                            # 신호에서 이 안의 도색선만 그 신호의 것으로 본다


def build(xodr):
    """{tl_id(controller id): [x, y]} — 신호들의 중앙을 도색 정지선에 진행방향 스냅."""
    root = ET.parse(xodr).getroot()
    sigpos = {}          # signalId -> (road_id, s, t, x, y)
    lines = {}           # road_id  -> [(s, t, x, y, hdg)]  도색 정지선
    for road in root.iter("road"):
        rid, rlen = road.get("id"), float(road.get("length", 0))
        for sig in road.findall("./signals/signal"):
            ss = min(float(sig.get("s", 0)), rlen)
            p = road_point(road, ss)
            if p:
                sigpos[sig.get("id")] = (rid, ss, sig.get("orientation", "none"),
                                         p[0], p[1])
        for o in road.findall("objects/object"):
            if (o.get("name") or "").split(".")[0] != STOPLINE:
                continue
            ls = min(max(float(o.get("s", 0)), 0.0), max(0.0, rlen))
            lt = float(o.get("t", 0.0))
            try:
                x, y, h = road_point(road, ls)
            except Exception:
                continue
            # 오브젝트 hdg 는 0(=+s 방향 차량용) 아니면 pi(=-s 방향 차량용) 다.
            ohd = float(o.get("hdg", 0.0))
            lines.setdefault(rid, []).append(
                (ls, "-" if abs(math.cos(ohd) + 1.0) < 0.5 else "+",
                 x - lt * math.sin(h), y + lt * math.cos(h), h + ohd))

    tl, snapped = {}, 0
    for c in root.findall(".//controller"):
        sids = [x.get("signalId") for x in c.findall("./control")]
        if not sids:                       # 정션 안 참조용 빈 controller 는 무시
            continue
        mine = [sigpos[s] for s in sids if s in sigpos]
        if not mine:
            continue
        ox = sum(p[3] for p in mine) / len(mine)
        oy = sum(p[4] for p in mine) / len(mine)

        # ★도색 정지선으로 진행방향만 보정. 같은 도로 · **같은 진행방향** · |Δs| <= SNAP_DS.
        #   ⚠️ 방향은 신호의 `orientation`(+ = +s 진행 차량용) 과 도색선의 hdg 로 가른다.
        #      예전에 t 부호로 갈랐더니 tl104 에서 **반대 방향 정지선**에 붙어 13.4m 를
        #      잘못 옮겼다(신호는 road 2116 s=13.5 의 +s 용, 도색선은 s=0.15 의 -s 용).
        cand = []
        for rid, ss, so, _sx, _sy in mine:
            for (ls, lo, lx, ly, lh) in lines.get(rid, []):
                if so in ("+", "-") and lo != so:
                    continue               # 반대 방향 차량용 선
                if abs(ls - ss) > SNAP_DS:
                    continue
                cand.append((abs(ls - ss), ls, lx, ly, lh))
        if cand:
            cand.sort()
            s0 = cand[0][1]
            grp = [q for q in cand if abs(q[1] - s0) < 1.0]   # 같은 자리의 차로별 도색
            h = grp[0][4]
            e = sum((q[2] - ox) * math.cos(h) + (q[3] - oy) * math.sin(h)
                    for q in grp) / len(grp)
            ox, oy = ox + e * math.cos(h), oy + e * math.sin(h)
            snapped += 1

        tl[str(int(c.get("id")))] = [round(ox, 2), round(oy, 2)]
    return tl, len(sigpos), snapped


def main():
    xodr = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith("--") else "tl_map.json"
    tl, nsig, snapped = build(xodr)
    json.dump(tl, open(out, "w", encoding="utf-8"), indent=0)
    print(f"signal {nsig}개 -> controller(=tl_id) {len(tl)}개 정지선 DB 저장: {out}")
    print(f"  그중 {snapped}개는 도색 정지선에 진행방향 스냅, {len(tl)-snapped}개는 계산값 그대로")

    if "--check" in sys.argv:
        ref = sys.argv[sys.argv.index("--check") + 1]
        meas = json.load(open(ref, encoding="utf-8")).get("tl_stops", {})
        print(f"\n실측 {len(meas)}개와 대조:")
        for k, v in sorted(meas.items(), key=lambda x: int(x[0])):
            c = tl.get(k)
            if c is None:
                print(f"  tl{k:>4}: DB에 없음")
                continue
            print(f"  tl{k:>4}: 오차 {math.hypot(c[0]-v[0], c[1]-v[1]):5.1f}m")


if __name__ == "__main__":
    main()
