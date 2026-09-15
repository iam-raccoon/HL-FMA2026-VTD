#!/usr/bin/env python3
"""경로 파일 자체를 검사한다 — **주행해보기 전에** 못 쓸 경로를 걸러내려고.

왜 만들었나(2026-08-16): 지도에서 계산한 경로가 녹화본과 중앙값 0.73m 로 겹쳐서
"됐다"고 판단했는데, 실제로 주행하니 **리스폰 81건 · 신호위반 5건 · 1386m 헤맴**으로
완주도 못 했다. 형상이 겹치는 것과 따라갈 수 있는 것은 다르다.
리스폰 로그가 원인을 말해줬다: `0.1m/48°` — 거의 안 움직였는데 헤딩이 48° 튄다.
경로 중간에 **진행 방향이 뒤집힌 지점**이 있었던 것.

검사 항목
  ① 헤딩 반전   : 연속 구간의 방위가 급격히 꺾임 (이번 실패의 직접 원인)
  ② 점 간격     : 너무 촘촘하거나(중복) 너무 벌어짐(구멍)
  ③ 도로 밖     : 주행 차로 안에 없는 점
  ④ 역주행      : 그 차로의 통행 방향과 반대로 진행
  ⑤ 곡률        : 차가 낼 수 없는 조향을 요구하는 구간

사용: python3 check_route.py <route.json> [xodr]      (xodr 없으면 ①②⑤만)
"""
import json
import math
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0] + "/../vtd")

REV_DEG = 60.0        # 연속 구간 방위가 이 이상 꺾이면 이상(1.4m 간격 기준)
GAP_MAX = 5.0         # 점 간격 상한[m]
GAP_MIN = 0.05        # 점 간격 하한[m]
WHEELBASE = 2.95
STEER_MAX = math.radians(35)


def is_real_junction(mp, jid):
    """갈래가 3개 이상이어야 **교차로**다. 2개짜리는 그냥 모퉁이(굽은 길)다.

    ★주행기(`src/drive.py`)가 회전 지시등·양보·일시정지에 쓰는 기준(`jx = 갈래>=3`,
      `build_lane_plan` 이 심는다)과 **같은 규칙**이다. 안 맞추면 채점기와 주행기가
      서로 다른 답을 내고, 그러면 둘 다 못 믿는다.

    실측 2026-09-04 사전테스트1 — 이 규칙이 없어서 **없는 위반**이 나왔다:
      junction 91(2814 <-> 2813)은 진입도로 2개, 연결로도 방향당 1개,
      차로도 1:1(2->-2 … 6->-6). 반경 60m 안에 다른 도로가 없다 —
      **교차 도로 자체가 없어 '좌회전' 이란 게 존재하지 않는다.**
      그런데 맵은 거기 좌/직/우 화살표를 그려 뒀고(RM_539_LUT·537_ST·538_SRT),
      연결로 기하가 87° 휘어 있어 `turn_kind` 가 'left' 를 준다.
      그 둘을 맞대니 "직진 차로에서 좌회전" 이 나왔다. 우회로 판정까지 붙어
      "lane +2 로 갔어야 한다(우리 잘못)" 라고 찍혔는데, lane +2 도 같은 연결로로
      같은 데를 간다. 거기로 옮기는 건 **의미 없는 차선변경**일 뿐이다.

    전 경로 대조(2026-09-04): 이 규칙으로 걸러지는 건 5곳 중 2곳이고 **둘 다 그 자리**다.
    갈래 3 이상인 3곳(코스 A 1 · wp9 2)은 그대로 남는다.
    """
    return mp.junc_arms.get(jid, 0) >= 3


def runs(idx, gap=2):
    """인덱스 목록을 연속 구간으로 묶는다."""
    if not idx:
        return []
    out, s, p = [], idx[0], idx[0]
    for i in idx[1:]:
        if i - p > gap:
            out.append((s, p)); s = i
        p = i
    out.append((s, p))
    return out


def main():
    rt = json.load(open(sys.argv[1], encoding="utf-8"))["ego_route"]
    n = len(rt)
    cum = [0.0]
    for i in range(1, n):
        cum.append(cum[-1] + math.dist(rt[i], rt[i-1]))
    print(f"경로 {n}점  {cum[-1]:.0f}m")

    bad = 0

    # ② 점 간격
    gaps = [math.dist(rt[i], rt[i-1]) for i in range(1, n)]
    big = [i for i, g in enumerate(gaps, 1) if g > GAP_MAX]
    tiny = [i for i, g in enumerate(gaps, 1) if g < GAP_MIN]
    print(f"\n[② 점 간격] 중앙값 {sorted(gaps)[len(gaps)//2]:.2f}m  "
          f"최대 {max(gaps):.2f}m  |  {GAP_MAX}m 초과 {len(big)}개 · {GAP_MIN}m 미만 {len(tiny)}개"
          + ("  ← ❌" if big or tiny else "  ✅"))
    for a, b in runs(big)[:3]:
        print(f"    s={cum[a]:.0f}m ({rt[a][0]:.0f},{rt[a][1]:.0f})")
    bad += len(big) + len(tiny)

    # ① 헤딩 반전
    hd = [math.atan2(rt[i][1]-rt[i-1][1], rt[i][0]-rt[i-1][0]) for i in range(1, n)]
    rev = []
    for i in range(1, len(hd)):
        d = abs((hd[i] - hd[i-1] + math.pi) % (2*math.pi) - math.pi)
        if math.degrees(d) > REV_DEG:
            rev.append(i)
    print(f"[① 헤딩 반전] {REV_DEG:.0f}° 초과 {len(rev)}곳"
          + ("  ← ❌ 이런 경로는 못 따라간다" if rev else "  ✅"))
    for a, b in runs(rev)[:6]:
        d = max(abs((hd[i]-hd[i-1]+math.pi) % (2*math.pi) - math.pi) for i in range(a, b+1))
        print(f"    s={cum[a]:.0f}m ({rt[a][0]:.0f},{rt[a][1]:.0f})  최대 {math.degrees(d):.0f}°")
    bad += len(rev)

    # ⑤ 곡률
    steer = []
    for i in range(1, len(hd)):
        ds = math.dist(rt[i+1], rt[i]) if i+1 < n else 0.0
        if ds < 0.2:
            continue
        dh = abs((hd[i] - hd[i-1] + math.pi) % (2*math.pi) - math.pi)
        k = dh / ds
        if math.atan(k * WHEELBASE) > STEER_MAX:
            steer.append(i)
    print(f"[⑤ 곡률] 조향 한계(35°) 초과 {len(steer)}곳"
          + ("  ← ⚠" if steer else "  ✅"))
    for a, b in runs(steer)[:3]:
        print(f"    s={cum[a]:.0f}m ({rt[a][0]:.0f},{rt[a][1]:.0f})")

    if len(sys.argv) < 3:
        print(f"\n=== 이상 {bad}곳 (xodr 없이 ①②⑤만 검사) ===")
        return

    # ③④ 지도 대조
    from build_lane_plan import Map
    mp = Map(sys.argv[2])
    from build_lane_plan import CELL

    def on_lane(px, py):
        """이 점을 담는 **모든** 주행차로 [(중앙거리, road, lane, 기준선헤딩)].

        ⚠️ 교차로는 연결로가 서로 겹쳐서 '가장 가까운 도로 하나'로 판정하면
           주행 검증된 경로에서도 27점이 '도로 밖'으로 나온다(실측). 후보를 다 봐야 한다.
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
            t = (px - x) * -math.sin(h) + (py - y) * math.cos(h)
            for lid, typ, lo, hi, _m in mp.lanes(rd, s):
                if typ != "driving" or not (lo - 1e-6 <= t <= hi + 1e-6):
                    continue
                off_c = abs(t - (lo + hi) / 2.0)
                key = (rd.get("id"), lid)
                if key not in found or off_c < found[key][0]:
                    found[key] = (off_c, rd, lid, h)
        return sorted(found.values(), key=lambda z: z[0])

    # ⚠️ on_lane 은 격자 탐색이라 비싸다. 점마다 **한 번만** 부르고 ③④⑥⑧ 이 나눠 쓴다.
    all_c = [on_lane(px, py) for px, py in [(r[0], r[1]) for r in rt]]

    off, wrong = [], []
    for i, cands in enumerate(all_c):
        if not cands:
            off.append(i); continue
        j = min(i, len(hd) - 1)
        # ★후보 **하나라도** 우리 진행 방향을 허용하면 역주행이 아니다.
        #   '가장 가까운 것 하나'로 보면 안 된다 — 교차로에서 연결로 6~8개가 같은 점을
        #   담고 있어 어느 걸 집느냐로 판정이 뒤집힌다(실측: 정상 경로에서 3점 오탐).
        #   교차로 연결로는 애초에 통행방향 판정을 보류하므로 '허용'으로 친다.
        ok = False
        for _off, rd, lid, h in cands:
            if rd.get("junction", "-1") != "-1":
                ok = True; break
            fwd = math.cos(hd[j] - h) > 0  # 도로 s 방향으로 가는가
            if (lid > 0) != fwd:           # 좌측(+) 차로는 s 반대로 가야 정상
                ok = True; break
        if not ok:
            wrong.append(i)
    print(f"[③ 도로 밖] {len(off)}점" + ("  ← ❌" if off else "  ✅"))
    for a, b in runs(off)[:4]:
        print(f"    s={cum[a]:.0f}~{cum[b]:.0f}m ({rt[a][0]:.0f},{rt[a][1]:.0f})")
    print(f"[④ 역주행] {len(wrong)}점" + ("  ← ❌" if wrong else "  ✅"))
    for a, b in runs(wrong)[:4]:
        print(f"    s={cum[a]:.0f}~{cum[b]:.0f}m ({rt[a][0]:.0f},{rt[a][1]:.0f})")
    bad += len(off) + len(wrong)

    # ⑥ ★어느 차로를 달리는가 — 지정차로(안쪽) 확인
    #   왜 필요한가(실측 2026-08-18): 생성 경로가 검증본 대비 **전 구간 한 차로 바깥**
    #   (3.20m)이었는데 ①~⑤ 를 전부 통과했다. 도로 안이고 방향도 맞으니까.
    #   그 탓에 v7 에서 연료통을 0.58m 로 스쳐 VTD 가 무너졌고, 도교법 지정차로도 어겼다.
    #   "이상 0곳" 이라고 적어두고 정작 가장 큰 걸 못 잡았다.
    seq, outer = [], []
    for i, cands in enumerate(all_c):
        if not cands:
            continue
        _o, rd, lid, _h = cands[0]
        rid = rd.get("id")
        px, py = rt[i][0], rt[i][1]
        if rd.get("junction", "-1") != "-1":      # 교차로 연결로는 차로 개념이 다르다
            continue
        s_at = None
        for x, y, h, r2, sv in mp.grid.get((int(px // CELL), int(py // CELL)), ()):
            if r2 is rd:
                s_at = sv; break
        same = sorted({l for l, typ, _lo, _hi, _m in mp.lanes(rd, s_at if s_at is not None else 1e-3)
                       if typ == "driving" and (l > 0) == (lid > 0)}, key=abs)
        rank = same.index(lid) if lid in same else 0
        if not seq or seq[-1][0] != rid or seq[-1][1] != lid:
            seq.append((rid, lid, rank, len(same), i))
        if rank > 0:
            outer.append(i)
    # ⚠️ 판정하지 않고 **보여만 준다.** 지정차로가 늘 최안쪽인 것도 아니고, ego 가
    #    실제로 스폰되는 차로가 이미 바깥일 수 있다(road 128 lane +2 = 바깥 1칸).
    #    자동 판정은 아래 ⑦(기준경로 대조)이 한다.
    print(f"[⑥ 차로 점유] 안쪽 아닌 점 {len(outer)}/{len(rt)} — 아래 순서를 눈으로 확인할 것")
    for rid, lid, rank, ntot, i in seq[:12]:
        mark = "  ← 바깥" + str(rank) + "칸" if rank else ""
        print(f"    s={cum[i]:5.0f}m  road {rid:>6} lane {lid:+d}  (같은방향 {ntot}차로 중 안쪽에서 {rank}){mark}")
    # ⑧ ★경로에 박힌 횡이동(차선변경) — **교차로 안이면 실격**
    #   왜 필요한가(실측 2026-08-19, road 128): 그래프는 "안쪽으로 한 칸"을 제대로
    #   계획했는데 sample_lane 이 목표를 **번호**로 찾는 바람에 차선변경이 통째로
    #   사라졌다. 바깥 차로에 남은 채 교차로까지 가서, 거기서 이음매 블렌딩이 3.2m 를
    #   끌어당겼다 — 화면엔 '사거리 한복판에서 이유 없는 차선변경'. ①~⑦ 이 전부
    #   통과했다(도로 안·방향 맞음·기준경로 이격도 중앙값 0.43m 라 안 걸림).
    #   ★차로 중심에서 벗어나 **차로와 차로 사이에 걸쳐 있는 점**을 세면 잡힌다.
    #     경로는 원래 차로 중심선을 따라 찍히므로, 걸친 구간 = 옮기는 중인 구간이다.
    #   ⚠️ '점을 담는 차로 아무거나'로 중심을 재면 안 된다. 교차로는 연결로가 서로
    #      겹쳐 어떤 자세로 있든 중심선 하나가 근처에 있고, 반대로 접근로 위의 정상
    #      차선변경이 겹친 연결로 때문에 '교차로 안'으로 찍힌다(둘 다 실측).
    #      경로 파일이 스스로 적어둔 `ego_lanes`(도로,차로,s,t)를 쓴다 — 그 도로에서만 본다.
    lanes_meta = json.load(open(sys.argv[1], encoding="utf-8")).get("ego_lanes")
    if not lanes_meta:
        print("[⑧ 경로에 박힌 차선변경] 생략 — ego_lanes 가 없는 경로(녹화본 등)")
    else:
        STRADDLE = 0.8          # 자기 차로 중심에서 이보다 벗어나면 '옮기는 중'[m]
        strad = []
        for i, m in enumerate(lanes_meta[:len(rt)]):
            rid, lid, sv, tv = m[:4]      # 5번째(차선변경 표시)는 여기서 안 쓴다
            rd = mp.roads.get(rid) if hasattr(mp.roads, "get") else None
            if rd is None:
                continue
            c = [(lo + hi) / 2.0 for l, typ, lo, hi, _m in mp.lanes(rd, sv)
                 if typ == "driving" and l == lid]
            if c and abs(tv - c[0]) > STRADDLE:
                strad.append(i)
        jr = {m[0] for m in lanes_meta
              if (mp.roads.get(m[0]) is not None
                  and mp.roads[m[0]].get("junction", "-1") != "-1")}
        lc_runs = [(a, b) for a, b in runs(strad, gap=6) if cum[min(b, len(cum) - 1)] - cum[a] > 8.0]
        bad_lc = [(a, b) for a, b in lc_runs
                  if any(lanes_meta[k][0] in jr for k in range(a, min(b + 1, len(lanes_meta))))]
        print(f"[⑧ 경로에 박힌 차선변경] {len(lc_runs)}곳"
              + (f"  ← ❌ 그중 {len(bad_lc)}곳이 교차로 안" if bad_lc else "  ✅ 교차로 안에는 없음"))
        for a, b in lc_runs[:6]:
            tag = "  ← ❌ 교차로 안" if (a, b) in bad_lc else ""
            print(f"    s={cum[a]:5.0f}~{cum[min(b, len(cum)-1)]:5.0f}m "
                  f"({rt[a][0]:.0f},{rt[a][1]:.0f}) road {lanes_meta[a][0]}{tag}")
        bad += len(bad_lc)

    # ⑨ ★노면 화살표(지정차로) 위반 — "좌회전 차선에서 직진" 류
    #   실측 2026-08-19: WP9 코스에 3곳 있었다. 원인은 두 가지였고 성격이 다르다.
    #    (a) 라우터가 회전차로를 **볼 수 없었다** — 좌/우회전 포켓은 교차로 직전에
    #        새로 생기는데 차선변경 이웃을 진입 laneSection 에서만 셌다(plan_route 수정).
    #    (b) 맵 자체가 모순 — 화살표는 '직진은 -2차로'인데 그 차로의 연결로는 우회전뿐.
    #        이건 우회로가 없어 못 고친다. **그래서 판정은 하지 않고 세어서 보여준다.**
    if lanes_meta:
        from plan_route import LaneGraph, travel_dir     # noqa: E402
        g = LaneGraph(mp, {}, {})
        KOR = {"left": "좌회전", "right": "우회전", "straight": "직진", "u": "유턴"}
        viol = []
        for i in range(1, len(lanes_meta)):
            rid, lid, _s, _t = lanes_meta[i - 1][:4]
            cid, clane, _s2, _t2 = lanes_meta[i][:4]
            if cid == rid or mp.roads.get(rid) is None or mp.roads.get(cid) is None:
                continue
            if mp.roads[rid].get("junction", "-1") != "-1":     # 연결로에서 나가는 건 대상 아님
                continue
            _jid = mp.roads[cid].get("junction", "-1")
            if _jid == "-1":                                   # 교차로 진입이 아니다
                continue
            if not is_real_junction(mp, _jid):
                continue
            allow = g.arrow_moves(rid, lid, travel_dir(lid))
            kind = g.turn_kind(cid, clane)
            if allow is not None and kind is not None and kind not in allow:
                viol.append((i, rid, lid, cid, allow, kind))
        # ★★위반마다 **우회로가 있는지 자동으로 판정한다.**
        #   실측 2026-08-20: 4곳 중 3곳은 맵 모순(지킬 차로가 아예 없다)이고 1곳은
        #   우리 라우터 버그였는데, 예전 출력은 둘을 구분하지 못해 "확인할 것"만 찍었다.
        #   대회 당일엔 이 구분이 곧 '손댈 수 있나 없나'다.
        #   판정: **같은 연결로로 들어가는 다른 진출 차로 중, 그 회전을 허용하는 화살표가
        #   그려진 차로가 있는가.** 있으면 우리가 잘못 간 것이다.
        def detour(rid, cid, kind):
            for c2, links in g.jconn.get(rid, ()):
                if c2 != cid:
                    continue
                for lex2 in links:
                    a2 = g.arrow_moves(rid, lex2, travel_dir(lex2))
                    k2 = g.turn_kind(c2, links[lex2])
                    if a2 is not None and k2 == kind and kind in a2:
                        return lex2
            return None

        marked = [(v, detour(v[1], v[3], v[5])) for v in viol]
        n_fix = sum(1 for _v, d in marked if d is not None)
        print(f"[⑨ 노면 화살표] 지시위반 {len(viol)}곳"
              + (f"  ← ⚠ 그중 **{n_fix}곳은 맞는 차로가 실재**한다(우리 잘못)"
                 if n_fix else ("  (전부 맵 모순 — 지킬 차로가 없다)" if viol else "  ✅")))
        for (i, rid, lid, cid, allow, kind), alt in marked[:6]:
            tail = (f"  ← **lane {alt:+d} 로 갔어야 한다**" if alt is not None
                    else "  (맵 모순: 그 회전을 허용하는 차로가 이 도로에 없다)")
            print(f"    s={cum[i]:5.0f}m  road {rid} lane {lid:+d} "
                  f"화살표 {{{','.join(KOR.get(a, a) for a in sorted(allow))}}} "
                  f"인데 연결로 {cid} 로 {KOR.get(kind, kind)}{tail}")

    # ⑦ ★기준 경로와 대조 (3번째 인자로 검증된 경로를 주면)
    #   오늘(2026-08-18) 버그를 실제로 잡아낸 검사가 이것이다. 생성 경로가 검증본 대비
    #   **전 구간 한 차로(3.20m)** 밖이었는데 ①~⑥ 으로는 안 걸렸다 — 도로 안이고
    #   방향도 맞으니까. 차로폭의 절반(≈1.6m)을 넘게 벌어지면 다른 차로를 달리는 것이다.
    if len(sys.argv) > 3:
        ref = json.load(open(sys.argv[3], encoding="utf-8"))["ego_route"]
        dev = sorted(min(math.dist((q[0], q[1]), (x, y)) for q in ref)
                     for x, y in [(r[0], r[1]) for r in rt[::5]])
        med, p90 = dev[len(dev) // 2], dev[int(len(dev) * 0.9)]
        off_lane = med > 1.6
        print(f"[⑦ 기준경로 대조] 이격 중앙값 {med:.2f}m  90%tile {p90:.2f}m"
              + ("  ← ❌ 한 차로 가까이 밀렸다" if off_lane else "  ✅"))
        if off_lane:
            bad += 1
    else:
        print("[⑦ 기준경로 대조] 생략 — 검증된 경로를 3번째 인자로 주면 대조한다")

    print(f"\n=== 이상 {bad}곳 " + ("— 이 경로는 쓰면 안 된다 ===" if bad else "— 사용 가능 ==="))


if __name__ == "__main__":
    main()
