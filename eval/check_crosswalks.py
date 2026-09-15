#!/usr/bin/env python3
"""어린이보호구역 **무신호 횡단보도**에서 정말 일시정지했는지, **어디에** 섰는지 채점.

  도로교통법 제27조⑦ (2022-01-11 신설)
  > 어린이 보호구역 내에 설치된 횡단보도 중 **신호기가 설치되지 아니한** 횡단보도 앞
  > (**정지선이 설치된 경우에는 그 정지선**)에서는 보행자의 횡단 여부와 관계없이 일시정지

왜 따로 만드나(2026-08-20): '섰나 안 섰나'는 눈으로 셌었다(코스 A 24/24). 그런데
사용자 지적 **"정지선은 무시하고 그냥 횡단보도 앞에서 멈추는데"** 처럼 **선 위치가 틀린 것**은
'섰다' 카운트로는 안 잡힌다. 조문이 정지선을 명시하므로 위치까지 채점한다.

판정(앞범퍼 기준 정지선까지의 거리):
    0 ~ 3.0m   ✅ 정지선 준수
    3.0m 초과  ⚠️ 너무 멀리 섬(= 횡단보도 본체 기준으로 선 것. 건너편 정지선을 잡았을 수도)
    음수       ❌ 정지선을 넘어서 섬
    안 섬      ❌ 미정지

사용: python3 eval/check_crosswalks.py <run.csv> <route.json> <route_lane.json>
"""
import csv
import json
import math
import sys

HERE = __file__.rsplit("/", 1)[0]
CW_DB = HERE + "/../routes/crosswalks.json"
FRONT = 3.81          # 뒷축(기록 좌표) -> 앞범퍼
ON_ROUTE = 6.0        # 경로에서 이 안에 있는 횡단보도만 '내가 지나는 것'
ZONE_LIM = 30 / 3.6 + 0.1
STOP_V = 0.3
GOOD = 3.0            # 정지선 앞 이 안에 서면 준수
HALF = 3.0            # 횡단보도 반폭(정지선이 없을 때 본체 기준)
SHARE_BACK = 45.0     # 자기 정지선이 없는 횡단보도: 이 앞에서 선 것을 인정


def path_back(route, cum, sx, sy, cx, cy):
    """정지 지점에서 횡단보도까지 **경로를 따라간 거리**. 뒤에 있지 않으면 None.

    ⚠️ 횡거리로 창을 치면 **곡선에서 샌다.** 실측 2026-08-24 `_TR` 판 road 2826:
       31.1m 뒤 정지선에서 제대로 섰는데, 그 지점에서 횡단보도를 보면 전방 25.9m ·
       **횡 6.3m** 라 횡 게이트(6.0m)에 0.3m 차이로 걸려 '미정지'로 찍혔다.
       횡 게이트는 **다른 도로의 횡단보도를 거르려는 것**이고 그건 duty 선정에서 이미
       했다. 공유 정지 인정에는 경로 거리를 쓴다.
    """
    i = min(range(len(route)), key=lambda k: (route[k][0] - sx) ** 2 + (route[k][1] - sy) ** 2)
    j = min(range(len(route)), key=lambda k: (route[k][0] - cx) ** 2 + (route[k][1] - cy) ** 2)
    return (cum[j] - cum[i]) if j > i else None


def ego_frame(x, y, h, px, py):
    dx, dy = px - x, py - y
    c, s = math.cos(-h), math.sin(-h)
    return dx * c - dy * s, dx * s + dy * c


def main():
    csv_path, route_path, lane_path = sys.argv[1], sys.argv[2], sys.argv[3]
    route = json.load(open(route_path, encoding="utf-8"))["ego_route"]
    lim = [p["lim"] for p in json.load(open(lane_path, encoding="utf-8"))["pts"]]
    cws = json.load(open(CW_DB, encoding="utf-8"))["crosswalks"]
    rows = [(float(r["t"]), float(r["x"]), float(r["y"]), float(r["heading"]), float(r["v"]))
            for r in csv.DictReader(open(csv_path, encoding="utf-8"))]
    if not rows:
        print("빈 CSV"); return

    # ① 이 경로에서 **서야 하는** 횡단보도 추리기
    duty = []
    for c in cws:
        if c["signal"]:
            continue                                   # 신호 있는 곳은 신호등 로직 담당
        best = min(range(len(route)),
                   key=lambda i: math.hypot(route[i][0] - c["x"], route[i][1] - c["y"]))
        d = math.hypot(route[best][0] - c["x"], route[best][1] - c["y"])
        if d > ON_ROUTE:
            continue
        if best < len(lim) and lim[best] > ZONE_LIM:
            continue                                   # 보호구역(30km/h) 아님
        duty.append(c)
    if not duty:
        print("이 경로엔 어린이보호구역 무신호 횡단보도가 없다"); return
    cum = [0.0]
    for k in range(1, len(route)):
        cum.append(cum[-1] + math.hypot(route[k][0] - route[k-1][0], route[k][1] - route[k-1][1]))
    # 완전정지한 지점들(연속 프레임은 하나로)
    stops, last_t = [], None
    for t, x, y, h, v in rows:
        if v < STOP_V and (last_t is None or t - last_t > 1.0):
            stops.append((t, x, y, h, v)); last_t = t
        elif v < STOP_V:
            last_t = t

    def shared_ok(c):
        """자기 정지선이 없는 횡단보도 — 앞선 완전정지로 충족되나. (거리, 시각) 또는 None.

        ⚠️ 이건 **판정을 느슨하게 하는 규칙**이라 근거를 반드시 출력한다. 어디서 몇 m 뒤에
           섰는지 안 보이면 '눈속임'과 구분이 안 된다.
        """
        best = None
        for t2, sx2, sy2, _h2, _v in stops:
            d = path_back(route, cum, sx2, sy2, c["x"], c["y"])
            if d is not None and 0.0 < d <= SHARE_BACK and (best is None or d < best[0]):
                best = (d, t2, sx2, sy2)
        return best

    # ② 각 횡단보도마다 '앞범퍼가 넘기 직전'의 최저속 프레임을 찾는다
    ok = far = over = miss = spawn = shared = 0
    bad, shown, credited = [], [], []
    t0, x0, y0, h0, _ = rows[0]
    for c in duty:
        # ★출발지점에 걸친 횡단보도는 판정에서 뺀다. 코스 A 실측: ego 스폰 (851.2,-17.2) 에서
        #   road 173 s=240 (856,-11) 까지 7.8m — **정지선 위에서 출발**하는 셈이라 설 자리가
        #   없다. 이걸 위반으로 세면 고칠 수 없는 감점이 영구히 남는다.
        f0, l0 = ego_frame(x0 + FRONT * math.cos(h0), y0 + FRONT * math.sin(h0), h0,
                           c["x"], c["y"])
        # ⚠️ **앞에 있을 때만** 뺀다. 원래 `f0 < 10.0` 이라 스폰 **뒤**에 있는 것까지
        #    전부 걸렀다(실측 2026-08-25 코스 B: road 1222 가 f0=-31.0m 인데 '출발지점에
        #    걸침'으로 빠졌다 — 그런데 그 자리를 t=331s 에 실제로 다시 지난다).
        #    판정을 느슨하게 하는 규칙이 조용히 진짜 의무를 지우고 있었다.
        if -2.0 <= f0 < 10.0 and abs(l0) < ON_ROUTE:
            spawn += 1
            bad.append((c, f0, f"출발지점에 걸침(스폰이 {f0:.1f}m 앞) — 판정 제외"))
            continue
        appr = []          # (앞범퍼 기준 전방거리, 프레임)
        for t, x, y, h, v in rows:
            bx, by = x + FRONT * math.cos(h), y + FRONT * math.sin(h)
            fwd, lat = ego_frame(bx, by, h, c["x"], c["y"])
            # ⚠️ 창을 0 부터 열면 **정지선 바로 위에 선 프레임**을 놓친다(앞범퍼가 이미
            #    횡단보도 중심을 살짝 지난 상태로 서는 경우). 뒤로 2m 열어둔다.
            if -2.0 < fwd < 45.0 and abs(lat) < ON_ROUTE:
                appr.append((fwd, t, x, y, h, v))
        if not appr:
            miss += 1; bad.append((c, None, "접근 프레임 없음")); continue
        def gap_of(a):
            """그 정지 프레임의 (정지선까지 여유, 기준). drive.py 와 같은 규칙으로 선을 고른다."""
            _f, _t, ax, ay, ah, _v = a
            abx, aby = ax + FRONT * math.cos(ah), ay + FRONT * math.sin(ah)
            acwf, _ = ego_frame(abx, aby, ah, c["x"], c["y"])
            cd = []
            for q in c.get("stops", ()):
                qf, ql = ego_frame(abx, aby, ah, q[0], q[1])
                if not (abs(ql) <= ON_ROUTE and qf <= acwf + 0.5):
                    continue
                if len(q) > 2 and math.cos(q[2] - ah) < 0.5:
                    continue      # 나를 세우는 선이 아니다(건너편/교차로)
                cd.append(qf)
            return (max(cd), "정지선") if cd else (acwf - HALF, "본체")

        stops_here = [a for a in appr if a[5] < STOP_V]
        if not stops_here:
            # ⚠️ **'미정지'로 단정하기 전에 공유 정지를 먼저 본다.** 실측 2026-08-24
            #    road 2826: 31.1m 뒤 정지선에서 제대로 섰는데 곡선이라 그 프레임이
            #    접근창(횡 6m)에 안 들어와 `appr` 에 없었다. 그래서 '미정지'로 찍혔다.
            sh = shared_ok(c)
            if sh:
                shared += 1
                shown.append((c, sh))
                continue
            miss += 1
            bad.append((c, None, f"미정지(최저 {min(a[5] for a in appr) * 3.6:.1f}km/h)"))
            continue

        # ★★**정지가 여러 번이면 '가장 깊이 들어간 것'이 아니라 '규정을 지킨 것'을 본다.**
        #   실측 2026-08-25 코스 B road 2804: t=199.8 에 정지선 1.0m 앞에서 0.3km/h 로
        #   제대로 섰고(주행 규칙도 그래서 통과 처리했다), 그 뒤 **보행자가 나타나**
        #   횡단보도를 2m 지나 다시 섰다. 채점기가 뒤엣것을 집어 '정지선 6.3m 넘어서 섬'
        #   으로 찍었다 — **의무는 이미 이행했고 두 번째 정지는 보행자 보호(제27조①)다.**
        #   ⚠️ 느슨해지는 방향이라 **어느 정지를 인정했는지 반드시 출력**한다.
        good = None
        for a in sorted(stops_here, key=lambda z: -z[0]):    # 먼 쪽(=먼저 선 쪽)부터
            g, rf = gap_of(a)
            if 0.0 <= g <= GOOD:
                good = (a, g, rf)
                break
        deepest = min(stops_here, key=lambda a: a[0])
        if good is not None:
            a, gap, ref = good
            ok += 1
            dg, _ = gap_of(deepest)
            # ⚠️ **판정이 실제로 바뀐 것만** 남긴다. 정지선 앞에서 서고 1m 쯤 굴러간
            #    (둘 다 합격인) 흔한 경우까지 찍으면 목록이 소음이 되고, 정작
            #    '느슨하게 봐준 자리'가 안 보인다.
            if not (0.0 <= dg <= GOOD):
                credited.append((c, gap, a[1], dg))
            continue

        gap, ref = gap_of(deepest)
        if ref == "본체":
            # ★진행방향 정지선이 **없는** 횡단보도. 앞선 정지선/횡단보도에서 한 번 서는
            #   것으로 충족된다 [제27조⑦ "정지선이 설치된 경우에는 그 정지선"].
            #   교차로 건너편이면 거기서 서는 것 자체가 **교차로 내 정차**(제32조)다.
            sh = shared_ok(c) if gap > GOOD else None
            if sh:
                shared += 1
                shown.append((c, sh))
                continue
        if gap < 0:
            over += 1; bad.append((c, gap, f"{ref} 넘어서 섬"))
        elif gap <= GOOD:
            ok += 1
        else:
            far += 1; bad.append((c, gap, f"{ref}에서 {gap:.1f}m 앞 — 너무 멂"))

    n = len(duty) - spawn
    print(f"어린이보호구역 무신호 횡단보도 **{len(duty)}곳**"
          + (f" (출발지점에 걸친 {spawn}곳 제외 -> {n}곳 판정)" if spawn else ""))
    print(f"  ✅ 정지선 준수 {ok}   ✅ 앞선 정지로 충족 {shared}   "
          f"⚠️ 너무 멀리 {far}   ❌ 넘어서 섬 {over}   ❌ 미정지 {miss}")
    for c, (d, t2, sx2, sy2) in shown:
        print(f"    · road {c['road']} s={c['s']:.0f} ({c['x']:.0f},{c['y']:.0f}) — 자기 정지선 없음. "
              f"**{d:.1f}m 뒤** ({sx2:.0f},{sy2:.0f}) 에서 t={t2:.0f}s 에 정지함")
    for c, g, t_ok, g_deep in credited:
        print(f"    · road {c['road']} s={c['s']:.0f} ({c['x']:.0f},{c['y']:.0f}) — "
              f"t={t_ok:.0f}s 에 **정지선 {g:.1f}m 앞에서 규정대로 섰다**(이걸 인정). "
              f"그 뒤 {abs(g_deep):.1f}m 더 들어가 **다시 섰지만** 의무는 이미 이행됐다")
    for c, gap, why in bad[:15]:
        g = "" if gap is None else f"  (앞범퍼 {gap:+.1f}m)"
        print(f"    road {c['road']} s={c['s']:.0f} ({c['x']:.0f},{c['y']:.0f}) — {why}{g}")
    print(f"\n=== 횡단보도 {ok + shared}/{n} 충족"
      + (f" (정지선 {ok} + 공유 {shared})" if shared else "")
      + " ===" + ("  ✅" if ok + shared == n else "  ← 볼 것"))


if __name__ == "__main__":
    main()
