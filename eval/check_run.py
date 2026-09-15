#!/usr/bin/env python3
"""주행 CSV 채점: 신호위반 / 정지 여부 / 속도위반 / 경로이탈(리스폰) 점검.

신호위반 판정: 어떤 신호(tl_id)를 '통과하는 순간'(tl_id가 바뀌기 직전 프레임)의 상태가
RED면 위반. YELLOW는 통과 자체는 위반 아님(딜레마존)으로 보되 별도 표기.
사용: python3 check_run.py <run.csv> [제한속도 m/s]
"""
import csv as _csv
import json
import math
import sys

ST = {0: "UNSET", 1: "RED", 2: "YELLOW", 3: "GREEN", 4: "LEFT", 5: "GREEN_LEFT", 6: "FLASH"}


GOAL_R = 15.0        # 주최측 종료 판정이 "포인트 반경 10~20m"(강의 [103:56])
COVER_R = 6.0        # 경로점을 '지났다'고 볼 거리[m]. 차로 하나(3.2m)보다 넉넉히.
COVER_MIN = 0.90     # 이보다 적게 돌았으면 완주로 안 친다
FRONT_OVERHANG = 3.81  # 뒷축(=기록 좌표) -> 앞범퍼. 이만큼 넘어야 '정지선을 넘었다'
# 맵 전체 신호 정지선 DB. 있으면 '정말 정지선을 넘었는지'까지 보고 위반을 센다.
TL_MAP = __file__.rsplit("/", 1)[0] + "/../routes/tl_map_livinglab.json"


POPIN_WARN = 30.0    # 이보다 가까이서 **처음** 나타난 물체가 있으면 경고

# ★왜 이걸 보는가(2026-08-20, 사용자 질문 "대회에서 GT 에 숨기고 튀어나오는 건 없겠지?"):
#   객체 리스트는 "80m 이내, 가까운 순 30개"라 원리상 30m 앞에서 없던 게 생길 수 없다.
#   실측으로도 전 23판에서 **가장 늦은 첫 등장이 41.5m**(그것도 우리가 스폰시킨 EV_CUTIN)
#   이었고 VTD 가 만든 것은 최소 67.1m 였다. 그런데 이건 **사후에 계산해서 안 것**이라,
#   대회 당일 한 판에서 깨져도 모르고 지나간다. 매 판 자동으로 찍게 둔다.
#   ⚠️ 물체 식별자가 CSV 에 없어서 **크기지문 + 직전 프레임과의 최근접 매칭**으로 잇는다.
#      프레임이 벌어진 구간은 상대이동이 커져 가짜 등장이 되므로 건너뛴다.
def parse_objs(cell):
    out = []
    for o in (cell or "").split("|"):
        f = o.split(":")
        if len(f) >= 5:
            try:
                out.append((float(f[0]), float(f[1]), (f[2], f[3], f[4])))
            except ValueError:
                pass
    return out


def popins(rows):
    """(첫 등장거리, t, 그 프레임 객체수) 목록 — 직전 프레임에 짝이 없던 물체.

    ⚠️ **객체수를 같이 낸다.** 주변 교통을 넣으면 같은 차종이 수십 대라 크기지문이
       겹쳐 매칭이 실패하고 **가짜 등장**이 나온다. 또 30칸이 가득 차면 먼 것부터
       잘려 목록에서 들락거린다. 그 둘을 구분하려면 그 순간 몇 개가 잡혔는지가 필요하다.
       (실측 2026-08-24 `_TR` 판: 5건이 30m 안에서 '처음' 나타났다고 찍혔다.)
    """
    out = []
    for a, b in zip(rows, rows[1:]):
        if not (0 < b["t"] - a["t"] <= 0.2):
            continue                      # 프레임 끊긴 구간은 못 믿는다
        free = list(a["objs"])
        for fx, fy, sig in b["objs"]:
            m = [(math.hypot(fx - px, fy - py), i)
                 for i, (px, py, ps) in enumerate(free) if ps == sig]
            m = [c for c in m if c[0] < 6.0]
            if m:
                free.pop(min(m)[1])
            else:
                out.append((math.hypot(fx, fy), b["t"], len(b["objs"])))
    return out


def main():
    path = sys.argv[1]
    vlim = float(sys.argv[2]) if len(sys.argv) > 2 else 8.33
    # ★경로 파일을 주면 **완주 여부**까지 판정한다. 이게 없으면 요약줄이
    #   "신호위반 0 · 리스폰 0 · 접촉 0" 이라 미완주도 통과처럼 읽힌다
    #   (2026-08-16 실측: 그렇게 세 판을 통과로 오독했다).
    route = None
    if len(sys.argv) > 3:
        route = json.load(open(sys.argv[3], encoding="utf-8"))
    # ★차로계획을 주면 **구간별 제한속도**로 채점한다. 단일값으로 재면 50 구간을
    #   30 으로 보고 전 구간 위반, 30 구간을 50 으로 보고 위반을 놓친다.
    lane_plan = None
    if len(sys.argv) > 4:
        lane_plan = json.load(open(sys.argv[4], encoding="utf-8"))["pts"]
    try:
        tl_stops = {int(k): v for k, v in json.load(open(TL_MAP, encoding="utf-8")).items()}
    except Exception:
        tl_stops = {}
    rows = []
    with open(path, encoding="utf-8") as f:
        for r in _csv.DictReader(f):
            rows.append(dict(t=float(r["t"]), x=float(r["x"]), y=float(r["y"]),
                             h=float(r["heading"]),
                             v=float(r["v"]), tid=int(r["tl_id"]), st=int(r["tl_state"]),
                             reason=r.get("reason", ""),
                             objs=parse_objs(r.get("objs", "")),
                             clr=float(r["clr"]) if r.get("clr") not in (None, "") else None))
    if not rows:
        print("빈 CSV"); return
    dist = sum(math.hypot(rows[i+1]["x"]-rows[i]["x"], rows[i+1]["y"]-rows[i]["y"])
               for i in range(len(rows)-1))
    print(f"프레임 {len(rows)}  시간 {rows[-1]['t']:.0f}s  주행 {dist:.0f}m  "
          f"평균 {dist/max(1e-6,rows[-1]['t'])*3.6:.1f}km/h  최고 {max(r['v'] for r in rows)*3.6:.1f}km/h")

    done = None
    if route:
        gx, gy = route["ego_goal"][0], route["ego_goal"][1]
        rest = math.hypot(rows[-1]["x"] - gx, rows[-1]["y"] - gy)
        done = rest <= GOAL_R
        print(f"[완주] {'✅ 목표 도달' if done else '❌ 미완주'} — 마지막 위치가 목표에서 "
              f"{rest:.0f}m (판정 반경 {GOAL_R:.0f}m)")
        # ★목표에 닿았다고 코스를 다 돈 게 아니다. 실측 2026-08-19 새 코스 A:
        #   출발점 근처로 되돌아오는 순환 코스에서 제어기가 첫 프레임에 뒤쪽 구간을
        #   집어 **앞 2.4km 를 통째로 건너뛰었는데**, 목표점에는 도달하니 '완주 ✅'
        #   가 찍혔다(커버리지 56%). 그런 판은 경유지 미방문이라 대회에선 큰 감점이다.
        rt = route["ego_route"]
        step = max(1, len(rt) // 400)                 # 400점만 표본으로 봐도 충분
        pos = [(r["x"], r["y"]) for r in rows[::max(1, len(rows) // 1500)]]
        seen = sum(1 for i in range(0, len(rt), step)
                   if min((rt[i][0] - x) ** 2 + (rt[i][1] - y) ** 2 for x, y in pos)
                   <= COVER_R ** 2)
        ntot = len(range(0, len(rt), step))
        frac = seen / max(1, ntot)
        print(f"[경로 커버리지] {frac*100:.0f}% ({seen}/{ntot}점, {COVER_R:.0f}m 이내)"
              + ("  ✅" if frac >= COVER_MIN else "  ← ❌ 코스를 다 돌지 않았다"))
        if frac < COVER_MIN:
            done = False

    # 1) 신호 통과 판정
    #  ⚠️ 'tl_id 가 바뀌기 직전 프레임'을 통과 순간으로 보면 안 된다. VTD 는 우리가
    #     정지선에 닿기 **전에도** 그 신호 보고를 끊는다(시야에서 벗어나거나 다른
    #     controller 로 넘어갈 때). 실측 2026-08-19 WP9: tl312 를 적색으로 마지막
    #     보고한 지점이 정지선 **18.8m 앞**이었고, 우리는 14.4m 앞에서 **완전히 정지**한
    #     뒤 진행했는데 '적색 통과'로 찍혔다. 유일하게 남아 있던 '위반 1건'이 오탐이었다.
    #  -> 정지선을 아는 신호는 **실제로 넘었을 때만** 위반으로 센다.
    print("\n[신호 통과]")
    viol = 0
    for i in range(1, len(rows)):
        a, b = rows[i-1], rows[i]
        if a["tid"] == 0 or b["tid"] == a["tid"]:
            continue
        sl = tl_stops.get(a["tid"]) if tl_stops else None
        note = ""
        st_at = a["st"]            # 통과 순간의 상태(아래에서 더 정확한 값으로 바뀔 수 있다)
        if sl is not None:
            # ⚠️ 정지선까지의 **거리**로 재면 안 된다. DB 의 정지선 좌표는 그 신호의
            #    대표 차로 중심이라, 옆 차로로 실제 통과해도 4~8m 가 남는다(실측 WP9
            #    tl 90/58/53 — 녹색으로 지나갔는데 '통과 아님'이 됐다).
            #    **진행방향 성분**으로 본다: 앞범퍼가 선을 넘었으면 통과다.
            # ★'정지선보다 앞에 있다'가 아니라 **뒤에 있다가 앞으로 넘어간 순간**을 찾는다.
            #   실측 2026-08-20 새 코스 B: 오탐이 두 건 나왔다.
            #     ① ego 가 tl147 정지선을 **이미 94m 지난 자리에 스폰**된다. 넘은 적이
            #        없는데 '앞에 있으니 통과'로 세고, 그때 상태가 적색이라 위반.
            #     ② 같은 신호를 코스 후반에 제대로 만나 **적색에 14초 정지 후
            #        GREEN_LEFT 에 출발**했는데, 스캔이 0프레임부터라 ①의 순간을 다시
            #        집어 또 위반. 주행은 완벽했다.
            #   그리고 **마지막 넘김**을 쓴다 — 코스가 같은 신호를 두 번 지날 수 있다.
            crossed = False
            cross_i = None
            prev_u = None
            for j, r in enumerate(rows[:i + 1]):
                dx, dy = r["x"] - sl[0], r["y"] - sl[1]
                u = dx * math.cos(r["h"]) + dy * math.sin(r["h"])       # 진행방향 앞이 +
                lat = abs(-dx * math.sin(r["h"]) + dy * math.cos(r["h"]))
                # ⚠️ 횡 게이트는 '다른 도로의 신호'를 걸러내려는 것뿐이다. 넉넉히 준다 —
                #    좁게 잡으면 넓은 도로 바깥 차로로 **정말 적색 통과한 걸 놓친다**.
                #    놓치는 쪽이 오탐보다 위험하다.
                if lat >= 15.0:
                    prev_u = None            # 다른 도로에 있었다 -> 연속성 끊김
                    continue
                if prev_u is not None and prev_u + FRONT_OVERHANG <= 0.0 < u + FRONT_OVERHANG:
                    crossed = True
                    cross_i = j
                prev_u = u
            # ★위반 판정은 **넘는 순간의 상태**로 한다. 아래 `a["st"]` 는 그 신호를
            #   마지막으로 보고한 프레임의 상태인데, 녹색·황색에 적법하게 진입해
            #   교차로를 빠져나가는 사이 적색이 되면 그게 위반으로 찍힌다.
            #   (같은 오탐을 evaluate.py 에서도 잡았다 — 2026-08-20)
            #   ⚠️ 그 프레임이 이 신호를 보고 중일 때만 덮어쓴다. 시작부터 선 너머에
            #      있었던 경우까지 그 프레임 상태를 쓰면 엉뚱한 값이 된다.
            if cross_i is not None and rows[cross_i]["tid"] == a["tid"]:
                st_at = rows[cross_i]["st"]
            if not crossed:
                near = min(math.hypot(r["x"] - sl[0], r["y"] - sl[1]) for r in rows[:i + 1])
                note = f"  (정지선 {near:.0f}m 앞까지만 감 — 통과 아님)"
        # ★적신호 우회전은 **위반이 아니다** [시행규칙 별표2 '적색의 등화' 단서]:
        #   정지선·횡단보도·교차로 직전에 정지한 후, 신호에 따라 진행하는 다른 차마의
        #   교통을 방해하지 않으면 우회전할 수 있다.
        #   ⚠️ 이건 **판정을 느슨하게 하는 규칙**이라 조건을 실제로 검사하고 근거를 찍는다.
        #      그냥 '적색 통과를 봐준다'로 만들면 진짜 위반이 숨는다.
        #      ① 넘기 전에 **완전정지**했나(정지선 25m 안에서 v<0.3) ② 넘은 뒤 25m 안에
        #      **우회전**했나(방위가 45° 이상 우로). 둘 다여야 단서에 해당한다.
        rtor = None
        if st_at == 1 and not note and cross_i is not None:
            stopped_t = None
            for r in rows[:cross_i + 1]:
                if r["v"] < 0.3 and math.hypot(r["x"] - sl[0], r["y"] - sl[1]) < 25.0:
                    stopped_t = r["t"]
            dh = 0.0
            if stopped_t is not None:
                x0, y0, h0 = rows[cross_i]["x"], rows[cross_i]["y"], rows[cross_i]["h"]
                for r in rows[cross_i:]:
                    if math.hypot(r["x"] - x0, r["y"] - y0) > 25.0:
                        break
                    d = (r["h"] - h0 + math.pi) % (2 * math.pi) - math.pi
                    if d < dh:
                        dh = d
            if stopped_t is not None and dh <= -math.radians(45):
                rtor = (stopped_t, math.degrees(dh))
        tag = ST.get(st_at, st_at)
        mark = ""
        if rtor is not None:
            mark = (f"  ← ✅적신호 우회전(별표2 단서): t={rtor[0]:.0f}s 에 정지 후 "
                    f"{-rtor[1]:.0f}° 우회전")
        elif st_at == 1 and not note:
            mark = "  ← ❌신호위반"; viol += 1
        elif st_at == 2 and not note:
            mark = "  ← ⚠황색통과"
        print(f"  tl{a['tid']:>4}  통과시 {tag:10s} v={a['v']*3.6:5.1f}km/h  "
              f"({a['x']:7.1f},{a['y']:8.1f}){mark}{note}")

    # 2) 정지 이력
    stops, ins = [], False
    for r in rows:
        if r["v"] < 0.3 and not ins:
            ins = True; stops.append([r["t"], r["t"], r["x"], r["y"], r["tid"], r["st"]])
        elif r["v"] >= 0.5 and ins:
            ins = False; stops[-1][1] = r["t"]
    print(f"\n[정지] {len(stops)}회")
    for s0, s1, x, y, tid, st in stops:
        print(f"  {s0:6.1f}s ~ {s1:6.1f}s ({s1-s0:4.1f}s) at ({x:7.1f},{y:8.1f}) tl{tid}/{ST.get(st,st)}")

    # 3) 속도위반
    if lane_plan and route:
        # 프레임마다 가장 가까운 경로점의 제한을 쓴다(경로점 간격 1.4m)
        rt = route["ego_route"]
        lims = []
        k = 0
        for r in rows:
            # ⚠️ 창을 뒤로 열어두면(예전엔 k-40) **자기와 교차하는 코스**에서 앞선 구간의
            #    점에 붙잡혀 영영 못 빠져나온다. 실측 2026-08-19 코스 A(5278m, 같은
            #    동네를 두 번 지난다): k 가 1500 근처에 갇혀 뒤쪽 50km/h 구간을 전부
            #    30 으로 채점했고 **969프레임 과속**이 찍혔다 — 실제로는 50 구간을
            #    50 으로 달린 합법 주행이었다.
            #    -> 앞으로만 본다. 그래도 창을 벗어나면(5m 초과) 전역에서 다시 찾는다.
            best, bi = 1e18, k
            for j in range(k, min(len(rt), k + 400)):
                d = (rt[j][0] - r["x"]) ** 2 + (rt[j][1] - r["y"]) ** 2
                if d < best:
                    best, bi = d, j
            if best > 25.0:                       # 5m^2 — 창 밖으로 나갔다
                for j in range(len(rt)):
                    d = (rt[j][0] - r["x"]) ** 2 + (rt[j][1] - r["y"]) ** 2
                    if d < best:
                        best, bi = d, j
            k = bi
            p = lane_plan[bi] if bi < len(lane_plan) else None
            lims.append((p or {}).get("lim") or vlim)
        over = [r for r, L in zip(rows, lims) if r["v"] > L * 1.1]
        kinds = sorted({round(L * 3.6) for L in lims})
        print(f"\n[속도] 구간별 제한 {kinds} km/h  초과(10%↑) 프레임 {len(over)}/{len(rows)}")
        if over:
            w = max(over, key=lambda r: r["v"])
            wl = lims[rows.index(w)]
            print(f"    최악 {w['v']*3.6:.1f}km/h (제한 {wl*3.6:.0f}) at {w['t']:.1f}s "
                  f"({w['x']:.0f},{w['y']:.0f})")
    else:
        over = [r for r in rows if r["v"] > vlim * 1.1]
        print(f"\n[속도] 제한 {vlim*3.6:.0f}km/h  초과(10%↑) 프레임 {len(over)}/{len(rows)}")

    # 4) 리스폰 — 감점 항목이다. 좌표 점프뿐 아니라 **헤딩 스냅**도 봐야 한다.
    #    실측 2026-08-15: 추월 중 리스폰이 1.59m 이동 + 20° 스냅으로 와서 거리 기준을 빠져나갔다.
    t0 = rows[0]["t"]
    jumps = []
    for i in range(1, len(rows)):
        a, b = rows[i-1], rows[i]
        dt = max(b["t"] - a["t"], 0.02)
        d = math.hypot(b['x']-a['x'], b['y']-a['y'])
        dh = abs((b["h"] - a["h"] + math.pi) % (2*math.pi) - math.pi)
        # ⚠️ **거리 기준을 고정 3m 로 두면 안 된다.** 프레임 간격이 벌어지면 정상 주행도
        #    한 프레임에 3m 를 간다(2026-08-16 실측 v7: VTD 출력이 3.5Hz 로 떨어져
        #    0.28s x 10.7m/s = 3.00m -> **리스폰 111건**으로 세었다. 진짜 리스폰은
        #    "14.1m/32°" 한 건뿐이었다). 그 프레임에 물리적으로 가능한 거리를 기준으로 쓴다.
        reach = max(3.0, (a["v"] + 3.0) * dt * 1.6)     # 여유 있게: 가속·측정오차 포함
        if d > reach or dh > 3.0 * dt:
            jumps.append((round(b["t"]-t0, 1), f"{d:.1f}m/{math.degrees(dh):.0f}°"))
    print(f"[리스폰] {len(jumps)}건 {jumps[:5]}" + ("  ← ❌건당 감점" if jumps else "  ✅"))

    # 5) 차체 실여유 — 예전 채점기는 이걸 안 봐서 '연료통을 밟고도 충돌 아님'으로 통과시켰다
    have = [r for r in rows if r["clr"] is not None and r["clr"] < 90]
    hits = 0
    if have:
        m = min(have, key=lambda r: r["clr"])
        hits = sum(1 for r in have if r["clr"] < 0)
        near = sum(1 for r in have if 0 <= r["clr"] < 0.3)
        tag = "❌접촉" if hits else ("⚠아슬아슬" if near else "✅")
        print(f"[차체 실여유] 최소 {m['clr']:+.2f}m at {m['t']:.1f}s "
              f"({m['x']:.1f},{m['y']:.1f}) [{m['reason']}]   접촉 {hits}프레임 · 0.3m미만 {near}프레임  {tag}")
    elif rows[0]["clr"] is None:
        print("[차체 실여유] clr 컬럼 없음(구버전 로그)")
    else:
        print("[차체 실여유] 주행 내내 객체가 하나도 안 잡힘 — 시나리오에 상대차가 없거나 스폰 실패")

    # 6) GT 갑툭튀 감시 — 80m 이내 전부 온다는 전제가 그 판에서도 지켜졌는지
    pi = popins(rows)
    if any(r["objs"] for r in rows):
        bad = sorted(p for p in pi if p[0] < POPIN_WARN)
        if bad:
            full = sum(1 for p in bad if p[2] >= 28)
            print(f"[GT 갑툭튀] ⚠️ **{len(bad)}건이 {POPIN_WARN:.0f}m 안에서 처음 나타났다** "
                  f"— 최악 {bad[0][0]:.1f}m at {bad[0][1]:.1f}s (그때 객체 {bad[0][2]}개)")
            print(f"            그중 {full}건은 목록이 거의 찬 상태(28개 이상) — "
                  f"30칸 잘림이나 지문 겹침일 수 있다. 나머지 {len(bad)-full}건이 진짜 후보다")
        else:
            near = min(pi)[0] if pi else float("inf")
            print(f"[GT 갑툭튀] 가장 늦은 첫 등장 "
                  f"{'없음(내내 같은 물체)' if pi == [] else f'{near:.1f}m'}  ✅")

    # ★프레임율은 **중앙값이 아니라 최악 구간**으로 본다.
    #   실측 2026-08-18 v7: t=100s 에 25Hz -> 3.4Hz 로 무너져 끝까지 회복 못 했는데,
    #   앞 100초가 25Hz 라 **중앙값은 25.0Hz** 였다. 러너가 "정상"이라 찍는 바람에
    #   붕괴한 판의 점수를 그대로 믿을 뻔했다(그 판은 우리가 vp=0 을 줘도 VTD 가
    #   안 세워서 목표를 89m 지나쳤다 — 제어권 상실은 붕괴의 알려진 증상이다).
    ts = [r["t"] for r in rows]
    if len(ts) > 10:
        worst, worst_t, BUCKET = None, 0.0, 20.0
        b0 = ts[0]
        while b0 < ts[-1]:
            seg = [t for t in ts if b0 <= t < b0 + BUCKET]
            if len(seg) > 3:
                d = sorted(seg[i] - seg[i - 1] for i in range(1, len(seg)))
                hz = 1.0 / d[len(d) // 2] if d[len(d) // 2] > 0 else 0.0
                if worst is None or hz < worst:
                    worst, worst_t = hz, b0
            b0 += BUCKET
        if worst is not None and worst < 12.0:
            print(f"[프레임율] ⚠️ **최악 구간 {worst:.1f} Hz** (t={worst_t:.0f}s 부터 20초) "
                  f"— 무너진 구간이 있다. 이 판의 수치는 못 믿는다")

    tag = "" if done is None else ("완주 ✅ · " if done else "**미완주** ❌ · ")
    print(f"\n=== {tag}신호위반 {viol}건 · 리스폰 {len(jumps)}건 · 접촉 {hits}프레임 ===")


if __name__ == "__main__":
    main()
