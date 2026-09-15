#!/usr/bin/env python3
"""주행 장면 GIF — 왼쪽: 주행 CSV 로 그린 '차가 아는 세상', 오른쪽: VTD 녹화.

    python3 vtd/make_gif.py --video H.mp4 --csv run_H.csv --course HL_FMA_NEW_H \\
        --t0 36.0 --dur 13 --fps 6 --offset 7.53 --vcrop 1040:585:120:113 --out docs/img/red_light_stop.gif

녹화는 `vtd/record_window.sh`(VTD 중계 창 mainRS_Broadcast). 찍는 순서·offset 맞추는 법은 docs/img/README.md.

왼쪽 그림에 들어가는 것 — 전부 제어기가 그 프레임에 실제로 가진 정보다.
  · 차로 경계선 — 지도(xodr)의 주행차로 경계. 패킷에는 차선이 없어서 제어기도 지도에서 읽는다
  · 계획 경로(파랑)와 지금 따라가는 횡오프셋 경로
  · 물체 — CSV `objs` 칸의 자차 기준 좌표·크기·상대방위. 크기로 분류해 차(주황)·사람(청록)·사물(회색)
  · 자차, 속도, **속도를 잡은 규칙(`cap_by`)**, 지시등, 신호 상태

영상과 CSV 는 시작 시각이 다르다(녹화를 먼저 켠다). `--offset` = 영상 시각 − CSV 시각.
모르면 `--find-offset` 으로 **출발 순간**(정지 화면이 움직이기 시작하는 때)을 찾는다 —
출발 전 화면이 가만히 있어야 맞는다. 긴 녹화는 뒤로 갈수록 몇 초씩 밀릴 수 있어,
출발에서 먼 장면은 `--offset` 을 그 장면 가까이의 사건으로 다시 맞추는 게 안전하다.
"""
import argparse
import csv
import json
import math
import os
import pickle
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

EGO_FRONT, EGO_REAR, EGO_HALF_W = 3.808, 1.04, 0.943

# 다크 배경 — dataviz 검증 통과(3색 all-pairs, dark): 경로 #3987e5 · 차 #d95926 · 사람 #199e70
C = dict(bg="#1a1a19", lane="#4a4a46", edge="#8a8984", center="#a8872f", stop="#d9d8d3", route="#3987e5",
         veh="#d95926", vru="#199e70", obs="#c3c2b7", ego="#ffffff", text="#ffffff", text2="#c3c2b7",
         red="#e34948", yellow="#eda100", green="#1baf7a", dim="#3a3a37")
TL_COLOR = {1: "red", 2: "yellow", 3: "green", 4: "green", 5: "green", 6: "yellow"}
STOPLINES = []
OFFVIEW = 45.0                 # 시야 밖 앞쪽 물체를 가장자리에 표시할 거리[m]
OFFVIEW_LAT = 5.0              # 그중 좌우로 이만큼 안쪽만[m]


# ──────────────────────────── 지도: 차로 경계선 ────────────────────────────
def lane_segments(xodr, cache):
    """주행차로 경계 선분 [(x1,y1,x2,y2,kind)] 을 격자(20m)로 묶어 돌려준다."""
    if cache and os.path.exists(cache):
        return pickle.load(open(cache, "rb"))
    from build_lane_plan import Map
    mp = Map(xodr)
    per_road = {}
    for pts in mp.grid.values():
        for x, y, h, rd, s in pts:
            per_road.setdefault(rd.get("id"), []).append((s, x, y, h, rd))
    grid = {}
    for rid, samples in per_road.items():
        samples.sort(key=lambda q: q[0])
        prev = {}
        for s, x, y, h, rd in samples:
            cur = {}
            for lid, typ, lo, hi, mark in mp.lanes(rd, s):
                if typ != "driving":
                    continue
                # OpenDRIVE: 차로의 roadMark 는 **바깥쪽**(기준선에서 먼 쪽) 경계에 칠해진다.
                outer, inner = (hi, lo) if lid > 0 else (lo, hi)
                kind = "edge" if (mark or "").startswith("solid") or mark == "curb" else "lane"
                marks = [("out", outer, kind)]
                if abs(lid) == 1:
                    marks.append(("in", inner, "center"))       # 가장 안쪽 차로의 안쪽 = 중앙선
                for side, tt, kd in marks:
                    px, py = x - math.sin(h) * tt, y + math.cos(h) * tt
                    cur[(lid, side)] = (px, py, kd)
            for k, (px, py, kind) in cur.items():
                if k in prev:
                    qx, qy, _ = prev[k]
                    if math.hypot(px - qx, py - qy) < 4.0:
                        cell = (int(px // 20), int(py // 20))
                        grid.setdefault(cell, []).append((qx, qy, px, py, kind))
            prev = cur
    if cache:
        pickle.dump(grid, open(cache, "wb"))
    return grid


def near(grid, x, y, r):
    out = []
    for a in range(int((x - r) // 20), int((x + r) // 20) + 1):
        for b in range(int((y - r) // 20), int((y + r) // 20) + 1):
            out.extend(grid.get((a, b), ()))
    return out


# ──────────────────────────── CSV ────────────────────────────
def load_rows(path):
    rows = []
    for r in csv.DictReader(open(path, encoding="utf-8")):
        objs = []
        for cell in (r.get("objs") or "").split("|"):
            p = cell.split(":")
            if len(p) < 6:
                continue
            try:
                fx, fy, ln, wd, ht, sp = map(float, p[:6])
                dh = float(p[9]) if len(p) > 9 and p[9] not in ("", "nan") else 0.0
            except ValueError:
                continue
            objs.append((fx, fy, ln, wd, ht, sp, dh))
        rows.append(dict(t=float(r["t"]), x=float(r["x"]), y=float(r["y"]), h=float(r["heading"]),
                         v=float(r["v"]), sig=int(float(r["sig"] or 0)), off=float(r["off"] or 0),
                         cap=r.get("cap_by", ""), reason=r.get("reason", ""),
                         tl=int(float(r.get("tl_state") or 0)), objs=objs))
    return rows


def row_at(rows, t):
    lo, hi = 0, len(rows) - 1
    while lo < hi:
        m = (lo + hi) // 2
        if rows[m]["t"] < t:
            lo = m + 1
        else:
            hi = m
    return rows[lo]


# ──────────────────────────── 왼쪽 그림 ────────────────────────────
def render_left(ax, row, grid, route, W, AHEAD, BEHIND):
    import matplotlib.patches as mpatches
    from matplotlib.collections import LineCollection
    from vtd_io import classify_object, ObjectClass, is_vru

    ex, ey, eh = row["x"], row["y"], row["h"]
    rot = math.pi / 2 - eh                      # 진행방향을 화면 위(+y)로
    cr, sr = math.cos(rot), math.sin(rot)

    def tf(px, py):
        dx, dy = px - ex, py - ey
        return dx * cr - dy * sr, dx * sr + dy * cr

    ax.clear()
    ax.set_facecolor(C["bg"])
    ax.set_xlim(-W / 2, W / 2)
    ax.set_ylim(-BEHIND, AHEAD)
    ax.set_aspect("equal")
    ax.axis("off")

    R = max(W, AHEAD + BEHIND)
    segs = {"lane": [], "edge": [], "center": []}
    for x1, y1, x2, y2, kind in near(grid, ex, ey, R):
        segs[kind].append((tf(x1, y1), tf(x2, y2)))
    ax.add_collection(LineCollection(segs["lane"], colors=C["lane"], linewidths=2.0, zorder=1))
    ax.add_collection(LineCollection(segs["edge"], colors=C["edge"], linewidths=2.6, zorder=1))
    ax.add_collection(LineCollection(segs["center"], colors=C["center"], linewidths=3.0, zorder=1))
    # 정지선 — routes/stoplines_all.json 의 (x, y, 진행방위). 차로 하나 폭으로 가로질러 긋는다.
    bars = []
    for sx, sy, sh in STOPLINES:
        if abs(sx - ex) > R or abs(sy - ey) > R:
            continue
        nx, ny = -math.sin(sh) * 1.6, math.cos(sh) * 1.6
        bars.append((tf(sx - nx, sy - ny), tf(sx + nx, sy + ny)))
    ax.add_collection(LineCollection(bars, colors=C["stop"], linewidths=5.0, zorder=2))

    # 계획 경로와 지금 따라가는 횡오프셋 경로
    pts = [tf(px, py) for px, py in route]
    vis = [(a, b) for a, b in pts if -W < a < W and -BEHIND - 5 < b < AHEAD + 5]
    if len(vis) > 1:
        ax.plot([p[0] for p in vis], [p[1] for p in vis], color=C["route"], lw=3.0,
                alpha=0.55, zorder=2, solid_capstyle="round")
    if abs(row["off"]) > 0.05 and len(vis) > 1:
        ax.plot([p[0] - row["off"] for p in vis], [p[1] for p in vis], color=C["route"], lw=4.5,
                zorder=3, solid_capstyle="round")

    # 물체 — 자차 기준(fx 앞, fy 왼쪽). 화면은 x 오른쪽이 +, 위가 앞.
    for fx, fy, ln, wd, ht, sp, dh in row["objs"]:
        cls = classify_object(ln, wd, ht)
        if cls is ObjectClass.ROAD_SURFACE:
            continue
        if cls is ObjectClass.VRU and not is_vru(ln, wd, ht, sp):
            cls = ObjectClass.VEHICLE                       # 40 km/h 넘게 달리는 이륜차는 차
        col = {ObjectClass.VEHICLE: C["veh"], ObjectClass.VRU: C["vru"]}.get(cls, C["obs"])
        # 시야 밖 **앞쪽** 물체는 위 가장자리에 ▲ 와 거리 — 비키기·서기를 시작한 이유가 화면 밖에 있을 때가 많다.
        # 내 차로 언저리(좌우 5 m)만 — 맞은편 차까지 그리면 속도 글자와 겹친다
        if fx >= AHEAD and abs(fy) >= OFFVIEW_LAT:
            continue
        if AHEAD <= fx < AHEAD + OFFVIEW:
            ax.plot([-fy], [AHEAD - 1.1], marker="^", markersize=18, color=col, zorder=7)
            ax.text(-fy, AHEAD - 2.5, f"{fx:.0f} m", color=col, fontsize=19, ha="center", va="top",
                    family="DejaVu Sans", zorder=7)
            continue
        if not (-BEHIND - 5 < fx < AHEAD + 5 and abs(fy) < W):
            continue
        if cls is ObjectClass.OBSTACLE and max(ln, wd) < 1.0:
            # 연료통·라바콘은 실제 크기(0.3~0.5 m)로는 안 보인다 — 자리에 마름모 표지를 겹쳐 그린다
            ax.plot([-fy], [fx], marker="D", markersize=20, color=col, markeredgecolor=C["bg"],
                    markeredgewidth=2.0, zorder=6)
        ang = math.radians(dh)
        rect = mpatches.Rectangle((-wd / 2, -ln / 2), wd, ln, linewidth=3.0, edgecolor=col,
                                  facecolor=col + "55", zorder=5)
        import matplotlib.transforms as mtf
        rect.set_transform(mtf.Affine2D().rotate(ang).translate(-fy, fx) + ax.transData)
        ax.add_patch(rect)

    # 자차(뒷축 = 원점)
    ax.add_patch(mpatches.FancyBboxPatch((-EGO_HALF_W, -EGO_REAR), 2 * EGO_HALF_W, EGO_REAR + EGO_FRONT,
                                         boxstyle="round,pad=0,rounding_size=0.35", linewidth=3.2,
                                         edgecolor=C["ego"], facecolor=C["ego"] + "30", zorder=6))


def draw_hud(fig, row, font):
    kmh = row["v"] * 3.6
    fig.text(0.05, 0.965, f"{kmh:.0f}", color=C["text"], fontsize=40, fontweight="bold",
             ha="left", va="top", family=font)
    fig.text(0.05, 0.835, "km/h", color=C["text2"], fontsize=15, ha="left", va="top", family=font)
    fig.text(0.05, 0.765, "속도 제약", color=C["text2"], fontsize=15, ha="left", va="top", family=font)
    fig.text(0.05, 0.705, row["cap"] or "-", color=C["text"], fontsize=19, ha="left", va="top",
             family="DejaVu Sans Mono")
    # 지시등·신호는 오른쪽 **아래** — 위 가장자리는 시야 밖 물체 ▲ 표시 자리
    for side, x0, ch in ((1, 0.70, "◀"), (2, 0.85, "▶")):
        on = row["sig"] == side
        fig.text(x0, 0.235, ch, color=(C["yellow"] if on else C["dim"]), fontsize=30, ha="left", va="top")
    st = row["tl"]
    if st in TL_COLOR:
        fig.text(0.70, 0.105, "신호", color=C["text2"], fontsize=15, ha="left", va="top", family=font)
        fig.text(0.86, 0.122, "●", color=C[TL_COLOR[st]], fontsize=26, ha="left", va="top")


# ──────────────────────────── 영상 ────────────────────────────
def find_offset(video, csv_rows, scan=(0, 180)):
    """출발 순간으로 offset 을 찾는다: 정지 화면(2초 이상)이 움직이기 시작하는 첫 시각."""
    import numpy as np
    W, H, FPS = 192, 108, 10
    raw = subprocess.run(["ffmpeg", "-v", "error", "-ss", str(scan[0]), "-t", str(scan[1] - scan[0]),
                          "-i", video, "-vf", f"fps={FPS},scale={W}:{H},format=gray", "-f", "rawvideo", "-"],
                         capture_output=True, check=True).stdout
    fr = np.frombuffer(raw, np.uint8).reshape(-1, H, W).astype(np.float32)
    band = fr[:, int(H * 0.30):int(H * 0.55), :]
    d = np.abs(np.diff(band, axis=0)).mean(axis=(1, 2))
    # 정지 화면(2초 이상) 뒤 움직이기 시작한 순간들 중 **가장 오래 서 있던 뒤**를 출발로 본다.
    # 시나리오 로드·카메라 전환도 짧은 정지를 만들지만, 제어기를 켜기 전 대기가 가장 길다.
    # 출발은 **서서히** 움직인다(실측 첫 2초 흐름 0.3~1.0, 중계 창은 차체가 화면 가운데를 가려 더 작다) —
    # 크기로 거르지 말고, 그 뒤 2초 동안 **거의 매 프레임** 정지 문턱을 넘으면 출발로 본다.
    # (카메라 전환 같은 한 번 튀는 변화는 그 뒤가 다시 가만히 있어 걸러진다)
    cands, still = [], 0
    for i, v in enumerate(d):
        if v < 0.25:
            still += 1
        else:
            if still >= 2 * FPS and (d[i:i + 2 * FPS] >= 0.25).mean() >= 0.8:
                cands.append((still, scan[0] + (i + 1) / FPS))
            still = 0
    if not cands:
        raise SystemExit("출발 순간을 못 찾았다 — --offset 을 직접 줄 것")
    t_move_v = max(cands)[1]
    t_move_c = next(r["t"] for r in csv_rows[1:]
                    if math.hypot(r["x"] - csv_rows[0]["x"], r["y"] - csv_rows[0]["y"]) > 0.02)
    return t_move_v - t_move_c


def refine_offset(video, rows, t0, dur, rough, kind):
    """장면 안의 **정지(stop) 또는 출발(go)** 순간으로 offset 을 다시 맞춘다.

    긴 녹화는 스트림 끊김으로 뒤로 갈수록 몇 초씩 밀린다(실측: 같은 판에서 출발 기준 109.3 s,
    15초 뒤 정지 기준 111.7 s). 장면마다 그 안의 사건으로 맞추는 게 확실하다.
    보닛 바로 앞 내 차로 노면의 흐름으로 본다 — 옆 차로 교통에 흔들리지 않는다.
    """
    import numpy as np
    seg = [r for r in rows if t0 - 1 <= r["t"] <= t0 + dur + 1]
    tev = None
    for a, b in zip(seg[:-1], seg[1:]):
        if kind == "stop" and a["v"] >= 0.05 and b["v"] < 0.05:
            tev = b["t"]; break
        if kind == "go" and a["v"] < 0.05 and b["v"] >= 0.3:
            tev = b["t"]; break
    if tev is None:
        raise SystemExit(f"장면 안에 '{kind}' 순간이 없다 — --refine 을 빼거나 다른 사건을 쓸 것")
    W, H, FPS, SPAN = 320, 180, 10, 6.0
    start = max(0.0, tev + rough - SPAN)
    raw = subprocess.run(["ffmpeg", "-v", "error", "-ss", f"{start:.2f}", "-t", str(2 * SPAN), "-i", video,
                          "-vf", f"fps={FPS},scale={W}:{H},format=gray", "-f", "rawvideo", "-"],
                         capture_output=True, check=True).stdout
    fr = np.frombuffer(raw, np.uint8).reshape(-1, H, W).astype(np.float32)
    band = fr[:, int(H * 0.66):int(H * 0.80), int(W * 0.30):int(W * 0.70)]
    d = np.abs(np.diff(band, axis=0)).mean(axis=(1, 2))
    sm = np.convolve(d, np.ones(5) / 5, mode="same")
    hold = FPS                                     # 1초
    for i in range(len(sm) - hold):
        win = sm[i:i + hold]
        if kind == "stop" and win.max() < 2.0 and sm[i] < 1.6 and (i == 0 or sm[i - 1] >= 1.6):
            return start + (i + 1) / FPS - tev
        if kind == "go" and i >= hold and sm[i - hold:i].max() < 1.6 and sm[i] > 3.0:
            return start + (i + 1) / FPS - tev
    raise SystemExit(f"영상에서 '{kind}' 순간을 못 찾았다 — --offset 을 직접 줄 것")


def main():
    ap = argparse.ArgumentParser(description="왼쪽 CSV 그림 | 오른쪽 VTD 녹화 GIF")
    ap.add_argument("--video", required=True)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--course", required=True, help="routes/<course>.json 을 쓴다")
    ap.add_argument("--t0", type=float, required=True, help="CSV 시각[s]에서 시작")
    ap.add_argument("--dur", type=float, default=7.0)
    ap.add_argument("--offset", type=float, help="영상 시각 = CSV 시각 + offset")
    ap.add_argument("--find-offset", action="store_true", help="녹화 첫 부분의 출발 순간으로 대략 맞춘다")
    ap.add_argument("--refine", choices=["stop", "go"],
                    help="장면 안의 정지/출발 순간으로 offset 을 다시 맞춘다(권장)")
    ap.add_argument("--fps", type=int, default=8)
    ap.add_argument("--vcrop", help="녹화에서 쓸 영역 W:H:X:Y (ffmpeg crop). 예: 화면 녹화의 속도계를 잘라낼 때")
    ap.add_argument("--height", type=int, default=300, help="GIF 높이[px]. 왼쪽 정사각형, 오른쪽 16:9")
    ap.add_argument("--xodr", default=os.path.join(ROOT, "map", "HL_FMA_VTD_LivingLab.xodr"))
    ap.add_argument("--cache", default=os.path.join(tempfile.gettempdir(), "hlfma_lane_segments.pkl"))
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from PIL import Image
    font = "NanumBarunGothic" if any("NanumBarunGothic" in f.name for f in font_manager.fontManager.ttflist) \
        else "DejaVu Sans"

    rows = load_rows(a.csv)
    route = json.load(open(os.path.join(ROOT, "routes", f"{a.course}.json"), encoding="utf-8"))["ego_route"]
    offset = a.offset
    if offset is None:
        if not a.find_offset:
            raise SystemExit("--offset 을 주거나 --find-offset 을 켤 것")
        offset = find_offset(a.video, rows)
    if a.refine:
        rough = offset
        offset = refine_offset(a.video, rows, a.t0, a.dur, offset, a.refine)
        print(f"offset {rough:.2f} -> {offset:.2f} s  ({a.refine} 순간으로 다시 맞춤)")
    print(f"offset = {offset:.2f} s  (영상 {a.t0 + offset:.1f}s 부터)")
    grid = lane_segments(a.xodr, a.cache)
    STOPLINES.extend(json.load(open(os.path.join(ROOT, "routes", "stoplines_all.json")))["stoplines"])

    Hh = a.height
    Wl, Wr = Hh, Hh * 16 // 9
    tmp = tempfile.mkdtemp(prefix="hlfma_gif_")
    subprocess.run(["ffmpeg", "-v", "error", "-ss", f"{a.t0 + offset:.2f}", "-t", f"{a.dur:.2f}", "-i", a.video,
                    "-vf", (f"crop={a.vcrop}," if a.vcrop else "") + f"fps={a.fps},scale={Wr}:{Hh}:flags=lanczos",
                    os.path.join(tmp, "v%04d.png")], check=True)
    vframes = sorted(f for f in os.listdir(tmp) if f.startswith("v"))

    W_M = 26.0                                  # 왼쪽 그림 가로=세로[m]
    VIEW_H = W_M
    BEHIND = 6.0
    fig = plt.figure(figsize=(Wl * 2 / 100, Hh * 2 / 100), dpi=100)
    fig.patch.set_facecolor(C["bg"])
    ax = fig.add_axes([0, 0, 1, 1])
    n = int(round(a.dur * a.fps))
    for k in range(n):
        row = row_at(rows, a.t0 + k / a.fps)
        render_left(ax, row, grid, route, W_M, VIEW_H - BEHIND, BEHIND)
        for t in list(fig.texts):
            t.remove()
        draw_hud(fig, row, font)
        fig.savefig(os.path.join(tmp, "l.png"), facecolor=C["bg"])
        left = Image.open(os.path.join(tmp, "l.png")).convert("RGB").resize((Wl, Hh), Image.LANCZOS)
        right = Image.open(os.path.join(tmp, vframes[min(k, len(vframes) - 1)])).convert("RGB")
        canvas = Image.new("RGB", (Wl + Wr, Hh), C["bg"])
        canvas.paste(left, (0, 0))
        canvas.paste(right, (Wl, 0))
        canvas.save(os.path.join(tmp, f"c{k:04d}.png"))
    plt.close(fig)

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    # 팔레트 = 영상에서 뽑은 72색 + 그림 색(C) 전부. 영상 색만 뽑으면 작은 신호 점·지시등이
    # 비슷한 영상 색에 흡수돼 초록불이 파랗게, 빨간불이 주황으로 나온다.
    frames = os.path.join(tmp, "c%04d.png")
    pal = os.path.join(tmp, "pal.png")
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", str(a.fps), "-i", frames,
                    "-vf", "palettegen=max_colors=72:stats_mode=diff", pal], check=True)
    cols = list(dict.fromkeys(Image.open(pal).convert("RGB").getdata()))
    for hx in C.values():
        cols.append(tuple(int(hx[i:i + 2], 16) for i in (1, 3, 5)))
    cols = list(dict.fromkeys(cols))[:256]
    img = Image.new("RGB", (16, 16))
    img.putdata(cols + [cols[-1]] * (256 - len(cols)))
    img.save(pal)
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", str(a.fps), "-i", frames, "-i", pal,
                    "-lavfi", "[0][1]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle",
                    "-loop", "0", a.out], check=True)
    print(f"만듦: {a.out}  ({os.path.getsize(a.out) / 1e6:.1f} MB, {n} 프레임, {Wl + Wr}x{Hh})")


if __name__ == "__main__":
    main()
