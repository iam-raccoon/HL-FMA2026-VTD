#!/usr/bin/env bash
# ★**라우터를 건드릴 때의 안전망.** 저장된 경유지로 코스 8개를 다시 계획해
#   기존 `routes/*.json` 과 대조한다.
#
#     bash vtd/replan_all.sh              # 재계산 + 기존과 대조
#     bash vtd/replan_all.sh /tmp/out     # 결과를 거기에 두고 대조
#
# 왜 필요한가: `vtd/plan_route.py` 는 대회 당일 경로를 만드는 핵심이라, 고치고 나서
# "안 깨졌나"를 물을 방법이 있어야 한다. 예전 티켓에는 "원본 경유지를 보관하지 않아
# 재계산 대조가 안 된다"고 적혀 있었는데 **틀렸다** — 아래 두 곳에 다 있다:
#   · `scenarios/course_waypoints.json`  (A·B·D·E·G·H 4점씩 + spawn_heading 실측)
#   · `routes/pretest/route_pretest_*.csv` (주최측이 준 사전테스트 경로)
# 2026-09-04 실측: 이 스크립트로 7/7 이 **최대 어긋남 0.00m** 로 재현됐다.
set -u
H="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-/tmp/replan_$$}"
mkdir -p "$OUT"
python3 - "$H" "$OUT" <<'PY'
import csv, json, math, os, subprocess, sys
H, out = sys.argv[1], sys.argv[2]
X = f"{H}/map/HL_FMA_VTD_LivingLab.xodr"
wp = json.load(open(f"{H}/scenarios/course_waypoints.json", encoding="utf-8"))

jobs = []
for name, c in wp.items():
    if name.startswith("_"):
        continue
    pts = list(c["waypoints"])
    if c.get("start_from_ego"):      # 첫 경유지 차로가 스폰 방향으로 못 달리는 코스(D·E·G)
        pts[0] = c["ego_spawn"]
    p = f"{out}/{name}.csv"
    with open(p, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["x", "y"])
        for x, y in pts:
            w.writerow([x, y])
    jobs.append((name, p, ["--heading", str(c["spawn_heading_deg"])],
                 f"{H}/routes/{name}.json"))
for n, src in (("pretest1", f"{H}/routes/pretest/route_pretest_1.csv"),
               ("pretest2", f"{H}/routes/pretest/route_pretest_2.csv")):
    if os.path.exists(src):
        jobs.append((n, src, [], f"{H}/routes/race.json" if n == "pretest1" else None))

print(f"{'코스':16} {'점수':>7} {'거리':>8}  {'기존 대비 최대 어긋남':>18}")
fail = 0
for name, src, extra, ref in jobs:
    dst = f"{out}/{name}.json"
    r = subprocess.run(["python3", f"{H}/vtd/plan_route.py", X, *extra,
                        "--from-csv", src, dst], capture_output=True, text=True, timeout=900)
    if r.returncode != 0 or not os.path.exists(dst):
        print(f"  {name:14} ❌ 계획 실패"); fail += 1; continue
    a = json.load(open(dst, encoding="utf-8"))["ego_route"]
    L = sum(math.hypot(a[i][0]-a[i-1][0], a[i][1]-a[i-1][1]) for i in range(1, len(a)))
    tag = "기준 없음"
    if ref and os.path.exists(ref):
        b = json.load(open(ref, encoding="utf-8"))["ego_route"]
        m = max(min(math.hypot(p[0]-q[0], p[1]-q[1]) for q in b) for p in a[::5])
        tag = f"{m:.2f}m " + ("✅ 같다" if m < 0.5 else "⚠️ 달라졌다")
        if m >= 0.5:
            fail += 1
    print(f"  {name:14} {len(a):>7} {L:7.0f}m  {tag:>18}")
print()
print("=== 전부 재현 ===" if not fail else f"=== ⚠️ {fail}개가 기존과 다르다 — 의도한 변화인지 확인할 것 ===")
raise SystemExit(0)
PY
