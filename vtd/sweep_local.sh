#!/bin/bash
# 시나리오 여러 판을 **VTD PC 한 대에서** 순차로 돌리고 그 자리에서 채점한다.
#
#   bash vtd/sweep_local.sh v1 v2 v3 EV_PED ...        (VTD PC 에서 직접)
#
# 왜 따로 만들었나(2026-08-26): `run_regression.sh` 는 주행을 **제어PC(노트북)** 에서
# 띄운다. 그런데 노트북을 다시 깔면서 SSH 가 막혀 18판이 첫 줄에서 rc=1 로 죽었다
# ("제어PC 접속 실패"). 공식 시나리오 회귀가 노트북 한 대에 묶여 있을 이유는 없다 —
# `sweep_tr.sh` 가 이미 루프백(127.0.0.1)으로 잘 돌고 있었다. 같은 방식으로 돌린다.
#
# ⚠️ 경로 파일 짝은 **drive.sh 가 아는 것 하나뿐**이다. 여기서 다시 case 문을 쓰면
#    언젠가 갈라진다 — drive.sh 가 찍는 "주행: <경로>" 줄을 되읽어 채점에 쓴다.
# ⚠️ 얼어붙은 판은 freezewatch.sh 가 끊는다.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${OUT:-/home/user/runs_local}"
mkdir -p "$OUT"
cd "$HERE"
N=0; OK=0; FAILED=""
for A in "$@"; do
  case "$A" in v1|v2|v3|v4|v5|v6|v7) S="HL_FMA_VTD_LivingLab_$A" ;; *) S="$A" ;; esac
  N=$((N + 1))
  echo; echo "######## $S  $(date +%H:%M:%S)"
  if ! timeout 400 python3 -u vtd/load_scenario.py "$S" > "$OUT/$S.load.log" 2>&1; then
      echo "❌ 로드 실패 — 건너뜀"; tail -3 "$OUT/$S.load.log"; FAILED="$FAILED $S"; continue
  fi
  CSV="$OUT/run_$S.csv"; rm -f "$CSV"       # 옛 CSV 로 채점하는 사고를 막는다
  nohup bash vtd/freezewatch.sh "$CSV" 150 >> "$OUT/$S.log" 2>&1 &
  timeout 900 env HOST=127.0.0.1 CSV="$CSV" bash drive.sh "$S" > "$OUT/$S.log" 2>&1
  echo "  종료 rc=$? $(tail -1 "$OUT/$S.log" | cut -c1-70)"
  [ -s "$CSV" ] || { echo "❌ 주행 CSV 가 없다(기동 실패). 채점 안 함"; FAILED="$FAILED $S"; continue; }
  R="$HERE/routes/$(grep -oP '주행: \K[^ ]+' "$OUT/$S.log" | head -1)"
  L="${R%.json}_lane.json"
  [ -s "$R" ] || { echo "❌ 경로 파일을 못 찾았다: $R"; FAILED="$FAILED $S"; continue; }
  MAX=$(python3 -c "import json;p=json.load(open('$L'))['pts'];print(max(q['lim'] for q in p if q))")
  python3 - "$CSV" <<'PY'
import csv, statistics, sys
r = list(csv.DictReader(open(sys.argv[1])))
d = [float(r[i]['t']) - float(r[i-1]['t']) for i in range(1, len(r))]
hz = 1.0 / statistics.median(d) if d else 0
print(f"[프레임율] {hz:.1f} Hz" + ("" if hz > 12 else "  ← ⚠️ 무너짐. 이 판 결과는 못 믿는다"))
PY
  python3 eval/check_run.py "$CSV" "$MAX" "$R" "$L" 2>&1 | tail -12
  OK=$((OK + 1))
  sleep 5
done
echo; echo "======== 종료: $OK/$N 판 결과 있음$( [ -n "$FAILED" ] && echo "  ❌ 결과 없는 판:$FAILED" )"
echo "ALLDONE $(date +%H:%M:%S)"
