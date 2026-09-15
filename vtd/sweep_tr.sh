#!/bin/bash
# 주변교통 판(`_TR`)을 코스 목록만큼 순차로: 로드 -> 녹화 -> 주행. 사람 개입 없음.
#
#   bash vtd/sweep_tr.sh G H E A B D        (VTD PC 에서 직접 돌린다)
#
# 왜: 한 판이 10~13분이라 6판이면 70분이다. 손으로 붙어 있을 일이 아니다.
# ⚠️ 로드는 `vtd/load_scenario.py` 가 SCP + GUI 조작(xdotool, DISPLAY=:1)으로 한다.
# ⚠️ 얼어붙은 판은 `freezewatch.sh` 가 끊는다 — 없으면 1600초 타임아웃까지 태운다.
set -u
cd /home/user/hlfma2026
OUT=/home/user/runs_tr
for C in "$@"; do
  N=HL_FMA_NEW_${C}_TR
  echo "=== $C 로드 $(date +%H:%M:%S) ==="
  : > "$OUT/${C}_TR.load.log"
  # ★★로드는 **3회까지 다시 해 본다.** VTD Traffic 모듈은 3판쯤 연속으로 돌리면 죽고
  #   `ensure_vtd.sh` 가 강제 재시작하는데, **그 직후 첫 로드는 잘 실패한다** — 안정화가
  #   덜 된 것뿐이라 30초 뒤에 다시 하면 붙는다. 실측 2026-08-19: 코스 A 가 995초로 끝난
  #   뒤 코스 D 가 '로드 실패' 로 통째로 건너뛰어졌다.
  #   `prepare_course.sh` 와 `run_scenario.sh` 에는 이미 같은 재시도가 있다 —
  #   무인 스윕인 여기만 빠져 있었다. 한 판이 10~13분이라 건너뛰면 그날 밤을 버린다.
  # ⚠️ 로그를 **덮어쓰지 말 것.** 왜 실패했는지는 load_scenario.py 만 안다 —
  #    차수별로 붙여 써서 세 번의 이유를 다 남긴다.
  loaded=0
  for try in 1 2 3; do
      if timeout 400 python3 -u vtd/load_scenario.py "$N" >> "$OUT/${C}_TR.load.log" 2>&1; then
          loaded=1; break
      fi
      echo "  로드 실패(${try}차) -> 30초 뒤 재시도"
      tail -3 "$OUT/${C}_TR.load.log" | sed 's/^/     /'
      sleep 30
  done
  if [ $loaded -ne 1 ]; then
      echo "$C 로드 실패 — 3회 다 실패해서 건너뜀"; tail -8 "$OUT/${C}_TR.load.log"; continue
  fi
  nohup bash vtd/record_run.sh "$OUT/${C}_TR.mp4" 1500 127.0.0.1 > "$OUT/${C}_TR.rec.log" 2>&1 &
  REC=$!
  nohup bash /home/user/hlfma2026/vtd/freezewatch.sh "$OUT/run_${C}_TR.csv" 150 >> "$OUT/${C}_TR.log" 2>&1 &
  echo "=== $C 주행 $(date +%H:%M:%S) ==="
  timeout 1600 env HOST=127.0.0.1 CSV="$OUT/run_${C}_TR.csv" bash drive.sh "$N" > "$OUT/${C}_TR.log" 2>&1
  echo "=== $C 종료 $(date +%H:%M:%S) rc=$? $(tail -1 "$OUT/${C}_TR.log" | cut -c1-60) ==="
  kill -INT $REC 2>/dev/null; sleep 3; kill $REC 2>/dev/null
  sleep 5
done
echo "ALLDONE $(date +%H:%M:%S)"
