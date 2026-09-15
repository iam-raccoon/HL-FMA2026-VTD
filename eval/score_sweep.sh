#!/usr/bin/env bash
# OMEN 에서 주행 CSV 를 받아 코스별로 전부 채점한다.
#   bash eval/score_sweep.sh [코스...]        (기본: A B D E G H)
# 왜: 코스마다 check_run/check_crosswalks 를 손으로 부르면 인자(제한속도·경로·차로계획)를
#     틀리기 쉽다. 실제로 13:36 에 vlim 자리에 경로 파일을 넣어 죽었다.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OMEN="${OMEN:-${VTD_SSH:-user@192.168.50.11}}"
PW="${PW:?PW 에 VTD PC 비밀번호를 넣을 것}"
DST="$HERE/runs_tr"; mkdir -p "$DST"
COURSES=("$@"); [ ${#COURSES[@]} -eq 0 ] && COURSES=(A B D E G H)
for C in "${COURSES[@]}"; do
  R="$HERE/routes/HL_FMA_NEW_$C.json"; L="${R%.json}_lane.json"
  sshpass -p "$PW" scp -q -o StrictHostKeyChecking=no \
      "$OMEN:/home/user/runs_tr/run_${C}_TR.csv" "$DST/" 2>/dev/null || { echo "### $C — CSV 없음"; continue; }
  echo; echo "################ 코스 $C ################"
  # ★제한속도 인자는 **최댓값**을 준다. 구간별 채점은 차로계획(4번째 인자)이 한다.
  MAX=$(python3 -c "import json,sys;p=json.load(open('$L'))['pts'];print(max(q['lim'] for q in p if q))")
  python3 "$HERE/eval/check_run.py" "$DST/run_${C}_TR.csv" "$MAX" "$R" "$L" 2>&1 | tail -12
  python3 "$HERE/eval/check_crosswalks.py" "$DST/run_${C}_TR.csv" "$R" "$L" 2>&1 | tail -8
done
