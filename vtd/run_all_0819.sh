#!/bin/bash
# 2026-08-19 마지막 검증 한 바퀴: 새 코스 2개 + 공식 회귀 18판.
# 새 코스가 앞에 온다 — 오늘 잡은 버그(순환 코스에서 앞부분 통째로 건너뛰기)가
# 거기서만 드러나므로, 그게 고쳐졌는지부터 본다.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
T="${1:?새 코스 경로가 있는 디렉터리}"

for c in A B; do
  bash "$HERE/run_scenario.sh" HL_FMA_NEW_$c "$T/fin3_$c.json" 2>&1 \
    | grep -E "####|===|리스폰\]|커버리지|속도\]|실여유|프레임율|프레임 |❌"
done
echo "---- 새 코스 끝, 공식 회귀 시작"
bash "$HERE/run_regression.sh"
