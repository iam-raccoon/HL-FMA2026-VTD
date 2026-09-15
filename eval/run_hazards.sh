#!/usr/bin/env bash
# 돌발상황 시나리오 일괄 실행 + 요약. 사용: bash eval/run_hazards.sh
D="$(cd "$(dirname "$0")" && pwd)"
PORT_BASE="${PORT_BASE:-9930}"
i=0
for sc in "$D"/scenarios/hz_*.json; do
  i=$((i+1)); P=$((PORT_BASE+i))
  echo "######## $(basename "$sc")"
  PORT=$P bash "$D/run_eval.sh" "$sc" 2>&1 \
    | grep -E "시나리오|점수|목표도달|감점|최소 접근|actor|충돌" | sed 's/^/  /'
done
