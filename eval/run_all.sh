#!/usr/bin/env bash
# 오프라인 회귀 전판 + **기준선 점수 대조**.
#
#   bash eval/run_all.sh
#
# ⚠️ PASS/FAIL 만 보면 안 된다(2026-08-30 변이 검증에서 실측): 적신호를 무시하게
#    만든 버그가 -20 을 먹고도 80점 = PASS 로 표시됐다. 합격선(60)은 대회 기준이지
#    회귀 기준이 아니다. 회귀의 기준은 **기준선에서 1점도 안 떨어지는 것**이다.
#    기준선: eval/expected_scores.json (**전판 100**).
#    ⚠️ passped_junction 은 오래 85 였는데, 그건 주행 결함이 아니라 하네스가
#       차로계획을 안 넘겨서였다(2026-09-08). run_eval.sh 가 `<시나리오>_lane.json`
#       을 넘기게 고친 뒤 100 이다 — 그쪽 주석에 실측이 있다.
#    의도한 변화로 점수가 오르면 그 파일을 같이 갱신할 것.
set -u
D="$(cd "$(dirname "$0")" && pwd)"
EXP="$D/expected_scores.json"
fail=0
for sc in "$D"/scenarios/*.json; do
  # ⚠️ `*_lane.json` 은 차로계획이라 건너뛴다. 그런데 **판 이름 자체가 `_lane` 으로
  #    끝나면 판이 통째로 사라진다.** 실측 2026-09-09: `vru_bicycle_lane.json` 을
  #    넣었더니 24판 중 23판만 돌고 **아무 경고도 없었다** — 조용히 빠지는 게 최악이다.
  #    그래서 **짝이 되는 판이 실제로 있을 때만** 차로계획으로 보고, 아니면 크게 알린다.
  case "$sc" in *_lane.json)
    if [ -f "${sc%_lane.json}.json" ]; then continue; fi
    echo "  ⚠️  ${sc##*/} — 짝이 되는 판이 없다(이름이 _lane 으로 끝나는 판은 건너뛰어진다)"
    fail=$((fail+1)); continue;;
  esac
  n=$(basename "$sc" .json)
  out=$(PORT=$((9900+RANDOM%90)) timeout 300 bash "$D/run_eval.sh" "$sc" 2>&1)
  got=$(echo "$out" | grep -oP '점수\s*:\s*\K[0-9]+' | tail -1)
  want=$(python3 -c "import json;print(json.load(open('$EXP')).get('$n','?'))")
  if [ "$got" = "$want" ]; then
    printf "  ✅ %-28s %s\n" "$n" "$got"
  else
    printf "  ❌ %-28s %s (기준 %s)\n" "$n" "${got:-실패}" "$want"
    echo "$out" | grep -oP '^\s*- \d+\s+\K.*' | sed 's/^/       /'
    fail=$((fail+1))
  fi
done
echo
[ $fail -eq 0 ] && echo "=== 기준선 일치 — 회귀 통과 ===" || echo "=== ❌ ${fail}판이 기준선과 다르다 ==="
exit $fail
