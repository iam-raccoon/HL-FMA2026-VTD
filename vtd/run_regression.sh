#!/bin/bash
# 전체 회귀 18판(공식 8 + 이벤트 10).
#
# ⚠️ v7 은 오랫동안 `--max-speed 11.1`(40km/h) 로 돌렸다. 이유는 '붕괴 이력' —
#    t≈80~100s 에 25Hz -> 3.4Hz 로 떨어지고 목표를 89m 지나치던 것. 그런데
#    **그 진범은 속도가 아니라 우리 경로 생성기였고 2026-08-18 에 고쳤다**
#    (경로가 연료통을 0.58m 로 스치던 것 -> 3.32m). 고친 뒤로도 상한만 계속 남아 있었다.
#    2026-08-20 상한 없이 실측: **88s · 40.6km/h · 프레임율 25Hz(최저 12.5) · 전부 0**
#    (상한 있을 때 123s · 29.0km/h). **35초 빠르다.** -> 상한 제거.
#
# ⚠️⚠️ **예정된 VTD 재시작을 하지 않는다.**
#    예전엔 5판마다 무조건 `ensure_vtd.sh --force` 를 불렀다(`if [ $((n % 5)) -eq 1 ]`).
#    2026-08-19 실측: 그게 **멀쩡한 VTD 를 망가뜨렸다.** 11판째 그 재시작 직후부터
#    시나리오가 안 올라가 `EV_PED` 가 두 번 실패하고 **조용히 건너뛰어졌고**,
#    `EV_OBSTACLE` 은 3분 넘게 로드가 안 됐다(화면은 검은 채). 프로세스도 9910 도
#    멀쩡한데 로드만 안 됐다. **앞 10판은 그 재시작을 두 번 넘기고도 잘 돌았다** —
#    즉 재시작이 필요해서 터진 게 아니라 **재시작이 터뜨린 것**이다.
#    남은 8판을 재시작 없이 연속으로 돌리니 전부 통과했다.
#
#    재시작이 정말 필요한 두 경우는 **이미 따로 잡혀 있다**:
#      · Traffic 모듈 사망   -> `ensure_vtd.sh`(비강제)가 매 판 앞에서 잡는다
#                              (지정차로 회전을 '경로 이탈'로 보고 Traffic 이 죽는 건)
#      · 타임아웃 판(1000초) -> `run_scenario.sh` 가 그 판 끝에 강제 재시작한다
#                              (moduleManager 만 죽은 채 끝나 alive() 가 YES 로 보이는 건)
#    그래서 여기서는 **실패했을 때만** 손대고, 그 판을 한 번 재시도한다.
#
# ⚠️ 그리고 예전엔 **한 판이 건너뛰어져도 그냥 계속 진행**했다. 마지막에 판 수를 세지
#    않으면 17판만 돌고도 '전부 통과' 로 보인다. 아래에서 세고, 모자라면 exit 1 한다.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
R=~/hlfma2026/routes
EXPECT=18
KEEP='####|===|리스폰\]|속도\]|실여유|프레임율|프레임 |❌'
N=0; OK=0; FAILED=""

run() {   # run <시나리오> <경로.json> [slow]
  N=$((N + 1))
  local S rc
  S="$HERE/run_scenario.sh"
  [ "${3:-}" = "slow" ] && S="$HERE/run_scenario_slow.sh"

  bash "$S" "$1" "$2" 2>&1 | grep -E "$KEEP"
  rc=${PIPESTATUS[0]}      # ⚠️ grep 이 아니라 **run_scenario 의** 종료코드를 봐야 한다

  # ★실패했을 때만 VTD 를 손댄다. 그리고 **한 번만** 재시도한다 — 두 번 이상 매달리면
  #   판당 10분씩 태우고도 결국 못 건진다(로드 실패는 120초 × 2회를 이미 쓴 뒤다).
  if [ "$rc" -ne 0 ]; then
    echo "---- $1 실패(rc=$rc) -> VTD 강제 재시작 후 한 번만 재시도"
    bash "$HERE/ensure_vtd.sh" --force 2>&1 | tail -1
    bash "$S" "$1" "$2" 2>&1 | grep -E "$KEEP"
    rc=${PIPESTATUS[0]}
  fi

  if [ "$rc" -ne 0 ]; then
    echo "❌❌ $1 — 재시도까지 실패. **이 판은 결과가 없다**"
    FAILED="$FAILED $1"
  else
    OK=$((OK + 1))
  fi
}

for v in v1 v2 v3 v4 v5 v6; do run HL_FMA_VTD_LivingLab_$v $R/planned_from_xml_v1.json; done
run HL_FMA_VTD_LivingLab_v7 $R/planned_v7.json
run HL_FMA_WP9 $R/planned_wp9.json
for e in EV_LEADBRAKE EV_CUTIN EV_PED EV_OBSTACLE EV_ONCOMING \
         EV_CURVE EV_REARPASS EV_BOTHBLOCK EV_COMBO_PASSPED EV_COMBO_CHAIN; do
  run $e $R/planned_from_xml_v1.json
done

echo "======== 회귀 종료: $OK/$N 판 결과 있음 (예정 $EXPECT 판)"
if [ -n "$FAILED" ] || [ "$N" -ne "$EXPECT" ] || [ "$OK" -ne "$EXPECT" ]; then
  [ -n "$FAILED" ] && echo "❌ 결과 없는 판:$FAILED"
  [ "$N" -ne "$EXPECT" ] && echo "❌ 판 수가 안 맞는다 ($N ≠ $EXPECT) — 목록을 확인할 것"
  echo "   ⚠️ **판이 모자란 회귀는 '무결점'이 아니다.** 위 결과를 그대로 믿지 말 것."
  echo FULL_DONE_WITH_SKIPS
  exit 1
fi
echo FULL_DONE
