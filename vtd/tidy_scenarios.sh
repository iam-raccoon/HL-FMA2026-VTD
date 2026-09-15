#!/bin/bash
# VTD PC(OMEN)의 시나리오 폴더에서 **실험 잔재만** 보관함으로 치운다.
#
#   bash vtd/tidy_scenarios.sh          # 무엇을 치울지 보여주기만 한다(기본)
#   bash vtd/tidy_scenarios.sh --go     # 실제로 옮긴다
#
# ⚠️ **지우지 않는다. 옮긴다.** `_attic/` 으로 보내므로 되돌릴 수 있다.
#    2026-08-20 기준 59개 중 25개만 쓰고 있었는데, 그 25개가 뭔지 아무 데도 안 적혀
#    있어서 목록 만드는 데만 한참 걸렸다. 아래 세 목록이 그 답이다.
#
# ⚠️ VTD 기본 예제(TrafficDemo, Parking, Glare …)는 **건드리지 않는다.**
#    새 프로젝트를 만들 때 템플릿으로 쓰이고, 우리 것도 아니다.
set -u
OM="${OM:-${VTD_SSH:-user@192.168.50.11}}"
S='~/Hexagon/VTD.2025.2/Data/Projects/SampleProject/Scenarios'
SSH="sshpass -p ${OMEN_PW:?OMEN_PW 에 VTD PC 비밀번호를 넣을 것} ssh -o StrictHostKeyChecking=no -o ConnectTimeout=20"

# 치울 것 — 전부 우리가 만든 일회성 실험이다. 무엇이었는지 남겨둔다.
ATTIC=(
  REC_v1 REC_v2 REC_v3 REC_v4 REC_v5 REC_v6 REC_v7 REC2_v1 REC2_v7  # 공식 경로 녹화용
  V7_1CAN V7_AI1PARK V7_CAN1 V7_CANFAR V7_CANFIX                     # v7 붕괴 원인 추적
  V7_NOAI V7_NOAI1 V7_NOBUS V7_NOCAN V7_PYLON                        #   (오브젝트를 하나씩 뺀 판)
  HL_FMA_OFF5 HL_FMA_OFF6                                            # 오프라인 예제 초기본
  HL_FMA_DEMO HL_FMA_DRIVE HL_FMA_ROUTE HL_FMA_WATCH                 # 초기 실험
  HL_FMA_VTD_LivingLab                                               # 공식 v1~v7 이전의 원본
)

echo "=== 남길 것 (25개) ==="
echo "  공식 8   HL_FMA_VTD_LivingLab_v1~v7 · HL_FMA_WP9   (주최측. 절대 건드리지 말 것)"
echo "  이벤트 10 EV_*                                      (scenarios/ 에 백업 있음)"
echo "  새 코스 6 HL_FMA_NEW_A B D E G H                    (scenarios/ 에 백업 있음)"
echo "  기타 1   HL_FMA_OFFEX"
echo "  + VTD 기본 예제 9개(TrafficDemo, Parking, Glare …) — 건드리지 않는다"
echo
echo "=== 치울 것 (${#ATTIC[@]}개) -> _attic/ ==="
printf '  %s\n' "${ATTIC[@]}" | paste -d' ' - - - | column -t

[ "${1:-}" = "--go" ] || { echo; echo "(보여주기만 했다. 실제로 옮기려면 --go)"; exit 0; }

echo; echo "옮기는 중..."
LIST=$(printf '%s.xml ' "${ATTIC[@]}")
timeout 60 $SSH "$OM" "cd $S && mkdir -p _attic && for f in $LIST; do
  [ -e \"\$f\" ] && mv \"\$f\" _attic/ && echo \"  옮김 \$f\"
done; echo; echo \"남은 xml: \$(ls -1 *.xml 2>/dev/null | wc -l)개\"" || exit 1
echo "✅ 완료 — 되돌리려면 OMEN 의 _attic/ 에서 꺼내면 된다"
