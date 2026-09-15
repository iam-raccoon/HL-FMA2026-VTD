#!/bin/bash
# 레포의 `scenarios/*.xml` 을 VTD PC(OMEN)로 올린다.
#
# 왜 필요한가(2026-08-20): 우리가 만든 시나리오 17개(EV 10 + NEW 6 + OFFEX)가
# **OMEN 에만 있었다.** 초기화되면 통째로 날아가는데, `HL_FMA_NEW_*` 는 만들 때 쓴
# 좌표를 어디에도 안 적어놔서 **재생성도 불가능**했다(실제로 D·G·H 좌표를 찾다 실패했다).
# 이제 XML 도 좌표도 레포에 있으니, OMEN 은 **배포 대상일 뿐**이다. 밀려도 이 한 줄로 복구된다.
#
#   bash vtd/push_scenarios.sh           # 전부
#   bash vtd/push_scenarios.sh EV_PED HL_FMA_NEW_D
#
# ⚠️ 공식 시나리오(HL_FMA_VTD_LivingLab_v*, HL_FMA_WP9)는 **올리지 않는다.**
#    주최측이 준 것이라 우리가 덮어쓸 이유가 없고, 덮어쓰면 원본을 잃는다.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$HERE/../scenarios"
OM="${OM:-${VTD_SSH:-user@192.168.50.11}}"
DST='~/Hexagon/VTD.2025.2/Data/Projects/SampleProject/Scenarios'
# ★sshpass 가 없는 PC 도 있다(2026-09-11 제어PC 실측: `sshpass: command not found`).
#   없으면 그냥 scp 로 간다 — 키가 있으면 그대로 되고, 없으면 비밀번호를 물어본다.
#   대회 전날 도구 하나 없다고 시나리오를 못 올리는 일은 없어야 한다.
if command -v sshpass >/dev/null 2>&1; then
  SP="sshpass -p ${OMEN_PW:?OMEN_PW 에 VTD PC 비밀번호를 넣을 것}"
else
  SP=""
  echo "  ⚠️ sshpass 가 없다 — 비밀번호를 직접 입력하면 된다."
fi

if [ $# -gt 0 ]; then
  FILES=(); for n in "$@"; do FILES+=("$SRC/$n.xml"); done
else
  FILES=("$SRC"/*.xml)
fi

for f in "${FILES[@]}"; do
  [ -s "$f" ] || { echo "❌ 없다: $f"; exit 1; }
  # ★올리기 전에 파싱된다는 걸 확인한다. 실측 2026-08-19: `HL_FMA_NEW_D/E/G` 는
  #   XML 앞에 plan() 의 진행 로그가 섞여 깨져 있었고, **그래서 VTD 에 영영 안 올라갔다.**
  #   깨진 파일을 올려놓고 VTD 앞에서 5분씩 기다리는 것만큼 아까운 게 없다.
  python3 -c "import xml.etree.ElementTree as ET,sys; ET.parse(sys.argv[1])" "$f" \
    || { echo "❌ XML 이 깨졌다: $f"; exit 1; }
done

echo "올린다: ${#FILES[@]}개 -> $OM"
$SP scp -o StrictHostKeyChecking=no -q "${FILES[@]}" "$OM:$DST/" || exit 1
echo "✅ 완료"
