#!/bin/bash
# 처음 보는 코스 하나를 **대회 당일 절차 그대로** 만들어 VTD 에 올린다.
#
#   bash vtd/add_course.sh <이름> <출력디렉터리> x1 y1 x2 y2 [x3 y3 ...]
#
# 하는 일: 경로계획 -> check_route -> 차로계획 -> 시나리오 XML -> VTD 로 전송.
# 검사에서 '쓰면 안 된다'가 나오면 **거기서 멈춘다** — 못 쓸 경로로 VTD 를 5분 돌리는
# 것만큼 아까운 게 없다.
#
# 그다음:  bash vtd/run_scenario.sh <이름> <출력디렉터리>/<이름>.json
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
X="${X:-$HOME/hlfma2026_map/HL_FMA_VTD_LivingLab.xodr}"
TMPL="${TMPL:-$HOME/hlfma2026_map/HL_FMA_WP9.xml}"   # 실제로 도는 시나리오를 템플릿으로
OM="${OM:-${VTD_SSH:-user@192.168.50.11}}"
SCEN="~/Hexagon/VTD.2025.2/Data/Projects/SampleProject/Scenarios"

NAME="${1:?코스 이름 (예: HL_FMA_NEW_C)}"; shift
OUT="${1:?출력 디렉터리}"; shift
[ $# -ge 4 ] || { echo "좌표를 x y 쌍으로 2개 이상"; exit 1; }

R="$OUT/$NAME.json"
echo "### $NAME  경유지: $*"
python3 "$HERE/plan_route.py" "$X" "$@" "$R" || exit 1
# ★검사가 '쓰면 안 된다'면 여기서 끝낸다. 실측 2026-08-19: check_route 를 통과한
#   코스에서도 VTD 로 돌려야만 드러나는 게 있었지만(순환 코스 44% 미주행), 그렇다고
#   검사에서 걸린 걸 굳이 돌려볼 이유는 없다.
python3 "$HERE/../eval/check_route.py" "$R" "$X" | tee /tmp/_cr_$NAME.txt
grep -q "사용 가능" /tmp/_cr_$NAME.txt || { echo "❌ 검사 실패 — VTD 로 안 보낸다"; exit 1; }
python3 "$HERE/build_lane_plan.py" "$X" "$R" "${R%.json}_lane.json" >/dev/null || exit 1
python3 "$HERE/make_course.py" "$X" "$TMPL" "$@" > "$OUT/$NAME.xml" || exit 1
sshpass -p "${OMEN_PW:?OMEN_PW 에 VTD PC 비밀번호를 넣을 것}" scp -o StrictHostKeyChecking=no -q "$OUT/$NAME.xml" "$OM:$SCEN/" || exit 1
echo "✅ $NAME 준비 완료 -> bash vtd/run_scenario.sh $NAME $R"
