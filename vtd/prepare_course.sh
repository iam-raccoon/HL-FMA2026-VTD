#!/bin/bash
# ★대회 당일 절차 그대로. 좌표만 받아서 주행 직전까지 준비한다.
#
#   bash vtd/prepare_course.sh <시나리오이름> <출력디렉터리> x1 y1 x2 y2 [x3 y3 ...]
#
#   ① (연습용) 그 좌표로 시나리오 XML 을 만들어 VTD 에 올린다
#      — 당일엔 주최측 시나리오가 이미 있으므로 MAKE_SCENARIO=0 으로 건너뛴다
#   ② 시나리오 로드
#   ③ **ego 헤딩을 읽는다**  <- 이 단계가 없어서 코스 하나가 통째로 어긋났다
#   ④ 그 헤딩으로 경로 계산 -> check_route -> 차로계획
#
# 왜 ③ 이 필요한가(실측 2026-08-19, 새 코스 A): 출발 X,Y 를 담는 차로가 왕복도로에서
# 한쪽 방향뿐이었는데 VTD 는 코스 방향에 맞춰 **반대 차로에** 차를 놓았다. 우리 경로가
# 178° 반대로 시작했고, 차는 앞으로 가버려 **코스의 44% 를 안 갔다**(커버리지 56%).
# 좌표만으로는 방향을 알 수 없다. 차를 놓고 물어보는 수밖에 없다.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
X="${X:-$HOME/hlfma2026_map/HL_FMA_VTD_LivingLab.xodr}"
TMPL="${TMPL:-$HOME/hlfma2026_map/HL_FMA_WP9.xml}"
OM="${OM:-${VTD_SSH:-user@192.168.50.11}}"
VTD_IP="${VTD_IP:-$(python3 "$(dirname "$0")/../src/netcfg.py" --host-only)}"   # 자동판별(src/netcfg.py)
LAP="${LAP:-${CTRL_SSH:-user@192.168.50.10}}"           # 제어PC
SCEN="~/Hexagon/VTD.2025.2/Data/Projects/SampleProject/Scenarios"
MAKE_SCENARIO="${MAKE_SCENARIO:-1}"
SP="sshpass -p ${OMEN_PW:?OMEN_PW 에 VTD PC 비밀번호를 넣을 것}"
SSH="$SP ssh -o StrictHostKeyChecking=no -o ConnectTimeout=20"

NAME="${1:?시나리오 이름}"; shift
OUT="${1:?출력 디렉터리}"; shift
[ $# -ge 4 ] || { echo "좌표를 x y 쌍으로 2개 이상"; exit 1; }
R="$OUT/$NAME.json"

if [ "$MAKE_SCENARIO" = "1" ]; then
  echo "① 시나리오 생성/전송"
  python3 "$HERE/make_course.py" "$X" "$TMPL" "$@" > "$OUT/$NAME.xml" || exit 1
  $SP scp -o StrictHostKeyChecking=no -q "$OUT/$NAME.xml" "$OM:$SCEN/" || exit 1
fi

echo "② 시나리오 로드"
bash "$HERE/ensure_vtd.sh" | tail -1
# ⚠️ 한 번 실패했다고 포기하면 안 된다. 앞판이 900초를 넘겨 끝나면 run_scenario 가 VTD 를
#    강제 재시작하는데, 그 직후 첫 로드는 잘 실패한다(안정화가 덜 된 것뿐이다).
#    실측 2026-08-19: 코스 A 가 995초로 끝난 뒤 코스 D 가 '로드 실패'로 통째로 건너뛰어졌다.
#    run_scenario.sh 에는 같은 재시도가 이미 있다 — 여기만 빠져 있었다.
#  ⚠️⚠️ 출력을 **버리지 말 것.** 실측 2026-08-19: `>/dev/null 2>&1` 로 묻어놨더니
#     코스 D·E 가 '로드 실패'로 건너뛰어졌는데 **왜인지 알 방법이 없었다.**
#     load_scenario.py 는 어디를 클릭했는지, 모달을 닫았는지, streaming 이 왜 FAIL 인지를
#     전부 찍어준다 — 그걸 버리면 그 노력이 통째로 사라진다.
LOG="/tmp/_load_$NAME.log"
loaded=0
for try in 1 2 3; do
  if timeout 150 $SSH "$OM" "cd ~/hlfma2026/vtd && python3 load_scenario.py $NAME" \
       >>"$LOG" 2>&1; then
    loaded=1; break
  fi
  echo "   로드 실패(${try}차) -> 30초 기다렸다 재시도  [$LOG]"
  tail -3 "$LOG" | sed 's/^/      /'
  sleep 30
done
[ $loaded -eq 1 ] || { echo "❌ 로드 실패 — 건너뜀 (자세한 건 $LOG)"; tail -12 "$LOG"; exit 1; }

echo "③ ego 헤딩 읽기"
POSE=$(timeout 40 $SSH "$LAP" "cd ~/hlfma2026 && python3 vtd/ego_pose.py --host $VTD_IP --json" 2>/dev/null | tail -1)
HD=$(python3 -c "import json,sys; print(f\"{json.loads(sys.argv[1])['heading_deg']:.2f}\")" "$POSE" 2>/dev/null)
[ -n "${HD:-}" ] || { echo "❌ ego 헤딩을 못 읽었다: $POSE"; exit 1; }
echo "   ego $POSE"

echo "④ 경로 계산 (--heading $HD)"
python3 "$HERE/plan_route.py" "$X" --heading "$HD" "$@" "$R" || exit 1
python3 "$HERE/../eval/check_route.py" "$R" "$X" | tee "/tmp/_cr_$NAME.txt"
grep -q "사용 가능" "/tmp/_cr_$NAME.txt" || { echo "❌ 검사 실패 — 주행하지 말 것"; exit 1; }
python3 "$HERE/build_lane_plan.py" "$X" "$R" "${R%.json}_lane.json" >/dev/null || exit 1
echo "✅ 준비 완료 -> bash vtd/run_scenario.sh $NAME $R"
