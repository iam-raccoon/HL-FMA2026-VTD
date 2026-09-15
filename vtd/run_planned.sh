#!/bin/bash
# 계획 경로로 공식 시나리오 실행 — 개발머신에서 돌린다.
#   bash vtd/run_planned.sh v1 [경로.json]      기본: routes/planned_from_xml_v1.json
#
# ⚠️ **경로 파일을 매번 올린다.** 예전엔 제어PC 의 /tmp/plan.json 을 그냥 썼는데,
#    다른 코스를 돌린 뒤 잊고 실행하면 **엉뚱한 경로로 달린다**(실측 2026-08-16:
#    v1 시나리오에서 9경유지 경로 2397m 를 달려 '미완주'로 나왔다. 코드 문제가 아니었다).
#    로그 첫 줄에 어떤 경로를 쓰는지 찍는다.
set -u
V="${1:-v1}"
ROUTE="${2:-$HOME/hlfma2026/routes/planned_from_xml_v1.json}"
LANEP="${ROUTE%.json}_lane.json"
SP="sshpass -p ${OMEN_PW:?OMEN_PW 에 VTD PC 비밀번호를 넣을 것}"
OM="${VTD_SSH:-user@192.168.50.11}"; LAP="${CTRL_SSH:-user@192.168.50.10}"
SSH="$SP ssh -o StrictHostKeyChecking=no -o ConnectTimeout=20"
echo "######## $V  ($(basename $ROUTE))"
$SP scp -o StrictHostKeyChecking=no -q "$ROUTE" $LAP:/tmp/plan.json
$SP scp -o StrictHostKeyChecking=no -q "$LANEP" $LAP:/tmp/plan_lane.json
bash ~/hlfma2026/vtd/ensure_vtd.sh 2>&1 | tail -1
timeout 90 $SSH $OM "cd ~/hlfma2026/vtd && python3 load_scenario.py HL_FMA_VTD_LivingLab_$V" >/dev/null || exit 1
timeout 30 $SSH $LAP "cd ~/hlfma2026/src
for p in \$(pgrep -x python3 2>/dev/null); do kill -9 \$p; done
setsid python3 -u main.py --scenario /tmp/plan.json --port 9910 \
  --tl-map ~/hlfma2026/routes/tl_map_livinglab.json --lane-plan /tmp/plan_lane.json \
  --log-csv /tmp/$V.csv >/tmp/$V.log 2>&1 </dev/null &
sleep 2" >/dev/null
for i in $(seq 1 40); do
  if timeout 15 $SSH $LAP "grep -qE '목표 도달|link closed|Traceback' /tmp/$V.log 2>/dev/null"; then break; fi
  sleep 10
done
timeout 20 $SSH $LAP "pkill -x python3 2>/dev/null; grep -E '목표 도달|Traceback' /tmp/$V.log | tail -2"
timeout 20 $SSH $OM "python3 -c \"
import socket,struct
d=b'<SimCtrl><Stop/></SimCtrl>'
h=struct.pack('<HH64s64sI',40108,1,b'cmd',b'TaskControl',len(d))
s=socket.create_connection(('127.0.0.1',48179),timeout=5); s.sendall(h+d); s.close()\"" 2>/dev/null
timeout 30 $SP scp -o StrictHostKeyChecking=no $LAP:/tmp/$V.csv "/tmp/run_$V.csv" 2>/dev/null
python3 ~/hlfma2026/eval/check_run.py "/tmp/run_$V.csv" 8.33 "$ROUTE" "$LANEP"
