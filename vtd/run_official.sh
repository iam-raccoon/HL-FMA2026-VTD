#!/bin/bash
# 공식 시나리오 1개 실행: OMEN에 로드 -> 제어PC에서 직결랜 주행 -> CSV 회수
# lab-main(개발머신)에서 실행. 사용: bash run_official.sh v3 [route.json]
set -u
V="${1:-v1}"
ROUTE="${2:-/tmp/v1.json}"
SP="sshpass -p ${OMEN_PW:?OMEN_PW 에 VTD PC 비밀번호를 넣을 것}"
OM="${VTD_SSH:-user@192.168.50.11}"          # VTD PC
LAP="${CTRL_SSH:-user@192.168.50.10}"         # 제어PC (직결랜 192.168.100.2)
SSH="$SP ssh -o StrictHostKeyChecking=no -o ConnectTimeout=20"

# ★제어PC는 **자기 레포의 src/** 를 돌린다. 로컬에서 고친 걸 안 올리면 옛 코드로 검증하게 된다
#   (실측 2026-08-16: drive.py 를 고쳐놓고 그대로 돌려 '수정 전' 결과를 볼 뻔했다).
echo "=== [$V] 제어PC에 src 배포 ==="
timeout 60 $SP scp -o StrictHostKeyChecking=no -q ~/hlfma2026/src/*.py $LAP:'~/hlfma2026/src/' || exit 1

echo "=== [$V] VTD 로드 ==="
timeout 90 $SSH $OM "cd ~/hlfma2026/vtd && python3 load_scenario.py HL_FMA_VTD_LivingLab_$V" || exit 1

echo "=== [$V] 제어PC 주행 시작 (직결랜) ==="
timeout 30 $SSH $LAP "cd ~/hlfma2026/src
for p in \$(pgrep -x python3 2>/dev/null); do kill -9 \$p; done
setsid python3 -u main.py --scenario $ROUTE --port 9910 \
  --tl-map ~/hlfma2026/routes/tl_map_livinglab.json \
  --lane-plan ~/hlfma2026/routes/lane_plan_v1_v6.json \
  --log-csv /tmp/$V.csv >/tmp/$V.log 2>&1 </dev/null &
sleep 2; head -1 /tmp/$V.log"

echo "=== [$V] 완주 대기 ==="
for i in $(seq 1 40); do
  if timeout 15 $SSH $LAP "grep -qE '목표 도달|link closed|Traceback' /tmp/$V.log 2>/dev/null"; then break; fi
  sleep 10
done
timeout 20 $SSH $LAP "pkill -x python3 2>/dev/null; tail -2 /tmp/$V.log"
# 제어가 끊기면 VTD가 마지막 입력을 유지/경사에서 굴러가므로 시뮬을 세운다
timeout 20 $SSH $OM "python3 -c \"
import socket,struct
d=b'<SimCtrl><Stop/></SimCtrl>'
h=struct.pack('<HH64s64sI',40108,1,b'cmd',b'TaskControl',len(d))
s=socket.create_connection(('127.0.0.1',48179),timeout=5); s.sendall(h+d); s.close()\"" 2>/dev/null
timeout 30 $SP scp -o StrictHostKeyChecking=no $LAP:/tmp/$V.csv "/tmp/run_$V.csv" 2>/dev/null
echo "=== [$V] 채점 ==="
python3 ~/hlfma2026/eval/check_run.py "/tmp/run_$V.csv" 8.33 "$ROUTE"
