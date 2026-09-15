#!/bin/bash
# 아무 시나리오 이름 + 아무 경로로 주행/채점.  bash run_any.sh <시나리오명> <경로.json>
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
V="${1:?scenario}"; ROUTE="${2:?route}"
LANEP="${ROUTE%.json}_lane.json"
SP="sshpass -p ${OMEN_PW:?OMEN_PW 에 VTD PC 비밀번호를 넣을 것}"
OM="${VTD_SSH:-user@192.168.50.11}"; LAP="${CTRL_SSH:-user@192.168.50.10}"
SSH="$SP ssh -o StrictHostKeyChecking=no -o ConnectTimeout=20"
echo "######## $V  ($(basename $ROUTE))"
# ⚠️ 지난 판 CSV/로그를 **먼저 지운다**. 안 그러면 로드나 기동이 실패했을 때 옛 파일로
#    채점해 '통과'가 나온다(실측 2026-08-16: 4시간 전 CSV 로 채점될 뻔했다).
rm -f "/tmp/run_$V.csv"
timeout 20 $SSH $LAP "rm -f /tmp/$V.csv /tmp/$V.log" || { echo "제어PC 접속 실패"; exit 1; }
$SP scp -o StrictHostKeyChecking=no -q "$ROUTE" $LAP:/tmp/plan.json
$SP scp -o StrictHostKeyChecking=no -q "$LANEP" $LAP:/tmp/plan_lane.json
bash ~/hlfma2026/vtd/ensure_vtd.sh 2>&1 | tail -1
# ⚠️ 로드 실패를 여기서 반드시 잡는다. 놓치면 9910 이 안 열린 채 주행이 떠서
#    "CSV 없음(기동 실패)" 로만 보이고 진짜 원인이 가려진다(실측 2026-08-17, 4판 연속).
# ⚠️⚠️ 실패했다고 **여기서 VTD 를 죽이지 말 것**(2026-08-17에 해봤다가 크게 데었다).
#    `pkill -x taskControl` 뒤 재시작이 안 붙으면 VTD 가 통째로 내려가고, 그 다음
#    판들은 손도 못 댄다. 한 판 건지려다 세션 전체를 날린다.
#    load_scenario.py 안에 이미 streaming 재시도 3회가 있다. 그래도 안 되면 **건너뛴다**.
# 로드는 **VTD 를 건드리지 않고** 두 번까지 시도한다. 강제 재시작 직후 첫 로드가
# 잘 실패하는데(2026-08-17 회귀 4건 중 3건이 재시작 다음 판, v3 표본도 한 판 먹혔다),
# 그건 안정화가 덜 된 것이라 기다렸다 다시 부르면 붙는다.
#  ⚠️ 실패했다고 pkill 로 VTD 를 치지 말 것 — 그러다 통째로 내려서 재부팅까지 갔다.
loaded=0
for try in 1 2; do
  if timeout 120 $SSH $OM "cd ~/hlfma2026/vtd && python3 load_scenario.py $V" >/dev/null 2>&1; then
    loaded=1; break
  fi
  [ $try -eq 1 ] && { echo "  로드 실패(1차) -> 30초 기다렸다 재시도"; sleep 30; }
done
[ $loaded -eq 1 ] || { echo "❌ 로드 실패 — 건너뜀(VTD 는 건드리지 않음)"; exit 1; }
timeout 30 $SSH $LAP "cd ~/hlfma2026/src
for p in \$(pgrep -x python3 2>/dev/null); do kill -9 \$p; done
setsid python3 -u main.py --scenario /tmp/plan.json --port 9910 \
  --tl-map ~/hlfma2026/routes/tl_map_livinglab.json --lane-plan /tmp/plan_lane.json \
  --crosswalks ~/hlfma2026/routes/crosswalks.json \
  --log-csv /tmp/$V.csv >/tmp/$V.log 2>&1 </dev/null &
sleep 2" >/dev/null
# 갇힌 판을 1000초까지 끌지 않는다. 멈춘 채 60초가 지나면 끊는다 — 실측 2026-08-18:
# EV_COMBO_PASSPED 가 매번 1027초를 채워 검증 한 바퀴가 30분씩 날아갔다.
stuck=0
for i in $(seq 1 100); do
  if timeout 15 $SSH $LAP "grep -qE '목표 도달|link closed|Traceback' /tmp/$V.log 2>/dev/null"; then break; fi
  mv=$(timeout 15 $SSH $LAP "tail -60 /tmp/$V.csv 2>/dev/null | awk -F, 'NR>1&&\$5>0.3{n++}END{print n+0}'" 2>/dev/null)
  if [ "${mv:-1}" = "0" ]; then
    stuck=$((stuck+1))
    [ $stuck -ge 6 ] && { echo "  정지 60초 초과 -> 조기 중단"; break; }
  else
    stuck=0
  fi
  sleep 10
done
timeout 20 $SSH $LAP "pkill -x python3 2>/dev/null; grep -E 'Traceback' /tmp/$V.log | tail -1"
timeout 20 $SSH $OM "python3 -c \"
import socket,struct
d=b'<SimCtrl><Stop/></SimCtrl>'
h=struct.pack('<HH64s64sI',40108,1,b'cmd',b'TaskControl',len(d))
s=socket.create_connection(('127.0.0.1',48179),timeout=5); s.sendall(h+d); s.close()\"" 2>/dev/null
timeout 30 $SP scp -o StrictHostKeyChecking=no $LAP:/tmp/$V.csv "/tmp/run_$V.csv" 2>/dev/null
[ -s "/tmp/run_$V.csv" ] || { echo "❌ $V — 주행 CSV 가 없다(기동 실패). 채점 안 함"; exit 1; }
# ⚠️ 프레임율이 무너지면(VTD 과부하) 속도·리스폰 판정이 전부 헛값이 된다.
python3 - "/tmp/run_$V.csv" <<'PY'
import csv, statistics, sys
r = list(csv.DictReader(open(sys.argv[1])))
d = [float(r[i]['t']) - float(r[i-1]['t']) for i in range(1, len(r))]
hz = 1.0 / statistics.median(d) if d else 0
print(f"[프레임율] {hz:.1f} Hz" + ("" if hz > 12 else "  ← ⚠️ 무너짐. 이 판 결과는 못 믿는다"))
PY
python3 ~/hlfma2026/eval/check_run.py "/tmp/run_$V.csv" 8.33 "$ROUTE" "$LANEP"
# ⚠️ 타임아웃까지 간 판(=완주 못 하고 1000초를 채운 판)은 VTD 를 망가뜨린 채 끝난다.
#    실측 2026-08-16: v7(1029초) 직후 판이 두 번 다 **시작조차 못 했다**(0.0Hz).
#    그런 판 뒤에는 무조건 통째로 재시작한다.
if [ "$(python3 -c "import csv,sys
r=list(csv.DictReader(open('/tmp/run_$V.csv')))
print(1 if r and float(r[-1]['t'])>900 else 0)" 2>/dev/null)" = "1" ]; then
  echo "---- 타임아웃 판이었다 -> VTD 강제 재시작"
  # ⚠️ 부분 pkill 로 끝내면 안 된다. moduleManager 만 죽은 상태는 ensure_vtd 의
  #    alive() 가 YES 라 다음 판이 9910 없이 돈다(2026-08-17, 4판 연속 로드 실패).
  bash ~/hlfma2026/vtd/ensure_vtd.sh --force 2>&1 | tail -1
fi

# 여기까지 왔으면 성공이다. 종료코드를 명시한다 — run_regression.sh 가 이걸 보고
# 실패한 판만 재시도한다(마지막 문장이 if 라 암묵적 종료코드에 기대면 안 된다).
exit 0
