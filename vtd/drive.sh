#!/bin/bash
# 리셋 -> 시뮬 시작(넉넉히 대기) -> 스트리밍 확인 -> 제어(main.py) -> 카메라.
# 사용: bash drive.sh [scenario.json]   (기본 /tmp/HL_FMA_WATCH.json)
SCN="${1:-/tmp/HL_FMA_WATCH.json}"
cd ~/hlfma2026
export DISPLAY=:1 XAUTHORITY=/home/user/.Xauthority

# 기존 제어 종료(ssh 셸 안 죽게 python만)
for p in $(pgrep -f "main.py" 2>/dev/null); do
  [ "$(cat /proc/$p/comm 2>/dev/null)" = python3 ] && kill -9 $p
done

# 시나리오 리셋 + 시작 (넉넉한 대기 = 링크 조기 끊김 방지)
python3 -c "import socket,struct,time
def scp(x):
 d=x.encode();h=struct.pack('<HH64s64sI',40108,1,b'cmd',b'TaskControl',len(d));s=socket.create_connection(('127.0.0.1',48179),timeout=5);s.sendall(h+d);s.close()
scp('<SimCtrl><Stop/></SimCtrl>');time.sleep(1.5)
scp('<SimCtrl><Init/></SimCtrl>');time.sleep(3.5)
scp('<SimCtrl><Start/></SimCtrl>');time.sleep(2.5)"

# 9910 스트리밍 안정 확인(안 되면 한 번 더 Start)
python3 -c "import socket,struct,time
def ok():
 try:
  s=socket.create_connection(('127.0.0.1',9910),timeout=4);s.settimeout(3);b=b''
  while len(b)<1109:
   d=s.recv(1109-len(b));
   if not d: break
   b+=d
  s.close();return len(b)==1109
 except: return False
if not ok():
 h=struct.pack('<HH64s64sI',40108,1,b'cmd',b'TaskControl',13);import socket as so
 c=so.create_connection(('127.0.0.1',48179),timeout=5);c.sendall(h+b'<SimCtrl><Start/></SimCtrl>'[:13]);c.close();time.sleep(2)"

# 제어 실행(detach)
setsid python3 -u main.py --scenario "$SCN" --host 127.0.0.1 --port 9910 >/tmp/run.log 2>&1 </dev/null &

# 카메라(대회 공식 follower view) 적용 - 시작이 덮어쓰는 것 방지 위해 2회
sleep 2; python3 set_cams.py Ego >/dev/null 2>&1
sleep 1; python3 set_cams.py Ego >/dev/null 2>&1
echo "drive.sh: 주행+카메라(공식 follower view) scn=$SCN"
