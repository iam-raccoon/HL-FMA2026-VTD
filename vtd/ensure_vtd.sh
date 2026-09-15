#!/bin/bash
# VTD 가 정상인지 확인하고, 죽었으면 재시작한다. (개발머신에서 실행)
#
# 왜 필요한가(2026-08-16 실측): 지정차로 회전을 켜면 우리가 좌회전 차로로 옮기는 게
# ghostdriver 에게는 '경로 이탈'이라, 로그에
#     Player Ego is not mapped to a path since more than 5s
# 를 남기고 **Traffic 모듈이 죽는다**(3판쯤에서). 그러면 TaskControl 이
# "waiting for connection <Traffic>" 로 멈춰 이후 모든 시나리오가 timeout 난다.
# 대회는 한 판만 돌리니 무관하지만, 연속 검증할 때는 이게 막힌다.
#
# 사용: bash vtd/ensure_vtd.sh          (시나리오 돌리기 전에 매번)
set -u
SSH="sshpass -p ${OMEN_PW:?OMEN_PW 에 VTD PC 비밀번호를 넣을 것} ssh -o StrictHostKeyChecking=no -o ConnectTimeout=25"
OM=${VTD_SSH:-user@192.168.50.11}

# ⚠️ 여기서 9910(RDB) 을 생존 조건에 넣지 말 것. 실측 2026-08-17: 갓 띄운 VTD 는
#    taskControl·ghostdriver 만 있고 **moduleManager 도 9910 도 없다**(시나리오를
#    로드해야 뜬다). 넣으면 늘 NO 가 되어 무한 재시작한다.
#    대신 9910 확인은 로드 직후(run_any.sh)에서 한다 — 거기서는 떠 있어야 정상이다.
alive() {
  timeout 40 $SSH $OM "pgrep -x taskControl >/dev/null && pgrep ghostdriver >/dev/null \
    && echo YES || echo NO" 2>/dev/null | tail -1
}

# --force: alive() 를 묻지 않고 무조건 재시작한다.
#  ⚠️ 타임아웃 판(1000초) 뒤에 이게 필요하다. 그런 판은 moduleManager 만 죽은 채
#     끝나는데, 그러면 alive() 는 YES 라 재시작을 건너뛰고 **9910 이 없는 채로**
#     다음 판이 돈다. 실측 2026-08-17: 그 상태로 4판 연속 로드 실패했다.
#     부분 pkill 로 alive() 를 NO 로 만들려는 시도는 하지 말 것 — taskControl 만
#     죽였다가 재시작이 안 붙어 VTD 가 통째로 내려간 적이 있다. 정식 절차를 탄다.
[ "${1:-}" = "--force" ] || { [ "$(alive)" = "YES" ] && { echo "VTD OK"; exit 0; }; }

echo "VTD 이상 -> 재시작"
# ⚠️ vtdStop.sh 는 simServer 를 안 죽인다 -> 다음 실행이 라이선스 점유로 죽는다.
#    lmgrd(라이선스 데몬)는 절대 죽이지 말 것.
timeout 100 $SSH $OM "export DISPLAY=:1 XAUTHORITY=/home/user/.Xauthority
~/Hexagon/VTD.2025.2/bin/vtdStop.sh >/dev/null 2>&1; sleep 3
pkill -x moduleManager; pkill -9 -f 'Hexagon/VTD.2025.2/Runtime'; pkill -f startSlave.sh; sleep 4" \
  >/dev/null 2>&1
sleep 5
# ⚠️ DISPLAY 는 :0 이 아니라 :1 이다(who 로 확인). 틀리면 xterm 이 안 떠서 태스크가 죽는다.
timeout 60 $SSH $OM "export DISPLAY=:1 XAUTHORITY=/home/user/.Xauthority
cd ~/Hexagon/VTD.2025.2 && setsid ./bin/vtdStart.sh </dev/null >/tmp/vtdauto.log 2>&1 &
echo started" >/dev/null 2>&1
sleep 85

for _ in 1 2 3; do
  [ "$(alive)" = "YES" ] && { echo "재시작 완료"; exit 0; }
  sleep 20
done
echo "재시작 실패 — docs/ref/vtd_recovery.md 참고"
exit 1
