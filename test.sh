#!/bin/bash
# ★연습 전용 — drive.sh 와 **똑같이 주행하되 시점만 바꿔서** 본다.
#
#     bash ~/hlfma2026/test.sh HL_FMA_NEW_G_TR          # 오른쪽 옆에서(기본)
#     bash ~/hlfma2026/test.sh HL_FMA_NEW_G_TR side_l   # 왼쪽 옆에서
#
# 왜 drive.sh 를 안 고치고 이 파일을 따로 두나(2026-09-06):
#   대회 주행은 **공식 Relative-follower-view** 여야 한다. 확인하려고 drive.sh 의
#   시점을 건드리면 대회날 그대로 나간다. 주행 로직·경로·로그는 drive.sh 그대로 쓰고
#   **카메라만** 뒤에 덮어쓴다. 이 파일은 대회에서 쓰지 않는다.
#
# 무엇이 보이나:
#   공식 시점은 차 뒤 6m·위 2m 라 앞범퍼 앞 1~2m 노면이 차체에 가려 안 보인다.
#   그래서 정지선 1.2m 앞에 서도 화면에서는 선을 밟은 것처럼 보인다.
#   옆에서 보면 실제 간격이 그대로 보인다.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="${1:-}"
VIEW="${2:-side}"

case "$VIEW" in
  side|side_l|follower) ;;
  *) echo "❌ 모르는 시점: $VIEW  (side · side_l · follower)"; exit 1 ;;
esac
[ -n "$NAME" ] || { sed -n '2,9p' "$0" | sed 's/^# \?//'; exit 1; }

HOST="${HOST:-$(python3 "$HERE/src/netcfg.py" --host-only)}"
echo "  ⚠️ 연습 전용 시점: $VIEW  (대회 주행은 drive.sh — 공식 follower view)"

# drive.sh 를 그대로 돌리고(주행·로그·검사 전부 동일), 그 뒤에 카메라만 덮어쓴다.
bash "$HERE/drive.sh" "$NAME" "${@:3}" &   # 3번째부터는 drive.sh 로 그대로(--dry 등)
RUN=$!

# drive.sh 는 exec 직전에 공식 시점을 넣는다. 그 뒤에 덮어야 하므로 잠깐 기다렸다가
# 몇 번 다시 넣는다(Apply·리스폰이 시점을 되돌리는 일이 있다).
( for i in 1 2 3; do
    sleep 3
    kill -0 "$RUN" 2>/dev/null || break
    python3 "$HERE/vtd/set_cams.py" --view "$VIEW" --host "$HOST" >/dev/null 2>&1
  done ) &

wait "$RUN"; RC=$?

# 끝나면 공식 시점으로 되돌린다 — 다음 주행이 엉뚱한 시점으로 시작하지 않게.
python3 "$HERE/vtd/set_cams.py" --view follower --host "$HOST" >/dev/null 2>&1 || true
echo "  시점을 대회 공식(follower)으로 되돌렸다."
exit $RC
